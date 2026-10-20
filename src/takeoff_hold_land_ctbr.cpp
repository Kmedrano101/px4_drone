// Ciclo arm -> takeoff -> hold -> land con el NMPC de empuje colectivo y
// velocidades angulares (CTBR) controlando SOLO la fase de hold.
//
// Por que solo el hold, verificado en el fuente del tag v1.14.3: con
// offboard_control_mode.body_rate, control_mode.cpp:169-175 deja
// flag_multicopter_position_control_enabled en false, asi que mc_pos_control NO
// se ejecuta. Desaparecen de golpe la maquina de despegue (con su spoolup y su
// rampa), el limite de inclinacion MPC_TILTMAX_*, el estimador de empuje de
// hover y la sustitucion de nuestro setpoint mientras el dron esta en el suelo.
// El empuje que publicamos va del mezclador a los motores sin nada en medio.
//
// Despegar asi en la primera prueba seria temerario, y ademas no hace falta: lo
// que se quiere medir es el hold de 30 s. El despegue y el aterrizaje los hace
// PX4 con setpoints de posicion, igual que el ciclo PID, y el CTBR se engancha
// cuando el dron ya esta estable en su altura. La mision, las guardas y el
// registro son los de la clase base, como en los otros dos ciclos.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "px4_drone/mpc_ctbr.hpp"
#include "px4_drone/takeoff_position_hold_ev.hpp"

namespace
{
constexpr float kG = 9.80665f;
}

class TakeoffHoldLandCtbr : public TakeoffPositionHoldEv
{
public:
  TakeoffHoldLandCtbr()
  : TakeoffPositionHoldEv("takeoff_hold_land_ctbr")
  {
    if (!okToRun()) {
      return;
    }

    rate_hz_ = p("ctbr_rate_hz", 50.0);
    thr_hover_ = p("ctbr_thr_hover", 0.30);
    thr_min_ = p("ctbr_thr_min", 0.10);
    thr_max_ = p("ctbr_thr_max", 0.75);
    thr_slew_ = p("ctbr_thr_slew_per_s", 1.5);
    engage_delay_s_ = p("ctbr_engage_delay_s", 3.0);
    max_tilt_deg_ = p("ctbr_max_tilt_deg", 20.0);
    lim_.v_xy = p("ctbr_v_xy", 0.6);
    lim_.v_up = p("ctbr_v_up", 0.6);
    lim_.v_dn = p("ctbr_v_dn", 0.4);
    lim_.t_min = p("ctbr_t_min_g", 0.5);
    lim_.t_max = p("ctbr_t_max_g", 1.6);
    lim_.w_xy = p("ctbr_w_xy", 1.0);
    lim_.w_z = p("ctbr_w_z", 0.5);
    w_.q_pos_xy = p("ctbr_q_pos_xy", 30.0);
    w_.q_pos_z = p("ctbr_q_pos_z", 60.0);
    w_.q_att_tilt = p("ctbr_q_att_tilt", 20.0);
    w_.r_rate = p("ctbr_r_rate", 2.0);

    if (rate_hz_ < 20.0 || rate_hz_ > 200.0) {
      refuseToRun("ctbr_rate_hz debe estar entre 20 y 200 Hz.");
      return;
    }
    if (thr_hover_ <= 0.05 || thr_hover_ >= 0.9 || thr_min_ >= thr_max_ ||
      thr_min_ < 0.0 || thr_max_ > 0.95)
    {
      refuseToRun(
        "empuje normalizado invalido: se exige 0.05 < ctbr_thr_hover < 0.9 y "
        "0 <= ctbr_thr_min < ctbr_thr_max <= 0.95.");
      return;
    }

    mpc_ = std::make_unique<px4_drone::MpcCtbr>();
    if (!mpc_->ready()) {
      refuseToRun("no se pudo crear el solver CTBR (" + mpc_->error() + ").");
      return;
    }
    mpc_->setWeights(w_);

    diag_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("ctbr_status", 10);

    rclcpp::QoS qos(5);
    qos.best_effort();
    const std::string v = get_parameter("topic_version_suffix").as_string();
    att_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude" + v, qos,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr m) {
        last_att_ = m;
        last_att_time_ = now();
      });

    // Lazo rapido propio: el de la mision va a 20 Hz, que no basta para cerrar
    // actitud desde la Pi. Al comandar tasas, el lazo de actitud es nuestro.
    fast_timer_ = create_wall_timer(
      std::chrono::microseconds(static_cast<int64_t>(1e6 / rate_hz_)),
      [this]() {lazoRapido();});

    RCLCPP_WARN(
      get_logger(),
      "CTBR: NMPC con actitud (horizonte %d x %.3f s = %.2f s) a %.0f Hz. Publica TASAS + EMPUJE, "
      "asi que con body_rate PX4 NO ejecuta mc_pos_control: sin maquina de despegue, sin limite "
      "de inclinacion y sin estimador de empuje. Se engancha SOLO en el hold, %.1f s despues de "
      "entrar, y el despegue y el aterrizaje los hace PX4 con setpoints de posicion. Empuje de "
      "hover %.2f normalizado (recorte %.2f-%.2f, rampa %.1f /s); se suelta si la inclinacion "
      "pasa de %.0f grados.",
      mpc_->horizon(), mpc_->dt(), mpc_->horizon() * mpc_->dt(), rate_hz_, engage_delay_s_,
      thr_hover_, thr_min_, thr_max_, thr_slew_, max_tilt_deg_);
  }

protected:
  // El heartbeat decide que lazos usa PX4. Solo pedimos body_rate mientras
  // estamos realmente enganchados; el resto del ciclo es posicion, como el PID.
  void publishOffboardControlMode() override
  {
    if (enganchado_) {
      publishOffboardControlModeFlags(false, false, false, /*body_rate=*/true);
    } else {
      publishOffboardControlModeFlags(/*position=*/true, false, false);
    }
  }

  void emitTrackingSetpoint(float x, float y, float z, float yaw) override
  {
    ref_[0] = x;
    ref_[1] = y;
    ref_[2] = z;
    ref_yaw_ = yaw;
    ref_valida_ = true;

    actualizarEngache();

    if (!enganchado_) {
      // Lo publica la base: setpoint de posicion y lo cierra la cascada de PX4.
      TakeoffPositionHoldEv::emitTrackingSetpoint(x, y, z, yaw);
    }
    // Enganchados no se publica nada aqui: lo hace el lazo rapido.
  }

  // Frenar por obstaculo: se suelta el CTBR y lo resuelve PX4. Es la fase en la
  // que menos interesa estar experimentando.
  void publishBrakeSetpoint() override
  {
    soltar("frenado por obstaculo");
    TakeoffPositionHoldEv::publishBrakeSetpoint();
  }

private:
  double p(const std::string & nombre, double defecto)
  {
    return declare_parameter<double>(nombre, defecto);
  }

  float inclinacionDeg() const
  {
    if (last_att_ == nullptr) {
      return 180.0f;
    }
    const auto & q = last_att_->q;   // [w, x, y, z]
    // Angulo entre el z del cuerpo y la vertical: cos(tilt) = 1 - 2(qx^2+qy^2)
    const float c = std::clamp(1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]), -1.0f, 1.0f);
    return std::acos(c) * 180.0f / static_cast<float>(M_PI);
  }

  bool datosFrescos() const
  {
    if (last_att_ == nullptr || last_att_time_.nanoseconds() == 0) {
      return false;
    }
    return (now() - last_att_time_).seconds() < 0.2;
  }

  void actualizarEngache()
  {
    const bool en_hold = inHoldPhase();
    if (!en_hold) {
      if (enganchado_) {
        soltar("el ciclo salio de la fase de hold");
      }
      hold_desde_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return;
    }
    if (hold_desde_.nanoseconds() == 0) {
      hold_desde_ = now();
    }
    if (enganchado_ || no_reenganchar_) {
      return;
    }
    if ((now() - hold_desde_).seconds() < engage_delay_s_) {
      return;   // deja que PX4 estabilice la altura antes de tomar el mando
    }
    if (!datosFrescos() || localPosition() == nullptr) {
      return;
    }
    if (inclinacionDeg() > max_tilt_deg_) {
      return;
    }
    // Se arranca desde el empuje de hover y se deja que la rampa lo mueva: asi
    // el cambio de controlador no mete un escalon de empuje.
    if (!offboardArmed()) {
      return;   // en el hold pero sin control: no se toma nada
    }
    thr_pub_ = static_cast<float>(thr_hover_);
    enganchado_ = true;
    // Heartbeat de body_rate ya, no en el proximo ciclo de 20 Hz: hasta que PX4
    // lo reciba, su propio controlador de actitud sigue publicando
    // vehicle_rates_setpoint y habria dos emisores en el mismo topic.
    publishOffboardControlModeFlags(false, false, false, /*body_rate=*/true);
    RCLCPP_WARN(
      get_logger(),
      "CTBR ENGANCHADO: a partir de ahora el lazo de actitud es nuestro (tasas + empuje a %.0f Hz). "
      "PX4 solo cierra tasas.", rate_hz_);
  }

  void soltar(const std::string & motivo)
  {
    if (!enganchado_) {
      return;
    }
    enganchado_ = false;
    // Definitivo para lo que queda de ciclo. Soltar y volver a tomar el mando
    // a tirones sobre una condicion marginal seria peor que cualquiera de las
    // dos opciones por separado; el resto del hold lo vuela PX4.
    no_reenganchar_ = true;
    if (!offboardArmed()) {
      // El control ya no es nuestro (piloto, failsafe o desarme). No se publica
      // NADA mas: ni posicion ni heartbeat. Dejar de hablar es la entrega.
      RCLCPP_ERROR(
        get_logger(), "CTBR SOLTADO (%s) con el control ya perdido: se deja de publicar.",
        motivo.c_str());
      return;
    }
    RCLCPP_WARN(
      get_logger(), "CTBR SOLTADO (%s): se vuelve a setpoints de posicion y manda PX4.",
      motivo.c_str());
    // En este mismo ciclo, no en el siguiente: el controlador de tasas de PX4
    // obedece el ultimo setpoint recibido hasta que el lazo de posicion vuelva.
    publishOffboardControlModeFlags(/*position=*/true, false, false);
    publishHoldCurrentPosition();
  }

  void lazoRapido()
  {
    if (!enganchado_) {
      return;
    }
    // Primero la fase, antes que los datos: si el ciclo ya no esta en el hold
    // (termino el tiempo, salto un obstaculo, HOLD/ATERRIZAR desde la webui, o
    // controlLostDuringFlight() lo dejo en kFinished porque el piloto tomo el
    // mando), seguir publicando tasas seria pelear con quien controle ahora.
    // La base solo lo detecta a 20 Hz; este lazo va a 50 y no puede esperar.
    if (!inHoldPhase()) {
      soltar("el ciclo ya no esta en el hold");
      return;
    }
    // Si el lazo se queda sin datos o sin solucion, soltar es mas seguro que
    // seguir repitiendo la ultima tasa: el controlador de tasas de PX4 obedece
    // el ultimo setpoint recibido indefinidamente.
    if (!datosFrescos()) {
      soltar("actitud obsoleta (> 200 ms)");
      return;
    }
    const auto * lp = localPosition();
    if (lp == nullptr || !std::isfinite(lp->z) || !ref_valida_) {
      soltar("sin posicion valida");
      return;
    }
    const float tilt = inclinacionDeg();
    if (tilt > max_tilt_deg_) {
      soltar("inclinacion " + std::to_string(static_cast<int>(tilt)) + " grados");
      return;
    }

    lim_.z_floor_ned = originZNed();
    mpc_->setLimits(lim_);

    const float pm[3] = {lp->x, lp->y, lp->z};
    const float vm[3] = {lp->vx, lp->vy, lp->vz};
    const float qm[4] = {
      last_att_->q[0], last_att_->q[1], last_att_->q[2], last_att_->q[3]};

    const auto r = mpc_->solve(pm, vm, qm, ref_, ref_yaw_);
    if (!r.ok) {
      if (++fallos_ >= kFallosParaSoltar) {
        soltar("el solver lleva " + std::to_string(fallos_) + " ciclos sin solucion");
      }
      return;   // sin solucion no se publica nada nuevo
    }
    fallos_ = 0;

    // Empuje especifico (m/s^2) -> normalizado. Mapeo LINEAL con el hover como
    // ancla: hover (T = g) -> ctbr_thr_hover. Es aproximado y depende de
    // THR_MDL_FAC, que en este FC esta en 0 (mapeo lineal del acelerador,
    // mientras una helice es cuadratica). HAY QUE CALIBRARLO en banco antes de
    // fiarse de el; esta como parametro por eso.
    const float objetivo = std::clamp(
      static_cast<float>(thr_hover_) * r.thrust_sp / kG,
      static_cast<float>(thr_min_), static_cast<float>(thr_max_));
    const float paso = static_cast<float>(thr_slew_ / rate_hz_);
    thr_pub_ += std::clamp(objetivo - thr_pub_, -paso, paso);

    publishRatesSetpoint(r.rates[0], r.rates[1], r.rates[2], thr_pub_);

    solve_ms_acc_ += r.solve_ms;
    solve_ms_max_ = std::max(solve_ms_max_, r.solve_ms);
    if (++ciclos_ >= static_cast<int>(5.0 * rate_hz_)) {   // cada 5 s
      RCLCPP_INFO(
        get_logger(),
        "CTBR: solver %.2f ms de media, %.2f ms maximo (%d ciclos a %.0f Hz); empuje %.3f "
        "normalizado; inclinacion %.1f grados",
        solve_ms_acc_ / ciclos_, solve_ms_max_, ciclos_, rate_hz_, thr_pub_, tilt);
      ciclos_ = 0;
      solve_ms_acc_ = 0.0;
      solve_ms_max_ = 0.0;
    }
    publicarDiagnostico(r, tilt);
  }

  void publicarDiagnostico(const px4_drone::CtbrResult & r, float tilt)
  {
    std_msgs::msg::Float32MultiArray m;
    static const char * kEtiquetas[] = {
      "solve_ms", "status", "enganchado",
      "thrust_sp_ms2", "thrust_norm",
      "w_x", "w_y", "w_z", "tilt_deg"};
    constexpr size_t kN = sizeof(kEtiquetas) / sizeof(kEtiquetas[0]);
    m.layout.dim.resize(kN);
    for (size_t i = 0; i < kN; ++i) {
      m.layout.dim[i].label = kEtiquetas[i];
      m.layout.dim[i].size = 1;
      m.layout.dim[i].stride = 1;
    }
    m.data = {
      static_cast<float>(r.solve_ms), static_cast<float>(r.status), 1.0f,
      r.thrust_sp, thr_pub_,
      r.rates[0], r.rates[1], r.rates[2], tilt};
    diag_pub_->publish(m);
  }

  static constexpr int kFallosParaSoltar = 5;

  std::unique_ptr<px4_drone::MpcCtbr> mpc_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr att_sub_;
  rclcpp::TimerBase::SharedPtr fast_timer_;
  px4_msgs::msg::VehicleAttitude::SharedPtr last_att_;
  rclcpp::Time last_att_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time hold_desde_{0, 0, RCL_ROS_TIME};

  px4_drone::CtbrLimits lim_;
  px4_drone::CtbrWeights w_;
  double rate_hz_{50.0};
  double thr_hover_{0.30};
  double thr_min_{0.10};
  double thr_max_{0.75};
  double thr_slew_{1.5};
  double engage_delay_s_{3.0};
  double max_tilt_deg_{20.0};

  bool enganchado_{false};
  bool no_reenganchar_{false};
  bool ref_valida_{false};
  float ref_[3] = {0.0f, 0.0f, 0.0f};
  float ref_yaw_{0.0f};
  float thr_pub_{0.30f};
  int fallos_{0};
  int ciclos_{0};
  double solve_ms_acc_{0.0};
  double solve_ms_max_{0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakeoffHoldLandCtbr>();
  const bool ok = node->okToRun();
  if (ok) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
