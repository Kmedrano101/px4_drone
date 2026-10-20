// Ciclo arm - takeoff - hold - descenso - land - disarm con control MPC.
//
// Hereda de TakeoffPositionHoldEv, asi que la mision, la espera del switch del
// RC, la salud del SLAM, la parada por obstaculo, HOLD/ATERRIZAR desde la webui
// y el registro son EXACTAMENTE los mismos que en el nodo PID. Lo unico que
// cambia es como se genera el setpoint:
//
//   PID (por defecto): setpoint de POSICION -> cascada P/PID dentro de PX4.
//   MPC (este nodo):   setpoint de ACELERACION con posicion y velocidad en NaN
//                      -> PX4 apaga sus lazos de posicion y velocidad
//                      (control_mode.cpp) y el MPC es el controlador exterior.
//
// Esa es la condicion para que comparar los dos signifique algo: misma mision,
// mismas guardas, mismos logs, distinto controlador.
//
// El modelo es un triple integrador con el jerk como control (ver
// scripts/mpc/generate_cycle_mpc.py): en Offboard PX4 1.14.3 no suaviza la
// trayectoria, asi que el limite de jerk del MPC es lo que da la rampa que el
// setpoint de posicion no tiene.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <std_msgs/msg/float32_multi_array.hpp>

#include "px4_drone/mpc_cycle.hpp"
#include "px4_drone/takeoff_position_hold_ev.hpp"

class TakeoffHoldLandMpc : public TakeoffPositionHoldEv
{
public:
  TakeoffHoldLandMpc()
  : TakeoffPositionHoldEv("takeoff_hold_land_mpc")
  {
    if (!okToRun()) {
      return;  // la base ya se nego (falta confirm_takeoff, altura, obstaculo...)
    }

    lim_.v_xy = p("mpc_v_xy", 0.6);
    lim_.v_up = p("mpc_v_up", 0.6);
    lim_.v_dn = p("mpc_v_dn", 0.4);
    lim_.a_xy = p("mpc_a_xy", 1.5);
    lim_.a_up = p("mpc_a_up", 1.5);
    lim_.a_dn = p("mpc_a_dn", 1.5);
    lim_.j_max = p("mpc_j_max", 4.0);
    w_.q_pos_xy = p("mpc_q_pos_xy", 30.0);
    w_.q_pos_z = p("mpc_q_pos_z", 60.0);
    w_.q_vel_xy = p("mpc_q_vel_xy", 8.0);
    w_.q_vel_z = p("mpc_q_vel_z", 12.0);
    w_.q_acc = p("mpc_q_acc", 1.0);
    w_.r_jerk = p("mpc_r_jerk", 0.5);
    descent_agl_ = p("mpc_descent_agl_m", 0.25);

    // Observador de perturbacion y compensacion de retardo. Los dos salen
    // APAGADOS (0.0): con los valores por defecto este nodo se comporta
    // exactamente igual que antes, que es la condicion para que la comparacion
    // PID / MPC / MPC+ siga siendo entre lo mismo. El tercer ciclo es este
    // mismo ejecutable con el launch que los activa.
    dist_gain_ = p("mpc_dist_gain", 0.0);
    dist_max_ = p("mpc_dist_max", 1.5);
    dist_min_agl_ = p("mpc_dist_min_agl_m", 0.35);
    delay_comp_s_ = p("mpc_delay_comp_s", 0.0);

    if (dist_gain_ < 0.0 || dist_gain_ > 5.0) {
      refuseToRun("mpc_dist_gain debe estar entre 0 (apagado) y 5.0 1/s.");
      return;
    }
    if (dist_max_ <= 0.0 || dist_max_ > 5.0) {
      refuseToRun("mpc_dist_max debe estar entre 0 y 5.0 m/s2.");
      return;
    }
    if (delay_comp_s_ < 0.0 || delay_comp_s_ > 0.3) {
      refuseToRun("mpc_delay_comp_s debe estar entre 0 (apagado) y 0.3 s.");
      return;
    }

    diag_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("mpc_status", 10);

    mpc_ = std::make_unique<px4_drone::MpcCycle>();
    if (!mpc_->ready()) {
      refuseToRun("no se pudo crear el solver MPC (" + mpc_->error() + ").");
      return;
    }
    mpc_->setWeights(w_);

    if (dist_gain_ > 0.0 || delay_comp_s_ > 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "MPC+ ACTIVO: observador de perturbacion ganancia %.2f 1/s, tope %.2f m/s2, solo por "
        "encima de %.2f m del suelo; compensacion de retardo %.0f ms. Con ganancia 0 y retardo 0 "
        "este nodo es el MPC de siempre.",
        dist_gain_, dist_max_, dist_min_agl_, 1e3 * delay_comp_s_);
    } else {
      RCLCPP_INFO(
        get_logger(),
        "MPC clasico: sin observador de perturbacion ni compensacion de retardo "
        "(mpc_dist_gain = 0, mpc_delay_comp_s = 0).");
    }

    RCLCPP_WARN(
      get_logger(),
      "CONTROL POR MPC: horizonte %d x %.3f s = %.2f s. Publica ACELERACION (posicion y velocidad "
      "en NaN), asi que los lazos de posicion y velocidad de PX4 quedan apagados. Limites: v_xy "
      "%.2f, subida %.2f, bajada %.2f m/s; a %.2f m/s2; jerk %.1f m/s3. Descenso controlado hasta "
      "%.2f m del suelo antes de entregar a PX4.",
      mpc_->horizon(), mpc_->dt(), mpc_->horizon() * mpc_->dt(), lim_.v_xy, lim_.v_up, lim_.v_dn,
      lim_.a_xy, lim_.j_max, descent_agl_);
  }

protected:
  float controlledDescentAgl() const override {return static_cast<float>(descent_agl_);}

  // PX4 solo apaga sus lazos de posicion y velocidad si el heartbeat pide
  // explicitamente aceleracion (control_mode.cpp, rama NAVIGATION_STATE_OFFBOARD).
  void publishOffboardControlMode() override
  {
    publishOffboardControlModeFlags(/*position=*/false, /*velocity=*/false, /*acceleration=*/true);
  }

  // Antes de armar basta con mantener vivo el flujo: aceleracion nula.
  void publishHoldCurrentPosition() override
  {
    publishAccelerationSetpoint(0.0f, 0.0f, 0.0f, headingOrZero());
  }

  void emitTrackingSetpoint(float ned_x, float ned_y, float ned_z, float yaw_ned) override
  {
    resolverYPublicar(ned_x, ned_y, ned_z, yaw_ned, /*frenando=*/false);
  }

  // Frenar = pedir quedarse donde se esta con velocidad cero. El MPC lo resuelve
  // con su limite de jerk, en vez de cortar en seco.
  void publishBrakeSetpoint() override
  {
    const auto * lp = localPosition();
    const float x = lp != nullptr ? lp->x : 0.0f;
    const float y = lp != nullptr ? lp->y : 0.0f;
    resolverYPublicar(x, y, brakeZNed(), targetYawNed(), /*frenando=*/true);
  }

private:
  double p(const std::string & nombre, double defecto)
  {
    return declare_parameter<double>(nombre, defecto);
  }

  float headingOrZero() const
  {
    const auto * lp = localPosition();
    return (lp != nullptr && std::isfinite(lp->heading)) ? lp->heading : 0.0f;
  }

  void resolverYPublicar(float rx, float ry, float rz, float yaw, bool frenando)
  {
    const auto * lp = localPosition();
    if (lp == nullptr || !std::isfinite(lp->z)) {
      publishAccelerationSetpoint(0.0f, 0.0f, 0.0f, yaw);
      return;
    }

    // El suelo como restriccion dura del problema: el MPC no puede planificar
    // por debajo del punto donde se armo.
    lim_.z_floor_ned = originZNed();
    mpc_->setLimits(lim_);

    // Hueco desde el ciclo anterior: si es grande, esto es un ciclo nuevo
    // (rearmado, reinicio del nodo) y el observador tiene que empezar de cero.
    const rclcpp::Time ahora = now();
    const double hueco_s = ultimo_ciclo_.nanoseconds() > 0 ?
      (ahora - ultimo_ciclo_).seconds() : 0.0;
    if (hueco_s <= 0.0 || hueco_s > 0.5) {
      for (int i = 0; i < 3; ++i) {
        d_[i] = 0.0f;
      }
    }
    ultimo_ciclo_ = ahora;

    // Antiguedad del dato que estamos usando: se mide cuando cambia el sello de
    // tiempo del mensaje del FC. Es una cota inferior del retardo real (no
    // incluye lo que tarda dentro del FC), y sirve para elegir
    // mpc_delay_comp_s con datos en vez de a ojo.
    if (lp->timestamp != ultimo_sello_lp_) {
      ultimo_sello_lp_ = lp->timestamp;
      llegada_lp_ = ahora;
    }
    const double edad_lp_s = llegada_lp_.nanoseconds() > 0 ?
      (ahora - llegada_lp_).seconds() : 0.0;

    actualizarPerturbacion(lp, hueco_s);
    mpc_->setDisturbance(d_);

    float p_meas[3] = {lp->x, lp->y, lp->z};
    float v_meas[3] = {lp->vx, lp->vy, lp->vz};

    // Compensacion de retardo: el estado que llega describe donde estaba el dron
    // hace un rato, no donde esta. Se propaga hacia delante con el mismo modelo
    // del MPC antes de usarlo como condicion inicial. Con EKF2_EV_DELAY = 0 y la
    // pose del SLAM 2D a ~10 Hz, el desfase es real y se ve como oscilacion.
    if (delay_comp_s_ > 0.0) {
      const float tau = static_cast<float>(delay_comp_s_);
      for (int i = 0; i < 3; ++i) {
        const float a_ef = a_prev_[i] + d_[i];
        p_meas[i] += v_meas[i] * tau + 0.5f * a_ef * tau * tau;
        v_meas[i] += a_ef * tau;
      }
    }

    const float p_ref[3] = {rx, ry, rz};
    const float v_ref[3] = {0.0f, 0.0f, 0.0f};

    const auto r = mpc_->solve(p_meas, v_meas, a_prev_, p_ref, v_ref);

    float acc[3];
    if (r.ok) {
      fallos_ = 0;
      // PX4 NO satura el setpoint de aceleracion con MPC_ACC_* (issues 18033 y
      // 18855): se recorta aqui, aunque el MPC ya lo tenga como restriccion.
      acc[0] = clamp(r.acc[0], -lim_.a_xy, lim_.a_xy);
      acc[1] = clamp(r.acc[1], -lim_.a_xy, lim_.a_xy);
      acc[2] = clamp(r.acc[2], -lim_.a_up, lim_.a_dn);
    } else {
      // Sin solucion: se desvanece la ultima aceleracion en vez de cortar.
      ++fallos_;
      for (int i = 0; i < 3; ++i) {
        acc[i] = 0.8f * a_prev_[i];
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "MPC sin solucion (status %d, %d seguidos): desvaneciendo la aceleracion.",
        r.status, fallos_);
      if (fallos_ >= kFallosParaAbortar) {
        abortToLand("El MPC lleva " + std::to_string(fallos_) + " ciclos sin solucion.");
      }
    }

    // Coste de computo visible en el log de la prueba: es el numero que decide
    // si el MPC cabe en la Pi 4 o hace falta otro ordenador.
    solve_ms_acc_ += r.solve_ms;
    solve_ms_max_ = std::max(solve_ms_max_, r.solve_ms);
    if (++ciclos_ >= 100) {   // cada 5 s a 20 Hz
      RCLCPP_INFO(
        get_logger(),
        "MPC: solver %.2f ms de media, %.2f ms maximo (%d ciclos); status %d; "
        "perturbacion [%+.2f %+.2f %+.2f] m/s2; edad del estado %.0f ms",
        solve_ms_acc_ / ciclos_, solve_ms_max_, ciclos_, r.status,
        d_[0], d_[1], d_[2], 1e3 * edad_lp_s_);
      ciclos_ = 0;
      solve_ms_acc_ = 0.0;
      solve_ms_max_ = 0.0;
    }

    publishAccelerationSetpoint(acc[0], acc[1], acc[2], yaw);
    for (int i = 0; i < 3; ++i) {
      a_prev_[i] = acc[i];
    }
    edad_lp_s_ = edad_lp_s;
    publicarDiagnostico(r, acc, p_meas, p_ref, lp->dist_bottom, frenando);
  }

  static float clamp(float v, float lo, float hi) {return std::max(lo, std::min(hi, v));}

  // Observador de perturbacion constante: d integra la diferencia entre lo que
  // el dron acelero de verdad y lo que el modelo predecia (a comandada + d).
  // Es la pieza que le falta al triple integrador para no dejar sesgo
  // permanente en el hold cuando hay un error constante de empuje, de masa o de
  // bateria.
  //
  // Se adapta SOLO por encima de mpc_dist_min_agl_m. En tierra no se puede: con
  // el estado de despegue de PX4 por debajo de 'rampup', el FC DESCARTA nuestro
  // setpoint y lo sustituye por 100 m/s2 hacia abajo
  // (MulticopterPositionControl.cpp del tag v1.14.3), asi que el residuo no
  // mide nuestra perturbacion y el observador se iria a su tope.
  void actualizarPerturbacion(
    const px4_msgs::msg::VehicleLocalPosition * lp, double dt_s)
  {
    if (dist_gain_ <= 0.0) {
      return;
    }
    const bool en_vuelo = lp->dist_bottom_valid && std::isfinite(lp->dist_bottom) &&
      lp->dist_bottom > static_cast<float>(dist_min_agl_);
    const bool dato_util = dt_s > 0.0 && dt_s <= 0.5 &&
      std::isfinite(lp->ax) && std::isfinite(lp->ay) && std::isfinite(lp->az);
    if (!en_vuelo || !dato_util || fallos_ > 0) {
      return;   // tambien se congela si el solver va fallando: el residuo no vale
    }
    const float a_meas[3] = {lp->ax, lp->ay, lp->az};
    const float tope = static_cast<float>(dist_max_);
    for (int i = 0; i < 3; ++i) {
      const float residuo = a_meas[i] - (a_prev_[i] + d_[i]);
      d_[i] = clamp(
        d_[i] + static_cast<float>(dist_gain_ * dt_s) * residuo, -tope, tope);
    }
  }

  // Diagnostico para el rosbag: con esto se afina el MPC despues del vuelo y se
  // compara contra el PID (tiempo de solver, aceleracion pedida, error).
  void publicarDiagnostico(
    const px4_drone::MpcResult & r, const float acc[3], const float p_meas[3],
    const float p_ref[3], float dist_bottom, bool frenando)
  {
    std_msgs::msg::Float32MultiArray m;
    static const char * kEtiquetas[] = {
      "solve_ms", "status", "ok", "frenando",
      "acc_x", "acc_y", "acc_z",
      "err_x", "err_y", "err_z",
      "vel_pred_z", "dist_bottom", "z_floor",
      "d_x", "d_y", "d_z", "edad_lp_ms"};
    constexpr size_t kN = sizeof(kEtiquetas) / sizeof(kEtiquetas[0]);
    m.layout.dim.resize(kN);
    for (size_t i = 0; i < kN; ++i) {
      m.layout.dim[i].label = kEtiquetas[i];
      m.layout.dim[i].size = 1;
      m.layout.dim[i].stride = 1;
    }
    m.data = {
      static_cast<float>(r.solve_ms), static_cast<float>(r.status),
      r.ok ? 1.0f : 0.0f, frenando ? 1.0f : 0.0f,
      acc[0], acc[1], acc[2],
      p_ref[0] - p_meas[0], p_ref[1] - p_meas[1], p_ref[2] - p_meas[2],
      r.vel_pred[2], dist_bottom, lim_.z_floor_ned,
      d_[0], d_[1], d_[2], static_cast<float>(1e3 * edad_lp_s_)};
    diag_pub_->publish(m);
  }

  static constexpr int kFallosParaAbortar = 10;  // 0.5 s a 20 Hz

  std::unique_ptr<px4_drone::MpcCycle> mpc_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr diag_pub_;
  px4_drone::MpcLimits lim_;
  px4_drone::MpcWeights w_;
  double descent_agl_{0.25};
  double dist_gain_{0.0};
  double dist_max_{1.5};
  double dist_min_agl_{0.35};
  double delay_comp_s_{0.0};
  float d_[3] = {0.0f, 0.0f, 0.0f};        // perturbacion estimada (NED, m/s^2)
  float a_prev_[3] = {0.0f, 0.0f, 0.0f};
  rclcpp::Time ultimo_ciclo_{0, 0, RCL_ROS_TIME};
  rclcpp::Time llegada_lp_{0, 0, RCL_ROS_TIME};
  uint64_t ultimo_sello_lp_{0};
  double edad_lp_s_{0.0};
  int fallos_{0};
  int ciclos_{0};
  double solve_ms_acc_{0.0};
  double solve_ms_max_{0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakeoffHoldLandMpc>();
  const bool ok = node->okToRun();
  if (ok) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
