#ifndef PX4_DRONE__TAKEOFF_POSITION_HOLD_BASE_HPP_
#define PX4_DRONE__TAKEOFF_POSITION_HOLD_BASE_HPP_

#include <cstddef>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

// Base comun para nodos de despegue + mantenimiento de posicion en OFFBOARD.
//
// Maquina de estados:
//   kWaitPositionSource -> kWarmup -> kWaitRcOffboard -> kRequestOffboard
//   -> kArm -> kTakeoff -> kHold -> [kPattern] -> kLand -> kFinished
//   (kTakeoff/kHold/kPattern) -> kObstacleStop -> kLand   si obstacleTooClose()
//   (kTakeoff/kHold/kPattern) -> kUserHold [-> kLand]      con SIGUSR1 (HOLD de la webui)
//   cualquier estado de vuelo  -> kLand                     con SIGUSR2 (ATERRIZAR de la webui)
//
// La webui manda las senales directo al proceso del nodo: es instantaneo, no
// hace falta levantar ROS para publicar nada, y no corta el SLAM, el puente EV
// ni el enlace DDS con el FC (el STOP de antes mataba todo, y en vuelo eso
// dispara el failsafe de Offboard del FC). Antes de armar, cualquiera de las
// dos cancela la prueba sin armar.
//
// kWaitRcOffboard exige que el switch de modo del RC ya este puesto en
// OFFBOARD (nav_state_user_intention) antes de pedir el cambio de modo: el
// nodo nunca "fuerza" OFFBOARD con el switch en otra posicion, es el piloto
// quien decide entrar. Ver controlLostDuringFlight() para la salida
// simetrica: si el piloto mueve el switch fuera de OFFBOARD durante el
// vuelo, el nodo deja de publicar de inmediato.
//
// kPattern es opcional: solo se entra si pattern_distance_m > 0. Recorre las
// cuatro direcciones cardinales del CUERPO del dron volviendo al centro entre
// cada una (adelante, centro, atras, centro, izquierda, centro, derecha,
// centro), a la altura del hold y con el yaw congelado. Con la distancia a 0 el
// nodo se comporta exactamente como antes.
//
// Las subclases solo definen que fuente de posicion deben verificar antes de
// arrancar (GPS para exterior, estimador local con flujo optico + lidar para
// interior) via positionSourceReady()/positionSourceName(). Toda la logica de
// confirmacion de OFFBOARD, ARM, despegue relativo al punto de armado,
// mantenimiento y aterrizaje es identica para ambas variantes.
//
// Los setpoints se definen en variables ENU (convencion ROS 2, REP-103) y se
// convierten a NED (convencion PX4) justo antes de publicarlos, usando
// px4_drone::frames.
//
// COMO SE GENERA EL SETPOINT (punto de extension para el MPC): la maquina de
// estados no publica directamente, llama a emitTrackingSetpoint() /
// publishBrakeSetpoint() / publishOffboardControlMode(), que son virtuales. Por
// defecto emiten un setpoint de POSICION y PX4 cierra el lazo con su cascada
// P/PID. takeoff_hold_land_mpc los sobreescribe para resolver un MPC y emitir
// ACELERACION (con posicion y velocidad en NaN), con lo que PX4 apaga sus lazos
// de posicion y velocidad (control_mode.cpp) y el MPC pasa a ser el controlador
// exterior. Mision, guardas y registro son identicos en los dos: esa es la
// condicion para que la comparacion PID vs MPC signifique algo.
class TakeoffPositionHoldBase : public rclcpp::Node
{
public:
  TakeoffPositionHoldBase(
    const std::string & node_name, float default_takeoff_height_m, float default_hold_seconds);

  // false si el nodo se nego a arrancar por un chequeo de seguridad (falta
  // confirm_takeoff, o parametros invalidos). En ese caso no se crearon
  // publishers/subscribers/timer: el llamador debe evitar spin() y salir.
  bool okToRun() const {return ok_to_run_;}

protected:
  virtual bool positionSourceReady() const = 0;
  virtual std::string positionSourceName() const = 0;

  // Chequeo continuo durante kTakeoff/kHold/kPattern (a diferencia de
  // positionSourceReady(), que solo se evalua una vez antes de despegar).
  // Default true: indoor/outdoor no lo sobreescriben, ya que PX4 mismo
  // dispara un cambio de nav_state/failsafe si su fuente de posicion se
  // invalida (detectado por controlLostDuringFlight()). Las subclases cuya
  // fuente de posicion puede degradarse sin que PX4 lo note como failsafe
  // (ej. external vision con otra ayuda de respaldo todavia activa) la
  // sobreescriben. Si devuelve false, el nodo aterriza (a diferencia de
  // controlLostDuringFlight(), que solo deja de publicar): aca el nodo
  // todavia tiene el control, tiene que soltarlo el mismo.
  virtual bool positionSourceHealthyDuringFlight() const {return true;}

  // Parada por obstaculo. Default: sin sensor de obstaculos, nunca dispara.
  // obstacleClearForTakeoff() se evalua justo antes de armar: si devuelve
  // false, no se arma. obstacleTooClose() se evalua en cada ciclo de
  // kTakeoff/kHold/kPattern: si devuelve true, el nodo frena (velocidad
  // horizontal cero) y aterriza. En ambos, reason explica el motivo.
  virtual bool obstacleClearForTakeoff(std::string * reason) const
  {
    (void)reason;
    return true;
  }
  virtual bool obstacleTooClose(std::string * reason) const
  {
    (void)reason;
    return false;
  }

  // --- Generacion del setpoint. Virtuales: ver la nota de arriba.
  // Setpoint para ir/mantenerse en (NED x, y, z) con ese yaw.
  virtual void emitTrackingSetpoint(float ned_x, float ned_y, float ned_z, float yaw_ned);
  // Frenado: parar en horizontal manteniendo la altura.
  virtual void publishBrakeSetpoint();
  // Que lazos de PX4 se usan (position por defecto; el MPC pide acceleration).
  virtual void publishOffboardControlMode();
  // Lo que se publica antes de armar, mientras se calienta el enlace offboard.
  virtual void publishHoldCurrentPosition();

  // Publica un setpoint de POSICION (lo que usa el controlador por defecto).
  void publishTrajectorySetpoint(float ned_x, float ned_y, float ned_z, float yaw_ned);

  // Descenso controlado antes del aterrizaje: metros sobre el suelo a los que
  // el controlador baja el dron antes de entregar el contacto a PX4
  // (VEHICLE_CMD_NAV_LAND, que tiene detector de aterrizaje y desarma). 0 = no
  // hay descenso controlado, se entrega a PX4 desde la altura de vuelo, que es
  // lo que hace el nodo PID.
  virtual float controlledDescentAgl() const {return 0.0f;}

  // Aterrizaje forzado (lo usan las subclases cuando su controlador falla).
  void abortToLand(const std::string & reason);

  // Publica el heartbeat de offboard pidiendo esos lazos. PX4 decide con esto
  // que controladores activa (control_mode.cpp).
  void publishOffboardControlModeFlags(bool position, bool velocity, bool acceleration);

  // Publica un setpoint de ACELERACION (NED, m/s^2) con posicion y velocidad en
  // NaN. En PX4 1.14.3 una componente NaN no influye (PositionControl.cpp) y,
  // con offboard_control_mode.acceleration, los lazos de posicion y velocidad
  // quedan desactivados: el que controla es quien manda la aceleracion.
  void publishAccelerationSetpoint(float ned_ax, float ned_ay, float ned_az, float yaw_ned);

  // Altura objetivo (NED) y yaw congelado de la fase actual, para las subclases.
  float targetZNed() const {return target_z_ned_;}
  float targetYawNed() const {return target_yaw_ned_;}
  float originZNed() const {return origin_z_ned_;}
  float brakeZNed() const {return brake_z_ned_;}

  const px4_msgs::msg::VehicleLocalPosition * localPosition() const
  {
    return last_local_position_.get();
  }

  float takeoffHeightM() const {return takeoff_height_m_;}

  // Permite que una subclase rechace arrancar desde el cuerpo de su propio
  // constructor (despues de construir la base), para chequeos que dependen
  // de parametros propios de la subclase. No se puede hacer con un virtual
  // llamado desde el constructor de la base: la vtable de la subclase
  // todavia no esta lista durante la construccion de la base.
  void refuseToRun(const std::string & reason)
  {
    RCLCPP_FATAL(get_logger(), "%s Nodo detenido.", reason.c_str());
    ok_to_run_ = false;
  }

private:
  enum class State
  {
    kWaitPositionSource,  // esperando que la fuente de posicion (GPS o flujo optico+lidar) este sana
    kWarmup,              // streaming de setpoints antes de pedir el cambio de modo
    kWaitRcOffboard,      // esperando que el piloto ponga el switch del RC en OFFBOARD
    kRequestOffboard,
    kArm,
    kTakeoff,   // sube en linea recta desde el punto de armado hasta la altura objetivo
    kHold,      // mantiene esa posicion por hold_seconds_
    kPattern,   // recorre las 4 direcciones cardinales volviendo al centro entre cada una
    kObstacleStop,  // frena (velocidad horizontal cero, altura mantenida) y despues aterriza
    kUserHold,      // HOLD pedido desde la webui: frena y se queda quieto hasta ATERRIZAR o timeout
    kControlledDescent,  // baja con el propio controlador hasta controlledDescentAgl() y entrega a PX4
    kLand,      // entrega el aterrizaje a PX4 (VEHICLE_CMD_NAV_LAND) y espera el desarme
    kFinished
  };

  void onTimer();
  void publishVehicleCommand(
    uint16_t command, float param1 = 0.0f, float param2 = 0.0f, float param4 = 0.0f,
    float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void vehicleCommandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
  // true si el FC ya no esta en OFFBOARD o reporta failsafe: el piloto (RC)
  // o el propio FC tomaron el control. En ese caso deja de publicar
  // setpoints/heartbeat offboard y pasa a kFinished sin tocar NAV_LAND ni
  // ARM -- publicar heartbeat offboard con el control perdido es lo que
  // permite que el FC vuelva a OFFBOARD solo en cuanto la condicion que
  // disparo el failsafe desaparece (ver reporte de vuelo real 2026-09-11,
  // "riesgo grave: reentrada automatica en OFFBOARD").
  bool controlLostDuringFlight();

  // El switch del RC tiene que seguir en OFFBOARD hasta el momento de armar.
  // kWaitRcOffboard lo comprueba una vez, pero despues pasan hasta 13 s (10 de
  // espera de la confirmacion de OFFBOARD + 3 de kArmDelayCycles) en los que el
  // piloto puede mover el switch, y offboard_confirmed_ es un pestillo que no
  // se suelta. Sin esto el nodo armaba con el switch ya fuera de OFFBOARD.
  bool rcOffboardIntentLost();
  // Pide NAV_LAND y pasa a kLand. Comentario historico: a diferencia de
  // controlLostDuringFlight() (que solo deja de publicar porque el control ya
  // no es nuestro), aca el nodo todavia tiene el control y lo suelta el mismo.
  // Deja de seguir el setpoint de posicion y frena en seco en horizontal. No
  // usa la posicion del EKF a proposito: si el obstaculo aparece porque la
  // posicion (EV/SLAM) miente, mantener "la posicion actual" seguiria
  // arrastrando al dron. Ver obstacle_guard.hpp.
  void enterObstacleStop(const std::string & reason);
  // Aplica un pedido pendiente de la webui (SIGUSR1/SIGUSR2). true = el
  // estado cambio y el ciclo actual no debe seguir.
  bool handleUserRequest();
  void enterUserHold();
  // Fin de mision: descenso controlado si la subclase lo pide, o NAV_LAND.
  void startLandingSequence(const std::string & motivo);
  std::string fmtHoldTimeout() const;
  // Chequeos de vuelo comunes a kTakeoff/kHold/kPattern. true = el estado
  // cambio (control perdido, fuente de posicion caida u obstaculo) y el
  // llamador tiene que salir del ciclo sin publicar nada mas.
  bool flightChecksFailed();
  // Destino del tramo `leg` en NED. Los desplazamientos se definen en ejes del
  // CUERPO (adelante / derecha) y se giran con el yaw congelado del hold, para
  // que "adelante" sea hacia donde apunta el morro y no hacia el norte.
  void patternTargetNed(size_t leg, float * ned_x, float * ned_y) const;
  static const char * patternLegName(size_t leg);
  static size_t patternLegCount();

  static std::string navStateToString(uint8_t nav_state);
  static std::string commandResultToString(uint8_t result);

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  px4_msgs::msg::VehicleLocalPosition::SharedPtr last_local_position_;

  bool ok_to_run_{true};
  State state_{State::kWaitPositionSource};
  uint64_t cycle_count_{0};
  uint8_t last_nav_state_{255};
  uint8_t last_nav_state_user_intention_{255};
  uint8_t last_arming_state_{0};
  bool last_failsafe_{false};
  bool offboard_confirmed_{false};
  bool arm_wait_started_{false};
  uint64_t arm_wait_start_cycle_{0};
  bool arm_command_sent_{false};
  uint64_t arm_sent_cycle_{0};

  // Origen NED capturado en el momento de armar: el despegue/hold son
  // relativos a este punto, no a un origen global.
  float origin_x_ned_{0.0f};
  float origin_y_ned_{0.0f};
  float origin_z_ned_{0.0f};
  float target_x_ned_{0.0f};
  float target_y_ned_{0.0f};
  float target_z_ned_{0.0f};
  float target_yaw_ned_{0.0f};

  // Centro del patron: la posicion que se mantuvo durante el hold. Se guarda
  // aparte de target_*_ned_ porque esos si se mueven al recorrer los tramos.
  float center_x_ned_{0.0f};
  float center_y_ned_{0.0f};
  size_t pattern_leg_{0};
  bool pattern_arrived_{false};
  uint64_t pattern_arrived_cycle_{0};

  const float takeoff_height_m_;  // metros a subir en ENU (Z hacia arriba) desde el punto de armado
  const float hold_seconds_;
  const float pattern_distance_m_;    // 0 = sin patron, se aterriza justo despues del hold
  const float pattern_settle_seconds_;  // pausa al llegar a cada punto, para que se estabilice
  const float obstacle_brake_seconds_;  // frenado antes de pedir LAND tras un obstaculo

  float brake_z_ned_{0.0f};  // altura a mantener mientras se frena
  const float user_hold_timeout_s_;  // HOLD de la webui: aterriza solo pasado este tiempo
  bool user_hold_captured_{false};   // ya freno y fijo el punto de hold
  float user_hold_x_ned_{0.0f};
  float user_hold_y_ned_{0.0f};
  std::string obstacle_reason_;

  static constexpr float kLoopRateHz = 20.0f;
  static constexpr uint64_t kPositionSourceTimeoutCycles = 200;  // 10 s
  static constexpr uint64_t kWarmupCycles = 100;                 // 5 s
  static constexpr uint64_t kRcOffboardIntentTimeoutCycles = 200; // 10 s esperando el switch del RC
  static constexpr uint64_t kArmDelayCycles = 60;                // 3 s tras confirmar OFFBOARD
  static constexpr uint64_t kOffboardConfirmTimeoutCycles = 200; // 10 s
  static constexpr uint64_t kArmConfirmTimeoutCycles = 100;      // 5 s tras enviar el comando de ARM
  static constexpr uint64_t kTakeoffTimeoutCycles = 300;         // 15 s max para alcanzar la altura objetivo
  static constexpr float kAltitudeToleranceM = 0.15f;
  static constexpr uint64_t kLandTimeoutCycles = 1200;           // 60 s max esperando desarme
  static constexpr float kPatternToleranceM = 0.20f;             // radio para dar un tramo por alcanzado
  static constexpr uint64_t kPatternLegTimeoutCycles = 240;      // 12 s max por tramo

  static constexpr uint64_t kDescentTimeoutCycles = 600;  // 30 s bajando: algo va mal
  static constexpr float kDescentReachedM = 0.10f;       // margen para dar la altura por alcanzada

  static constexpr float kUserHoldBrakeSeconds = 1.0f;  // frenado antes de fijar el punto de hold

  static constexpr uint16_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
};

#endif  // PX4_DRONE__TAKEOFF_POSITION_HOLD_BASE_HPP_
