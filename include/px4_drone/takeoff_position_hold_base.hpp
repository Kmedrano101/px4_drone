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

  const px4_msgs::msg::VehicleLocalPosition * localPosition() const
  {
    return last_local_position_.get();
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
    kLand,      // entrega el aterrizaje a PX4 (VEHICLE_CMD_NAV_LAND) y espera el desarme
    kFinished
  };

  void onTimer();
  void publishOffboardControlMode();
  void publishTrajectorySetpoint(float ned_x, float ned_y, float ned_z, float yaw_ned);
  void publishHoldCurrentPosition();
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

  static constexpr uint16_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
};

#endif  // PX4_DRONE__TAKEOFF_POSITION_HOLD_BASE_HPP_
