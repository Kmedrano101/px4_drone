#ifndef PX4_DRONE__OFFBOARD_POSITION_HANDSHAKE_HPP_
#define PX4_DRONE__OFFBOARD_POSITION_HANDSHAKE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

// Handshake minimo de OFFBOARD en modo POSICION (a diferencia de
// offboard_control.cpp, que usa modo actitud): valida que el nodo,
// la aceptacion del modo por el FC y la maquina de estados funcionan
// tal cual las usaria takeoff_position_hold_*, pero sin comandar ningun
// despegue -- el setpoint de posicion se mantiene fijo en el origen NED
// (0,0,0) todo el tiempo. Pensado para correr SIN helices: valida
// exclusivamente la secuencia de comunicacion/armado/desarmado.
class OffboardPositionHandshake : public rclcpp::Node
{
public:
  explicit OffboardPositionHandshake(float default_hold_seconds = 5.0f);

private:
  enum class State
  {
    kWarmup,          // streaming setpoints antes de pedir el cambio de modo
    kRequestOffboard,
    kArm,
    kActive,          // armado, hold fijo en el origen, sin despegar
    kDisarm,          // pedido el desarme, esperando confirmacion
    kFinished
  };

  void onTimer();
  void publishOffboardControlMode();
  void publishTrajectorySetpoint();
  void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicleCommandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
  static std::string navStateToString(uint8_t nav_state);
  static std::string commandResultToString(uint8_t result);

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  State state_{State::kWarmup};
  uint64_t cycle_count_{0};
  uint8_t last_nav_state_{255};
  uint8_t last_arming_state_{0};
  bool offboard_confirmed_{false};
  bool arm_confirmed_{false};
  bool disarm_confirmed_{false};
  bool arm_wait_started_{false};
  uint64_t arm_wait_start_cycle_{0};
  bool external_disarm_detected_{false};

  const uint64_t active_cycles_;  // hold_seconds * kLoopRateHz -- ver constructor

  static constexpr float kLoopRateHz = 10.0f;               // 10 Hz pedido explicitamente
  static constexpr uint64_t kWarmupCycles = 50;              // 5 s de setpoints antes de cambiar de modo
  static constexpr uint64_t kArmDelayCycles = 30;             // 3 s tras confirmar OFFBOARD, se intenta armar
  static constexpr uint64_t kOffboardConfirmTimeoutCycles = 100; // 10 s max esperando confirmacion de OFFBOARD
  static constexpr uint64_t kArmConfirmTimeoutCycles = 50;    // 5 s max esperando confirmacion de ARMED
  static constexpr uint64_t kDisarmConfirmTimeoutCycles = 50; // 5 s max esperando confirmacion de DISARMED

  static constexpr uint16_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
};

#endif  // PX4_DRONE__OFFBOARD_POSITION_HANDSHAKE_HPP_
