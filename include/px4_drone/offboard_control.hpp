#ifndef PX4_DRONE__OFFBOARD_CONTROL_HPP_
#define PX4_DRONE__OFFBOARD_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl();

private:
  enum class State
  {
    kWarmup,          // streaming setpoints antes de pedir el cambio de modo
    kRequestOffboard,
    kArm,
    kOffboardActive,  // vuelo normal en offboard
    kSimulateFailsafe, // dejamos de publicar para forzar la perdida de heartbeat
    kFinished
  };

  void onTimer();
  void publishOffboardControlMode();
  void publishVehicleAttitudeSetpoint();
  void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicleCommandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
  static std::string navStateToString(uint8_t nav_state);
  static std::string commandResultToString(uint8_t result);

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  State state_{State::kWarmup};
  uint64_t cycle_count_{0};
  uint8_t last_nav_state_{255};
  uint8_t last_arming_state_{0};
  bool failsafe_confirmed_{false};
  bool offboard_confirmed_{false};
  bool arm_wait_started_{false};
  uint64_t arm_wait_start_cycle_{0};

  static constexpr float kLoopRateHz = 20.0f;              // 20 Hz: mas rapido que el minimo de 2 Hz
  static constexpr uint64_t kWarmupCycles = 100;            // 5 s de setpoints antes de cambiar de modo
  static constexpr uint64_t kArmDelayCycles = 60;           // 3 s tras confirmar OFFBOARD (nav_state), se intenta armar
  static constexpr uint64_t kOffboardConfirmTimeoutCycles = 200; // 10 s max esperando confirmacion de OFFBOARD
  static constexpr uint64_t kActiveCyclesBeforeFailsafeTest = 200; // 10 s en OFFBOARD antes del test
  static constexpr uint64_t kFailsafeMonitorCycles = 200;   // 10 s observando la reaccion del FC

  static constexpr uint16_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
};

#endif  // PX4_DRONE__OFFBOARD_CONTROL_HPP_
