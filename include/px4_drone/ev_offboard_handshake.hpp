#ifndef PX4_DRONE__EV_OFFBOARD_HANDSHAKE_HPP_
#define PX4_DRONE__EV_OFFBOARD_HANDSHAKE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

// Variante de offboard_position_handshake.cpp para el LiDAR 2D (SLAM ->
// external vision): valida que se puede armar y mantener OFFBOARD exigiendo
// el LiDAR 2D como fuente de posicion, sin comandar ningun despegue -- el
// setpoint se mantiene fijo en el origen NED (0,0,0), igual que el
// handshake original.
//
// No es una subclase de OffboardPositionHandshake ni de
// TakeoffPositionHoldBase a proposito: offboard_position_handshake.cpp ya
// esta volado y verificado en hardware real (kill switch confirmado), y
// tocarlo para agregarle una precondicion es mas riesgo que beneficio
// comparado con un nodo nuevo. Este nodo si incorpora los dos guards de
// seguridad agregados a TakeoffPositionHoldBase la semana pasada
// (kWaitRcOffboard y el equivalente de controlLostDuringFlight()), porque
// omitirlos en un nodo nuevo a sabiendas seria inconsistente con lo que ya
// se aprendio de un casi-accidente real.
//
// Maquina de estados:
//   kWaitEvReady -> kWarmup -> kWaitRcOffboard -> kRequestOffboard -> kArm
//   -> kActive -> kDisarm -> kFinished
class EvOffboardHandshake : public rclcpp::Node
{
public:
  explicit EvOffboardHandshake(float default_hold_seconds = 10.0f);

private:
  enum class State
  {
    kWaitEvReady,     // esperando xy/z_valid, heading_good_for_control, eph y odometria EV fresca
    kWarmup,          // streaming setpoints antes de pedir el cambio de modo
    kWaitRcOffboard,  // esperando que el piloto ponga el switch del RC en OFFBOARD
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
  void vehicleLocalPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void evOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void vehicleCommandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
  bool evReady() const;
  bool evOdometryFresh() const;
  static std::string navStateToString(uint8_t nav_state);
  static std::string commandResultToString(uint8_t result);

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr ev_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  px4_msgs::msg::VehicleLocalPosition::SharedPtr last_local_position_;

  State state_{State::kWaitEvReady};
  uint64_t cycle_count_{0};
  uint8_t last_nav_state_{255};
  uint8_t last_nav_state_user_intention_{255};
  uint8_t last_arming_state_{0};
  bool last_failsafe_{false};
  rclcpp::Time last_ev_odometry_stamp_{0, 0, RCL_ROS_TIME};
  bool ev_odometry_seen_{false};
  bool offboard_confirmed_{false};
  bool arm_confirmed_{false};
  bool disarm_confirmed_{false};
  bool arm_wait_started_{false};
  uint64_t arm_wait_start_cycle_{0};
  bool external_disarm_detected_{false};

  const uint64_t active_cycles_;  // hold_seconds * kLoopRateHz -- ver constructor

  static constexpr float kLoopRateHz = 10.0f;
  static constexpr uint64_t kEvReadyTimeoutCycles = 100;      // 10 s
  static constexpr uint64_t kWarmupCycles = 50;                // 5 s
  static constexpr uint64_t kRcOffboardIntentTimeoutCycles = 100; // 10 s
  static constexpr uint64_t kArmDelayCycles = 30;              // 3 s tras confirmar OFFBOARD
  static constexpr uint64_t kOffboardConfirmTimeoutCycles = 100; // 10 s
  static constexpr uint64_t kArmConfirmTimeoutCycles = 50;     // 5 s
  static constexpr uint64_t kDisarmConfirmTimeoutCycles = 50;  // 5 s

  // La 1.14.3 NO publica estimator_status_flags (su lista de topics se compila
  // en el firmware: 14 topics, frente a los 28 de main), asi que cs_ev_pos /
  // cs_ev_yaw nunca llegaban y el nodo se quedaba esperando para siempre --
  // y peor, en kActive habria pedido DESARME nada mas armar. Se sustituyen por
  // lo que si publica vehicle_local_position mas la frescura de la propia
  // odometria EV. Ver docs/2026-09-16_test_lidar2d_ev_offboard.md seccion 5.
  static constexpr float kMaxEphM = 1.0f;
  static constexpr double kEvOdometryMaxAgeS = 0.2;

  static constexpr uint16_t PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6;
};

#endif  // PX4_DRONE__EV_OFFBOARD_HANDSHAKE_HPP_
