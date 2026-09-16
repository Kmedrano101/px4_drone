#ifndef PX4_DRONE__EV_ODOMETRY_BRIDGE_HPP_
#define PX4_DRONE__EV_ODOMETRY_BRIDGE_HPP_

#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>

#include "px4_drone/frame_transforms.hpp"

// Puente pasivo SLAM 2D (slam_toolbox) -> EKF2 como "external vision".
//
// Lee la tf map->base_link que publica slam_toolbox (ver px4_drone_slam) y
// la traduce a px4_msgs::msg::VehicleOdometry en /fmu/in/vehicle_visual_odometry.
// No arma el dron ni manda ningun VehicleCommand: es solo instrumentacion,
// pensado para verificar que el EKF2 acepta la fusion (cs_ev_pos/cs_ev_yaw en
// estimator_status_flags) con el dron desarmado y movido a mano.
//
// slam_toolbox corrige la pose con loop closure (do_loop_closing: true en
// px4_drone_slam/config/slam_toolbox_params.yaml), asi que map->base_link
// puede saltar. Este nodo no lo evita: detecta el salto (umbral
// pose_jump_threshold_m) e incrementa reset_counter, que es la forma en que
// VehicleOdometry le avisa al EKF2 que la muestra actual no es continua con
// la anterior.
class EvOdometryBridge : public rclcpp::Node
{
public:
  EvOdometryBridge();

private:
  void onTimer();
  void estimatorStatusFlagsCallback(
    const px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg);

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_pub_;
  rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr
    estimator_status_flags_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  const std::string map_frame_;
  const std::string base_frame_;
  const float pose_jump_threshold_m_;

  bool tf_warning_logged_{false};
  std::optional<px4_drone::frames::Vec3> last_position_ned_;
  uint8_t reset_counter_{0};

  bool last_cs_ev_pos_{false};
  bool last_cs_ev_yaw_{false};
  bool last_cs_ev_hgt_{false};
  bool last_cs_ev_vel_{false};
};

#endif  // PX4_DRONE__EV_ODOMETRY_BRIDGE_HPP_
