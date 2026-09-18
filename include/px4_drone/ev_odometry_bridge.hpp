#ifndef PX4_DRONE__EV_ODOMETRY_BRIDGE_HPP_
#define PX4_DRONE__EV_ODOMETRY_BRIDGE_HPP_

#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

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
//
// Incertidumbre (2026-09-18): la tf se republica a publish_rate_hz porque el
// EKF2 deja de fusionar el EV si entre dos muestras pasan mas de 200 ms
// (EV_MAX_INTERVAL, ev_control.cpp:56), pero el SLAM solo actualiza la pose
// ~1-2 veces por segundo. Antes cada muestra iba con varianza NaN (el EKF usa
// EKF2_EVP_NOISE = 0.1 m), asi que una pose de hace 0.8 s pesaba como una
// medida nueva, repetida 16 veces. Ahora la varianza es la que da el SLAM en
// /pose mas lo que el dron puede haberse movido desde el barrido que la
// produjo: cov + (antiguedad * ev_max_speed_m_s)^2. Una pose recien calculada
// pesa; sus repeticiones cada vez menos, y entre poses manda el flujo optico.
// Cuando el SLAM pierde el tracking su covarianza sube sola (x50 en el vuelo
// del 2026-09-17) y el EKF deja de creerle. Ver el reporte del log 158.
class EvOdometryBridge : public rclcpp::Node
{
public:
  EvOdometryBridge();

private:
  void onTimer();
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void estimatorStatusFlagsCallback(
    const px4_msgs::msg::EstimatorStatusFlags::SharedPtr msg);

  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_pub_;
  rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr
    estimator_status_flags_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  const std::string map_frame_;
  const std::string base_frame_;
  const float pose_jump_threshold_m_;
  const float max_speed_m_s_;       // cota de lo que el dron se mueve mientras la pose envejece
  const float max_yaw_rate_rad_s_;

  // Ultima /pose del SLAM: varianzas en ENU (map) e instante del barrido.
  bool have_pose_{false};
  double pose_var_xx_enu_{0.0};
  double pose_var_yy_enu_{0.0};
  double pose_var_yaw_{0.0};
  rclcpp::Time pose_stamp_{0, 0, RCL_ROS_TIME};

  bool tf_warning_logged_{false};
  std::optional<px4_drone::frames::Vec3> last_position_ned_;
  uint8_t reset_counter_{0};

  bool last_cs_ev_pos_{false};
  bool last_cs_ev_yaw_{false};
  bool last_cs_ev_hgt_{false};
  bool last_cs_ev_vel_{false};
};

#endif  // PX4_DRONE__EV_ODOMETRY_BRIDGE_HPP_
