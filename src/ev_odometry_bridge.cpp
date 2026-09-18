#include "px4_drone/ev_odometry_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/exceptions.h>

using px4_msgs::msg::EstimatorStatusFlags;
using px4_msgs::msg::VehicleOdometry;

namespace
{
// Varianza para lo que el EV no mide (z, roll, pitch): enorme pero finita.
constexpr float kUnmeasuredVariance = 1.0e4f;
}  // namespace

EvOdometryBridge::EvOdometryBridge()
: Node("ev_odometry_bridge"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  map_frame_(declare_parameter<std::string>("map_frame", "map")),
  base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
  pose_jump_threshold_m_(
    static_cast<float>(declare_parameter<double>("pose_jump_threshold_m", 0.3))),
  max_speed_m_s_(static_cast<float>(declare_parameter<double>("ev_max_speed_m_s", 1.5))),
  max_yaw_rate_rad_s_(static_cast<float>(declare_parameter<double>("ev_max_yaw_rate_rad_s", 0.8)))
{
  const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);
  // Ver comentario equivalente en offboard_control.cpp: "" para firmware
  // v1.14 (default, FC actual), "_v1" para v1.17 (HKUST_NXT_DUAL).
  const std::string v = declare_parameter<std::string>("topic_version_suffix", "");

  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.transient_local();
  qos.keep_last(1);

  rclcpp::QoS status_qos(5);
  status_qos.best_effort();

  vehicle_odometry_pub_ = create_publisher<VehicleOdometry>(
    "/fmu/in/vehicle_visual_odometry" + v, qos);

  estimator_status_flags_sub_ = create_subscription<EstimatorStatusFlags>(
    "/fmu/out/estimator_status_flags" + v, status_qos,
    std::bind(&EvOdometryBridge::estimatorStatusFlagsCallback, this, std::placeholders::_1));

  // /pose de slam_toolbox: solo llega cuando procesa un barrido, con el
  // instante de ese barrido y su covarianza.
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    declare_parameter<std::string>("pose_topic", "/pose"), rclcpp::QoS(10),
    std::bind(&EvOdometryBridge::poseCallback, this, std::placeholders::_1));

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&EvOdometryBridge::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "ev_odometry_bridge iniciado: %s -> %s a %.0f Hz, publicando en vehicle_visual_odometry. "
    "No arma ni comanda el dron, solo puentea la pose del SLAM.",
    map_frame_.c_str(), base_frame_.c_str(), publish_rate_hz);
}

void EvOdometryBridge::poseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const auto & c = msg->pose.covariance;  // 6x6 por filas: x, y, z, roll, pitch, yaw
  if (!std::isfinite(c[0]) || !std::isfinite(c[7]) || !std::isfinite(c[35])) {
    return;
  }
  const rclcpp::Time now = get_clock()->now();
  rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
  // El sello es el del barrido (40-75 ms antes de llegar, medido en el bag del
  // 2026-09-17). Si viene del futuro o absurdamente viejo, se usa la llegada.
  const double lag = (now - stamp).seconds();
  if (lag < -0.05 || lag > 2.0) {
    stamp = now;
  }
  if (!have_pose_) {
    RCLCPP_INFO(
      get_logger(), "Primera /pose del SLAM: sigma xy %.3f m. Desde ahora la varianza del EV es "
      "la del SLAM + (antiguedad x %.1f m/s)^2.", std::sqrt(std::max(c[0], c[7])),
      static_cast<double>(max_speed_m_s_));
  }
  pose_var_xx_enu_ = c[0];
  pose_var_yy_enu_ = c[7];
  pose_var_yaw_ = c[35];
  pose_stamp_ = stamp;
  have_pose_ = true;
}

void EvOdometryBridge::onTimer()
{
  geometry_msgs::msg::TransformStamped t;
  try {
    t = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    if (!tf_warning_logged_) {
      RCLCPP_WARN(
        get_logger(), "Esperando tf %s -> %s (%s). No se publica hasta que este disponible.",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      tf_warning_logged_ = true;
    }
    return;
  }
  tf_warning_logged_ = false;

  // No usar t.header.stamp como "instante de la medida": slam_toolbox lo
  // estampa con margen hacia adelante (transform_publish_period/
  // transform_timeout, ver px4_drone_slam/config/slam_toolbox_params.yaml)
  // para tolerancia de TF, no para reflejar cuando se tomo el scan -- se
  // vio en pruebas que queda ~200 ms por delante del reloj real. Se usa el
  // momento de lectura como mejor estimacion disponible del sample.
  const uint64_t now_us = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);

  const auto & p = t.transform.translation;
  const px4_drone::frames::Vec3 position_ned =
    px4_drone::frames::enuNedSwap({static_cast<float>(p.x), static_cast<float>(p.y),
      static_cast<float>(p.z)});

  const auto & q_ros = t.transform.rotation;  // orden ROS: x, y, z, w
  const float yaw_enu = std::atan2(
    2.0f * (static_cast<float>(q_ros.w) * static_cast<float>(q_ros.z) +
    static_cast<float>(q_ros.x) * static_cast<float>(q_ros.y)),
    1.0f - 2.0f * (static_cast<float>(q_ros.y) * static_cast<float>(q_ros.y) +
    static_cast<float>(q_ros.z) * static_cast<float>(q_ros.z)));
  const float yaw_ned = px4_drone::frames::yawEnuNedSwap(yaw_enu);

  // Solo yaw: con un LiDAR 2D no hay roll/pitch que medir, se ponen a cero a
  // proposito en vez de arrastrar lo que traiga la tf (que en un SLAM 2D
  // deberia ser ~0 de todos modos, pero no hay que confiar en eso).
  const px4_drone::frames::Quat q_ned{
    std::cos(yaw_ned / 2.0f), 0.0f, 0.0f, std::sin(yaw_ned / 2.0f)};

  if (last_position_ned_.has_value()) {
    const float dx = position_ned[0] - (*last_position_ned_)[0];
    const float dy = position_ned[1] - (*last_position_ned_)[1];
    const float dz = position_ned[2] - (*last_position_ned_)[2];
    const float jump_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (jump_m > pose_jump_threshold_m_) {
      ++reset_counter_;
      RCLCPP_WARN(
        get_logger(),
        "Salto de pose detectado (%.2f m, posible loop closure de slam_toolbox). "
        "reset_counter -> %u",
        jump_m, reset_counter_);
    }
  }
  last_position_ned_ = position_ned;

  const float nan = std::numeric_limits<float>::quiet_NaN();

  VehicleOdometry msg{};
  msg.timestamp = now_us;
  msg.timestamp_sample = now_us;
  // NED, no FRD: la pose ya viene convertida a NED arriba (enuNedSwap /
  // yawEnuNedSwap), asi que FRD era una etiqueta falsa. Ademas EKF2 1.14.3
  // trata las dos ramas distinto a proposito (ev_yaw_control.cpp:139-170):
  // con NED hace resetQuatStateYaw y pone yaw_align = true; con FRD lo deja
  // en false, y entonces heading_good_for_control no llega a ser true nunca.
  msg.pose_frame = VehicleOdometry::POSE_FRAME_NED;
  msg.position = {position_ned[0], position_ned[1], position_ned[2]};
  msg.q = {q_ned[0], q_ned[1], q_ned[2], q_ned[3]};
  msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
  msg.velocity = {nan, nan, nan};
  msg.angular_velocity = {nan, nan, nan};
  if (have_pose_) {
    // EKF2.cpp:2106 y :2141 solo usan las varianzas si las TRES son finitas;
    // si no, las tres pasan a EKF2_EVP_NOISE / EKF2_EVA_NOISE. Por eso z,
    // roll y pitch (que el EV de un LiDAR 2D no mide y EKF2_EV_CTRL=9 no
    // fusiona) van con una varianza enorme pero finita, no NaN.
    const double age = std::max(0.0, (get_clock()->now() - pose_stamp_).seconds());
    const double drift = age * static_cast<double>(max_speed_m_s_);
    const double yaw_drift = age * static_cast<double>(max_yaw_rate_rad_s_);
    // map es ENU y el mensaje NED: x_ned = y_enu, y_ned = x_enu.
    msg.position_variance = {
      static_cast<float>(pose_var_yy_enu_ + drift * drift),
      static_cast<float>(pose_var_xx_enu_ + drift * drift),
      kUnmeasuredVariance};
    msg.orientation_variance = {
      kUnmeasuredVariance, kUnmeasuredVariance,
      static_cast<float>(pose_var_yaw_ + yaw_drift * yaw_drift)};
  } else {
    msg.position_variance = {nan, nan, nan};
    msg.orientation_variance = {nan, nan, nan};
  }
  msg.velocity_variance = {nan, nan, nan};
  msg.reset_counter = reset_counter_;
  msg.quality = -1;
  vehicle_odometry_pub_->publish(msg);
}

void EvOdometryBridge::estimatorStatusFlagsCallback(
  const EstimatorStatusFlags::SharedPtr msg)
{
  if (msg->cs_ev_pos != last_cs_ev_pos_) {
    RCLCPP_INFO(get_logger(), "cs_ev_pos: %s", msg->cs_ev_pos ? "true" : "false");
    last_cs_ev_pos_ = msg->cs_ev_pos;
  }
  if (msg->cs_ev_yaw != last_cs_ev_yaw_) {
    RCLCPP_INFO(get_logger(), "cs_ev_yaw: %s", msg->cs_ev_yaw ? "true" : "false");
    last_cs_ev_yaw_ = msg->cs_ev_yaw;
  }
  if (msg->cs_ev_hgt != last_cs_ev_hgt_) {
    RCLCPP_INFO(get_logger(), "cs_ev_hgt: %s", msg->cs_ev_hgt ? "true" : "false");
    last_cs_ev_hgt_ = msg->cs_ev_hgt;
  }
  if (msg->cs_ev_vel != last_cs_ev_vel_) {
    RCLCPP_INFO(get_logger(), "cs_ev_vel: %s", msg->cs_ev_vel ? "true" : "false");
    last_cs_ev_vel_ = msg->cs_ev_vel;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EvOdometryBridge>());
  rclcpp::shutdown();
  return 0;
}
