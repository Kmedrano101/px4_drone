#include "px4_drone/takeoff_position_hold_base.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "px4_drone/obstacle_guard.hpp"

// Variante LiDAR 2D (SLAM -> external vision): exige xy/z/heading_good_for_control
// (vehicle_local_position) sanos y sin dead_reckoning antes de despegar.
// NOTA (2026-09-17): PX4 1.14.3 vendor NO publica estimator_status_flags
// (su lista de topics DDS se compila en el firmware y no lo incluye). Por eso
// se usa vehicle_local_position: xy_valid, z_valid, heading_good_for_control,
// !dead_reckoning y eph < 1.0 m.
// La altura esta limitada a 1.2 m (techo fiable del LiDAR 1D, EKF2_RNG_A_HMAX),
// rechazada en el constructor si se pide mas.
//
// Parada por obstaculo (2026-09-18): con obstacle_stop_distance_m > 0 el nodo
// vigila /scan en los 360 grados. No arma si hay algo mas cerca que esa
// distancia, y en vuelo frena y aterriza si un obstaculo se acerca tanto que
// hay que empezar a frenar para quedar parado a esa distancia. Usa los
// barridos crudos, no el SLAM ni la posicion del EKF: ver obstacle_guard.hpp.
class TakeoffPositionHoldEv : public TakeoffPositionHoldBase
{
public:
  TakeoffPositionHoldEv()
  : TakeoffPositionHoldBase("takeoff_position_hold_ev", /*default_height=*/1.0f, /*default_hold=*/5.0f)
  {
    if (!okToRun()) {
      return;  // la base ya se nego (falta confirm_takeoff, parametros invalidos)
    }
    if (takeoffHeightM() > kMaxHeightM) {
      refuseToRun(
        "takeoff_height_m supera el techo fiable del lidar 1D (" +
        std::to_string(kMaxHeightM) + " m).");
      return;
    }

    px4_drone::ObstacleGuardConfig cfg;
    cfg.stop_distance_m = static_cast<float>(
      declare_parameter<double>("obstacle_stop_distance_m", 1.0));
    cfg.min_valid_range_m = static_cast<float>(
      declare_parameter<double>("obstacle_min_valid_range_m", cfg.min_valid_range_m));
    cfg.brake_decel_m_s2 = static_cast<float>(
      declare_parameter<double>("obstacle_brake_decel_m_s2", cfg.brake_decel_m_s2));
    laser_yaw_rad_ = static_cast<float>(
      declare_parameter<double>("laser_yaw_offset_deg", 0.0) * M_PI / 180.0);
    laser_above_range_m_ = static_cast<float>(
      declare_parameter<double>("laser_height_above_range_m", 0.134));
    scan_timeout_s_ = declare_parameter<double>("obstacle_scan_timeout_s", 0.5);
    const std::string scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    const std::string v = get_parameter("topic_version_suffix").as_string();

    if (cfg.stop_distance_m < 0.0f || (cfg.stop_distance_m > 0.0f && cfg.stop_distance_m < 0.3f)) {
      refuseToRun(
        "obstacle_stop_distance_m debe ser 0 (desactivada) o >= 0.3 m (recibido " +
        std::to_string(cfg.stop_distance_m) + ").");
      return;
    }
    stop_distance_m_ = cfg.stop_distance_m;
    if (stop_distance_m_ == 0.0f) {
      RCLCPP_WARN(get_logger(), "PARADA POR OBSTACULO DESACTIVADA (obstacle_stop_distance_m = 0).");
      return;
    }

    guard_ = std::make_unique<px4_drone::ObstacleGuard>(cfg);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&TakeoffPositionHoldEv::scanCallback, this, std::placeholders::_1));

    rclcpp::QoS status_qos(5);
    status_qos.best_effort();
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude" + v, status_qos,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {last_attitude_ = msg;});

    RCLCPP_WARN(
      get_logger(),
      "Parada por obstaculo ACTIVA: no arma con obstaculos a menos de %.2f m, y en vuelo frena y "
      "aterriza para quedar parado a %.2f m de cualquier obstaculo (360 grados, %s).",
      stop_distance_m_, stop_distance_m_, scan_topic.c_str());
  }

protected:
  bool positionSourceReady() const override
  {
    const auto * lp = localPosition();
    return lp != nullptr && lp->xy_valid && lp->z_valid && lp->heading_good_for_control &&
           !lp->dead_reckoning && std::isfinite(lp->eph) && lp->eph < kMaxEphM;
  }

  std::string positionSourceName() const override
  {
    return "LiDAR 2D SLAM/EV + LiDAR 1D (vehicle_local_position: xy/z/heading_good_for_control validos, "
           "!dead_reckoning, eph < 1.0 m)";
  }

  bool positionSourceHealthyDuringFlight() const override
  {
    const auto * lp = localPosition();
    return lp != nullptr && lp->xy_valid && !lp->dead_reckoning;
  }

  bool obstacleClearForTakeoff(std::string * reason) const override
  {
    if (!guard_) {
      return true;  // desactivada
    }
    if (scanUnusable(reason)) {
      return false;
    }
    const auto & st = guard_->last();
    if (std::isfinite(st.nearest_m) && st.nearest_m < stop_distance_m_) {
      *reason = fmt(
        "obstaculo a %.2f m (rumbo %+.0f grados, 0 = morro, + = izquierda); hacen falta %.2f m "
        "libres alrededor.", st.nearest_m, st.nearest_bearing_deg, stop_distance_m_);
      return false;
    }
    return true;
  }

  bool obstacleTooClose(std::string * reason) const override
  {
    if (!guard_) {
      return false;  // desactivada
    }
    if (scanUnusable(reason)) {
      return true;  // sin LiDAR no se puede garantizar la distancia: se para
    }
    const auto & st = guard_->last();
    if (st.triggered) {
      *reason = fmt(
        "obstaculo a %.2f m (rumbo %+.0f grados), acercandose a %.2f m/s: umbral de frenada "
        "%.2f m para quedar a %.2f m.", st.trigger_range_m, st.trigger_bearing_deg,
        st.closing_speed_m_s, st.trigger_distance_m, stop_distance_m_);
      return true;
    }
    return false;
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const rclcpp::Time now = this->now();
    px4_drone::ScanInput in;
    in.stamp_s = now.seconds();
    in.angle_min = msg->angle_min;
    in.angle_increment = msg->angle_increment;
    in.range_min = msg->range_min;
    in.range_max = msg->range_max;
    in.ranges = msg->ranges.data();
    in.count = msg->ranges.size();
    in.laser_yaw_rad = laser_yaw_rad_;

    // Sin actitud o sin altura valida (en tierra dist_bottom_valid es false)
    // no se filtra el suelo: con el dron nivelado no hace falta, y si hiciera
    // falta el error es parar de mas, no de menos.
    in.laser_height_m = std::numeric_limits<float>::quiet_NaN();
    if (last_attitude_) {
      const auto & q = last_attitude_->q;  // w, x, y, z; FRD -> NED
      in.roll_rad = std::atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
      in.pitch_rad = std::asin(std::clamp(2.0f * (q[0] * q[2] - q[3] * q[1]), -1.0f, 1.0f));
      const auto * lp = localPosition();
      if (lp != nullptr && lp->dist_bottom_valid && std::isfinite(lp->dist_bottom)) {
        in.laser_height_m = lp->dist_bottom + laser_above_range_m_;
      }
    }

    const auto st = guard_->update(in);
    last_scan_time_ = now;
    have_scan_ = true;

    if (std::isfinite(st.nearest_m) && st.nearest_m < 2.0f * stop_distance_m_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Obstaculo mas cercano %.2f m (rumbo %+.0f), acercandose %.2f m/s, umbral %.2f m",
        st.nearest_m, st.nearest_bearing_deg, st.closing_speed_m_s, st.trigger_distance_m);
    }
  }

  // true (y reason relleno) si /scan no sirve para garantizar la distancia.
  bool scanUnusable(std::string * reason) const
  {
    if (!have_scan_) {
      *reason = "no ha llegado ningun /scan del LiDAR 2D.";
      return true;
    }
    const double age = (this->now() - last_scan_time_).seconds();
    if (age > scan_timeout_s_) {
      *reason = fmt("el LiDAR 2D no publica /scan desde hace %.2f s.", age);
      return true;
    }
    const auto & st = guard_->last();
    if (st.valid_fraction < guard_->config().min_valid_fraction) {
      *reason = fmt(
        "solo el %.0f%% de los rayos del LiDAR 2D es valido (sensor tapado o fallando).",
        100.0f * st.valid_fraction);
      return true;
    }
    return false;
  }

  template<typename ... Args>
  static std::string fmt(const char * f, Args... args)
  {
    char buf[320];
    std::snprintf(buf, sizeof(buf), f, args ...);
    return buf;
  }

  static constexpr float kMaxHeightM = 1.2f;
  static constexpr float kMaxEphM = 1.0f;

  std::unique_ptr<px4_drone::ObstacleGuard> guard_;  // nullptr = parada desactivada
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  px4_msgs::msg::VehicleAttitude::SharedPtr last_attitude_;
  rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
  bool have_scan_{false};
  float stop_distance_m_{0.0f};
  float laser_yaw_rad_{0.0f};
  float laser_above_range_m_{0.134f};
  double scan_timeout_s_{0.5};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakeoffPositionHoldEv>();
  const bool ok = node->okToRun();
  if (ok) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
