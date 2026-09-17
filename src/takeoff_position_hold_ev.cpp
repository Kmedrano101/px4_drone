#include "px4_drone/takeoff_position_hold_base.hpp"

#include <cmath>

// Variante LiDAR 2D (SLAM -> external vision): exige xy/z/heading_good_for_control
// (vehicle_local_position) sanos y sin dead_reckoning antes de despegar.
// NOTA (2026-09-17): PX4 1.14.3 vendor NO publica estimator_status_flags
// (su lista de topics DDS se compila en el firmware y no lo incluye). Por eso
// se usa vehicle_local_position: xy_valid, z_valid, heading_good_for_control,
// !dead_reckoning y eph < 1.0 m.
// La altura esta limitada a 1.2 m (techo fiable del LiDAR 1D, EKF2_RNG_A_HMAX),
// rechazada en el constructor si se pide mas.
class TakeoffPositionHoldEv : public TakeoffPositionHoldBase
{
public:
  TakeoffPositionHoldEv()
  : TakeoffPositionHoldBase("takeoff_position_hold_ev", /*default_height=*/1.0f, /*default_hold=*/5.0f)
  {
    if (takeoffHeightM() > kMaxHeightM) {
      refuseToRun(
        "takeoff_height_m supera el techo fiable del lidar 1D (" +
        std::to_string(kMaxHeightM) + " m).");
      return;
    }
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

private:
  static constexpr float kMaxHeightM = 1.2f;
  static constexpr float kMaxEphM = 1.0f;
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
