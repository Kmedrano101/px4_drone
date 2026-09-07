#include "px4_drone/takeoff_position_hold_base.hpp"

// Variante interior: el flujo optico y el lidar 1D estan cableados
// directamente al FC (no a la RPi), asi que PX4/EKF2 ya se encarga de
// fusionarlos: este nodo no lee ningun sensor por su cuenta, solo verifica
// que la fusion resultante (vehicle_local_position) sea valida antes de
// despegar. El resto de la maquina de estados (OFFBOARD, ARM, despegue,
// hold, aterrizaje) es exactamente la misma que la variante exterior; ver
// TakeoffPositionHoldBase.
class TakeoffPositionHoldIndoor : public TakeoffPositionHoldBase
{
public:
  TakeoffPositionHoldIndoor()
  : TakeoffPositionHoldBase("takeoff_position_hold_indoor", /*default_height=*/1.0f, /*default_hold=*/8.0f)
  {
  }

protected:
  bool positionSourceReady() const override
  {
    const auto * lp = localPosition();
    return lp != nullptr && lp->xy_valid && lp->z_valid && lp->v_xy_valid && lp->dist_bottom_valid;
  }

  std::string positionSourceName() const override
  {
    return "flujo optico + lidar 1D (vehicle_local_position: xy/z/v_xy/dist_bottom validos)";
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakeoffPositionHoldIndoor>();
  const bool ok = node->okToRun();
  if (ok) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
