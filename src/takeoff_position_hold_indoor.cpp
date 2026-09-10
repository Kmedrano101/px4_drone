#include "px4_drone/takeoff_position_hold_base.hpp"

// Variante interior: el flujo optico y el lidar 1D estan cableados
// directamente al FC (no a la RPi), asi que PX4/EKF2 ya se encarga de
// fusionarlos: este nodo no lee ningun sensor por su cuenta, solo verifica
// que la fusion resultante (vehicle_local_position) sea valida antes de
// despegar. El resto de la maquina de estados (OFFBOARD, ARM, despegue,
// hold, aterrizaje) es exactamente la misma que la variante exterior; ver
// TakeoffPositionHoldBase.
//
// NOTA (2026-09-10): ya NO se exige dist_bottom_valid. Con las patas
// actuales el lidar 1D queda a veces por debajo de su rango minimo
// confiable en tierra (dist_bottom ~0.10 m), asi que dist_bottom_valid
// nunca llegaba a true y el despegue abortaba siempre por timeout (visto
// en 3 intentos de campo). z_valid ya certifica que la altura (fusion
// baro+IMU+lo que este disponible) es medible; dist_bottom solo aporta la
// referencia de distancia al suelo, que en este dron es opcional, no una
// condicion para poder volar. Si se vuelve a cambiar el tren de aterrizaje
// y el lidar queda otra vez dentro de rango, este requisito se puede
// endurecer de nuevo si hace falta.
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
    return lp != nullptr && lp->xy_valid && lp->z_valid && lp->v_xy_valid;
  }

  std::string positionSourceName() const override
  {
    return "flujo optico + lidar 1D (vehicle_local_position: xy/z/v_xy validos; "
           "dist_bottom no requerido, ver nota en el codigo)";
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
