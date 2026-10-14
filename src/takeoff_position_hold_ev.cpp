// Nodo EV con control por defecto (setpoints de posicion: la cascada P/PID
// de PX4). La clase vive en la cabecera porque takeoff_hold_land_mpc hereda
// de ella para reutilizar fuente de posicion, salud del SLAM y parada por
// obstaculo, cambiando solo como se genera el setpoint.
#include "px4_drone/takeoff_position_hold_ev.hpp"

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
