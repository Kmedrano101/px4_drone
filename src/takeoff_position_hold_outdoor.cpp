#include "px4_drone/takeoff_position_hold_base.hpp"

#include <px4_msgs/msg/sensor_gps.hpp>

// Variante exterior: exige un fix GPS 3D (o mejor) antes de intentar
// despegar. El resto de la maquina de estados (OFFBOARD, ARM, despegue,
// hold, aterrizaje) es exactamente la misma que la variante interior; ver
// TakeoffPositionHoldBase.
class TakeoffPositionHoldOutdoor : public TakeoffPositionHoldBase
{
public:
  TakeoffPositionHoldOutdoor()
  : TakeoffPositionHoldBase("takeoff_position_hold_outdoor", /*default_height=*/2.0f, /*default_hold=*/10.0f)
  {
    rclcpp::QoS qos(5);
    qos.best_effort();
    gps_sub_ = create_subscription<px4_msgs::msg::SensorGps>(
      "/fmu/out/vehicle_gps_position", qos,
      [this](const px4_msgs::msg::SensorGps::SharedPtr msg) {
        gps_ready_ = msg->fix_type >= kFixType3D;
      });
  }

protected:
  bool positionSourceReady() const override {return gps_ready_;}
  std::string positionSourceName() const override {return "GPS (fix 3D o mejor)";}

private:
  // FIX_TYPE_3D es una constante nombrada solo en px4_msgs a partir del
  // versionado de mensajes (firmware v1.17+); en versiones viejas (v1.14,
  // branch fc-v14-ffb6e80) fix_type es un uint8 sin constantes, documentado
  // en el .msg como "3: 3D fix". Mismo valor numerico en ambas, asi que se
  // fija aca para no depender de que px4_msgs la provea.
  static constexpr uint8_t kFixType3D = 3;

  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_sub_;
  bool gps_ready_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakeoffPositionHoldOutdoor>();
  const bool ok = node->okToRun();
  if (ok) {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
