// Relay: /scan_raw (tamano variable, del driver del LD19) -> /scan (n rayos fijos).
// La logica esta en include/px4_drone/scan_fixed_size.hpp; esto solo es la
// tuberia ROS. Ver ahi por que hace falta.
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "px4_drone/scan_fixed_size.hpp"

class ScanFixedSize : public rclcpp::Node
{
public:
  ScanFixedSize()
  : Node("scan_fixed_size")
  {
    const std::string in = declare_parameter<std::string>("input_topic", "/scan_raw");
    const std::string out = declare_parameter<std::string>("output_topic", "/scan");
    const int n = static_cast<int>(declare_parameter<int>("n_rays", 500));
    if (n < 36 || n > 4096) {
      RCLCPP_FATAL(get_logger(), "n_rays debe estar entre 36 y 4096 (recibido %d).", n);
      rclcpp::shutdown();
      return;
    }
    n_ = static_cast<std::size_t>(n);

    // El driver publica RELIABLE; un suscriptor best-effort le vale igual y no
    // retiene al driver si este nodo se retrasa. La salida se publica con la
    // QoS de sensor, que es lo que esperan slam_toolbox y la guarda.
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(out, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      in, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr m) {
        pub_->publish(px4_drone::resampleScan(*m, n_));
        if (++count_ == 1 || count_ % 600 == 0) {   // al arrancar y cada minuto a 10 Hz
          RCLCPP_INFO(
            get_logger(), "barrido %lu: %zu rayos de entrada -> %zu fijos",
            static_cast<unsigned long>(count_), m->ranges.size(), n_);
        }
      });
    RCLCPP_INFO(
      get_logger(), "Remuestreando %s (tamano variable) -> %s a %zu rayos fijos (%.3f grados).",
      in.c_str(), out.c_str(), n_, 360.0 / static_cast<double>(n_));
  }

private:
  std::size_t n_{500};
  unsigned long count_{0};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanFixedSize>());
  rclcpp::shutdown();
  return 0;
}
