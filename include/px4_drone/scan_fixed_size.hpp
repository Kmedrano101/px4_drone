#ifndef PX4_DRONE__SCAN_FIXED_SIZE_HPP_
#define PX4_DRONE__SCAN_FIXED_SIZE_HPP_

#include <cmath>
#include <cstddef>
#include <limits>

#include <sensor_msgs/msg/laser_scan.hpp>

// Remuestrea un LaserScan a una rejilla angular FIJA de n rayos.
//
// Por que existe: el LD19 entrega un numero variable de puntos por vuelta
// (498-506) y su driver publica cada barrido con su propio tamano y su propio
// angle_increment. slam_toolbox fija el numero de rayos del sensor con el
// PRIMER barrido y karto (LaserRangeFinder::Validate) rechaza con `return
// false` cualquier barrido cuyo numero no coincida: solo pasaba el 20-30 % y la
// pose del SLAM llegaba al EKF a ~1 Hz (banco del 2026-10-21; es el 1.2 Hz del
// accidente del 2026-09-18). Con tamano fijo pasan todos.
//
// Mismo criterio que ya usa el driver para rellenar su propio barrido: binning
// por angulo, el MENOR rango por celda (lo mas cercano manda, que es lo que
// importa para esquivar), NaN en las celdas sin retorno. C++ puro, sin ROS en
// la logica, para poder probarlo con barridos sinteticos y con los bags.
namespace px4_drone
{

inline sensor_msgs::msg::LaserScan resampleScan(
  const sensor_msgs::msg::LaserScan & in, std::size_t n)
{
  constexpr float kTwoPi = 2.0f * 3.14159265358979f;
  const float nan = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::msg::LaserScan out;
  out.header = in.header;
  out.range_min = in.range_min;
  out.range_max = in.range_max;
  out.scan_time = in.scan_time;
  out.angle_min = 0.0f;
  out.angle_increment = kTwoPi / static_cast<float>(n);
  // angle_max = incremento * (n-1): asi (max-min)/inc + 1 == n exactamente,
  // que es lo que karto calcula para saber cuantos rayos espera.
  out.angle_max = out.angle_increment * static_cast<float>(n - 1);
  out.time_increment = n > 1 ? in.scan_time / static_cast<float>(n - 1) : 0.0f;
  out.ranges.assign(n, nan);
  const bool has_intensity = in.intensities.size() == in.ranges.size();
  if (has_intensity) {
    out.intensities.assign(n, nan);
  }

  for (std::size_t i = 0; i < in.ranges.size(); ++i) {
    const float r = in.ranges[i];
    if (!std::isfinite(r)) {
      continue;
    }
    float a = in.angle_min + static_cast<float>(i) * in.angle_increment;
    a = std::fmod(a, kTwoPi);
    if (a < 0.0f) {
      a += kTwoPi;
    }
    // Celda mas cercana en angulo; el ultimo rayo del barrido de entrada puede
    // caer en 2pi, que es la celda 0.
    std::size_t k = static_cast<std::size_t>(std::lround(a / out.angle_increment));
    if (k >= n) {
      k -= n;
    }
    if (!std::isfinite(out.ranges[k]) || r < out.ranges[k]) {
      out.ranges[k] = r;
      if (has_intensity) {
        out.intensities[k] = in.intensities[i];
      }
    }
  }
  return out;
}

}  // namespace px4_drone

#endif  // PX4_DRONE__SCAN_FIXED_SIZE_HPP_
