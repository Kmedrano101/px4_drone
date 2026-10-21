// Pruebas de resampleScan: sinteticas y, si se le da un CSV de bag_to_scans,
// sobre barridos reales del LD19.
//   g++ -O2 -I include $(ros2 pkg prefix sensor_msgs)/include/... (ver CMake)
//   test_scan_fixed_size [scans.csv]
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "px4_drone/scan_fixed_size.hpp"

namespace
{
int failures = 0;
void check(bool ok, const char * what)
{
  std::printf("%s  %s\n", ok ? "ok  " : "FALLA", what);
  if (!ok) {++failures;}
}
constexpr float kPi = 3.14159265358979f;

// Barrido como lo publica el driver del LD19: n rayos sobre [0, 2pi], inc = 2pi/(n-1).
sensor_msgs::msg::LaserScan ld19(std::size_t n)
{
  sensor_msgs::msg::LaserScan s;
  s.angle_min = 0.0f;
  s.angle_max = 2.0f * kPi;
  s.angle_increment = 2.0f * kPi / static_cast<float>(n - 1);
  s.range_min = 0.02f;
  s.range_max = 12.0f;
  s.scan_time = 0.1f;
  s.ranges.assign(n, std::numeric_limits<float>::quiet_NaN());
  return s;
}
std::size_t kartoExpects(const sensor_msgs::msg::LaserScan & s)
{
  return static_cast<std::size_t>(std::lround((s.angle_max - s.angle_min) / s.angle_increment)) + 1;
}
}  // namespace

int main(int argc, char * argv[])
{
  const std::size_t N = 500;

  std::printf("== cualquier tamano de entrada sale con %zu rayos y karto espera %zu ==\n", N, N);
  for (std::size_t n : {498, 499, 500, 501, 502, 503, 504, 505, 506, 450, 600}) {
    auto in = ld19(n);
    for (std::size_t i = 0; i < n; ++i) {in.ranges[i] = 3.0f;}
    auto out = px4_drone::resampleScan(in, N);
    char msg[96];
    std::snprintf(msg, sizeof(msg), "entrada %zu -> salida %zu, karto espera %zu",
      n, out.ranges.size(), kartoExpects(out));
    check(out.ranges.size() == N && kartoExpects(out) == N, msg);
  }

  std::printf("\n== un pilar a 2 m en rumbo 90 grados sale en la celda de 90 grados ==\n");
  {
    auto in = ld19(503);
    const std::size_t i90 = static_cast<std::size_t>(std::lround((kPi / 2) / in.angle_increment));
    in.ranges[i90] = 2.0f;
    auto out = px4_drone::resampleScan(in, N);
    const std::size_t k90 = static_cast<std::size_t>(std::lround((kPi / 2) / out.angle_increment));
    check(std::fabs(out.ranges[k90] - 2.0f) < 1e-6f, "rango 2.0 en la celda de 90 grados");
    std::size_t finitos = 0;
    for (float r : out.ranges) {finitos += std::isfinite(r) ? 1 : 0;}
    check(finitos == 1, "y en ninguna otra");
  }

  std::printf("\n== dos rayos en la misma celda: manda el mas cercano ==\n");
  {
    auto in = ld19(1000);   // el doble de rayos que celdas: cada celda recibe 2
    in.ranges[100] = 1.5f;
    in.ranges[101] = 1.2f;
    auto out = px4_drone::resampleScan(in, N);
    float m = std::numeric_limits<float>::infinity();
    for (float r : out.ranges) {if (std::isfinite(r)) {m = std::min(m, r);}}
    check(std::fabs(m - 1.2f) < 1e-6f, "se conserva 1.2 (el menor) y no 1.5");
  }

  std::printf("\n== el ultimo rayo del LD19 (angulo 2pi) cae en la celda 0, no se pierde ==\n");
  {
    auto in = ld19(503);
    in.ranges[502] = 4.0f;
    auto out = px4_drone::resampleScan(in, N);
    check(std::fabs(out.ranges[0] - 4.0f) < 1e-6f, "rango 4.0 en la celda 0");
  }

  std::printf("\n== cabecera y metadatos se conservan ==\n");
  {
    auto in = ld19(501);
    in.header.frame_id = "base_laser";
    auto out = px4_drone::resampleScan(in, N);
    check(out.header.frame_id == "base_laser" && out.range_min == in.range_min &&
      out.range_max == in.range_max && std::fabs(out.scan_time - in.scan_time) < 1e-6f,
      "frame_id, range_min/max y scan_time");
  }

  if (argc > 1) {
    std::printf("\n== barridos reales de %s ==\n", argv[1]);
    std::ifstream f(argv[1]);
    std::string line;
    std::size_t total = 0, bad = 0, min_in = 1e9, max_in = 0;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') {continue;}
      std::stringstream ss(line);
      std::string tok;
      std::vector<double> head;
      for (int i = 0; i < 9 && std::getline(ss, tok, ','); ++i) {head.push_back(std::strtod(tok.c_str(), nullptr));}
      if (head.size() < 9) {continue;}
      sensor_msgs::msg::LaserScan in;
      in.angle_min = static_cast<float>(head[4]);
      in.angle_increment = static_cast<float>(head[5]);
      in.range_min = static_cast<float>(head[6]);
      in.range_max = static_cast<float>(head[7]);
      while (std::getline(ss, tok, ',')) {in.ranges.push_back(std::strtof(tok.c_str(), nullptr));}
      in.angle_max = in.angle_min + in.angle_increment * static_cast<float>(in.ranges.size() - 1);
      min_in = std::min(min_in, in.ranges.size());
      max_in = std::max(max_in, in.ranges.size());
      auto out = px4_drone::resampleScan(in, N);
      ++total;
      if (out.ranges.size() != N || kartoExpects(out) != N) {++bad;}
    }
    char msg[128];
    std::snprintf(msg, sizeof(msg), "%zu barridos reales (entrada %zu..%zu rayos): %zu salen distintos de %zu",
      total, min_in, max_in, bad, N);
    check(total > 0 && bad == 0, msg);
  }

  std::printf("\n%s (%d fallos)\n", failures == 0 ? "TODAS PASAN" : "HAY FALLOS", failures);
  return failures == 0 ? 0 : 1;
}
