// Reproduce barridos grabados a traves de ObstacleGuard, sin ROS ni dron.
//
// Entrada por stdin, una linea por barrido (la genera
// tools/obstacle_guard/bag_to_scans.py a partir de un rosbag):
//   t,roll,pitch,laser_h,angle_min,angle_inc,range_min,range_max,n,r0,...,r(n-1)
// (rangos no validos como nan). Uso:
//   obstacle_guard_replay [stop_distance_m] < scans.csv
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "px4_drone/obstacle_guard.hpp"

int main(int argc, char * argv[])
{
  px4_drone::ObstacleGuardConfig cfg;
  if (argc > 1) {
    cfg.stop_distance_m = std::strtof(argv[1], nullptr);
  }
  px4_drone::ObstacleGuard guard(cfg);

  std::printf(
    "stop_distance %.2f m, reaccion %.2f s, frenada %.1f m/s2, %d sectores, %d rayos, debounce %d\n",
    cfg.stop_distance_m, cfg.reaction_time_s, cfg.brake_decel_m_s2, cfg.sector_count,
    cfg.min_points, cfg.debounce_scans);
  std::printf("      t  cercano  rumbo  v_acerc  sector_disp  umbral  suelo  valido  estado\n");

  std::string line;
  bool first_trigger_seen = false;
  std::vector<float> ranges;
  while (std::getline(std::cin, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::stringstream ss(line);
    std::string tok;
    std::vector<double> head;
    for (int i = 0; i < 9 && std::getline(ss, tok, ','); ++i) {
      head.push_back(std::strtod(tok.c_str(), nullptr));
    }
    if (head.size() < 9) {
      continue;
    }
    ranges.clear();
    while (std::getline(ss, tok, ',')) {
      ranges.push_back(std::strtof(tok.c_str(), nullptr));  // "nan" -> NaN
    }

    px4_drone::ScanInput in;
    in.stamp_s = head[0];
    in.roll_rad = static_cast<float>(head[1]);
    in.pitch_rad = static_cast<float>(head[2]);
    in.laser_height_m = static_cast<float>(head[3]);
    in.angle_min = static_cast<float>(head[4]);
    in.angle_increment = static_cast<float>(head[5]);
    in.range_min = static_cast<float>(head[6]);
    in.range_max = static_cast<float>(head[7]);
    in.ranges = ranges.data();
    in.count = ranges.size();

    const auto st = guard.update(in);
    const char * estado = st.triggered ? "PARAR" : (st.raw_trigger ? "disparo" : "");
    std::printf(
      "%7.2f  %6.2f  %+5.0f  %6.2f     %6.2f@%+4.0f  %6.2f  %5d  %5.0f%%  %s\n",
      in.stamp_s, st.nearest_m, st.nearest_bearing_deg, st.closing_speed_m_s, st.trigger_range_m,
      st.trigger_bearing_deg, st.trigger_distance_m, st.floor_rejected, 100.0f * st.valid_fraction,
      estado);
    if (st.triggered && !first_trigger_seen) {
      first_trigger_seen = true;
      std::printf(
        ">>> PRIMERA PARADA en t=%.2f s: obstaculo a %.2f m (rumbo %+.0f), acercandose a %.2f m/s, "
        "umbral %.2f m\n",
        in.stamp_s, st.trigger_range_m, st.trigger_bearing_deg, st.closing_speed_m_s,
        st.trigger_distance_m);
    }
  }
  if (!first_trigger_seen) {
    std::printf(">>> No habria parado en ningun momento.\n");
  }
  return 0;
}
