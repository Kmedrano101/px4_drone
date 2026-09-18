#ifndef PX4_DRONE__OBSTACLE_GUARD_HPP_
#define PX4_DRONE__OBSTACLE_GUARD_HPP_

#include <cstddef>
#include <deque>
#include <utility>
#include <vector>

// Parada por obstaculo con el LiDAR 2D, usando SOLO los barridos crudos.
//
// Por que no usa ni el SLAM ni la posicion del EKF: en el vuelo del 2026-09-17
// (patron en cruz, log 158) el SLAM perdio el tracking y dijo que el dron
// estaba quieto mientras recorria ~3 m a ~1.2 m/s; el EKF se lo creyo. Lo unico
// que vio bien el obstaculo acercarse fue /scan (distancia minima de 3.4 a
// 0.47 m). Ver reports/2026-09-18_takeoff-ev-log158-caida-stabilized.md en
// px4_drone_dev.
//
// El barrido se divide en sectores. En cada sector la distancia del obstaculo
// es la del min_points-esimo rayo mas cercano (un solo rayo no dispara nada), y
// la velocidad de acercamiento sale de como baja esa distancia en los ultimos
// barridos. El sector dispara si el obstaculo esta mas cerca que
//     stop_distance + v * reaction_time + v^2 / (2 * brake_decel)
// es decir, la distancia a la que hay que empezar a frenar para quedar parado a
// stop_distance. Con el dron inclinado el plano del laser puede cortar el
// suelo: los rayos que miden lo que mediria el suelo se descartan.
//
// C++ puro, sin ROS, para poder reproducir barridos grabados fuera del dron
// (ver src/obstacle_guard_replay.cpp).
namespace px4_drone
{

struct ObstacleGuardConfig
{
  float stop_distance_m{1.0f};     // distancia a la que el dron tiene que quedar parado
  float min_valid_range_m{0.15f};  // por debajo: el propio dron (cables, antenas) o ruido
  float reaction_time_s{0.35f};    // barrido a 10 Hz + debounce + lazo a 20 Hz + DDS y PX4
  float brake_decel_m_s2{2.0f};    // frenada supuesta; MPC_ACC_HOR es 3.0, se deja margen
  float max_closing_speed_m_s{2.5f};  // mas rapido que esto no es el dron: es otro objeto en el sector
  int sector_count{36};            // 10 grados por sector
  int min_points{3};               // rayos a esa distancia o menos para creerse el obstaculo
  int debounce_scans{2};           // barridos seguidos disparando para dar la parada
  float closing_window_s{0.4f};    // ventana para estimar la velocidad de acercamiento
  float floor_margin{0.85f};       // rayo que mide >= este % de la distancia al suelo = suelo
  float min_valid_fraction{0.10f};  // menos rayos validos que esto = sensor tapado o roto
};

struct ScanInput
{
  double stamp_s{0.0};
  float angle_min{0.0f};           // convencion ROS: FLU, antihorario positivo
  float angle_increment{0.0f};
  float range_min{0.0f};
  float range_max{0.0f};
  const float * ranges{nullptr};
  std::size_t count{0};
  float laser_yaw_rad{0.0f};       // giro del laser respecto al cuerpo (FLU)
  float roll_rad{0.0f};            // actitud del cuerpo (FRD -> NED)
  float pitch_rad{0.0f};
  float laser_height_m{0.0f};      // altura del plano del laser sobre el suelo; NaN = desconocida
};

struct ObstacleStatus
{
  bool triggered{false};         // con debounce: hay que parar
  bool raw_trigger{false};       // este barrido, sin debounce
  float nearest_m{0.0f};         // obstaculo mas cercano (robusto); NaN si no hay ninguno
  float nearest_bearing_deg{0.0f};  // en el cuerpo: 0 = morro, +90 = izquierda
  float closing_speed_m_s{0.0f};    // del sector que dispara (o del mas cercano)
  float trigger_distance_m{0.0f};   // umbral de ese sector
  float trigger_range_m{0.0f};      // distancia del sector que dispara
  float trigger_bearing_deg{0.0f};
  int floor_rejected{0};         // rayos descartados por ser suelo
  float valid_fraction{0.0f};    // rayos validos / total
};

class ObstacleGuard
{
public:
  explicit ObstacleGuard(const ObstacleGuardConfig & cfg);

  ObstacleStatus update(const ScanInput & in);
  const ObstacleStatus & last() const {return last_;}
  const ObstacleGuardConfig & config() const {return cfg_;}
  void reset();

  // Distancia a la que hay que empezar a frenar acercandose a v m/s.
  float triggerDistance(float closing_speed_m_s) const;

private:
  ObstacleGuardConfig cfg_;
  std::vector<std::deque<std::pair<double, float>>> history_;  // por sector: (t, distancia)
  std::vector<std::vector<float>> sector_ranges_;  // reutilizado entre barridos
  int consecutive_{0};
  ObstacleStatus last_;
};

}  // namespace px4_drone

#endif  // PX4_DRONE__OBSTACLE_GUARD_HPP_
