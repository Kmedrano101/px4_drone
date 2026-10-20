#include "px4_drone/obstacle_guard.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace px4_drone
{

namespace
{
constexpr float kPi = 3.14159265358979f;
constexpr float kTwoPi = 2.0f * kPi;
// Holgura sobre el salto maximo fisico entre dos barridos: ruido del sensor y
// cambio del rayo que queda como min_points-esimo dentro del sector.
constexpr float kJumpToleranceM = 0.05f;
// Salto de rango que parte un grupo de rayos contiguos en dos objetos distintos.
constexpr float kClusterGapM = 0.08f;
// Huecos de indice tolerados dentro de un grupo: el filtro de suelo o un rayo
// sin retorno agujerean un objeto real sin partirlo de verdad.
constexpr std::size_t kClusterIndexGap = 2;

float wrapTwoPi(float a)
{
  a = std::fmod(a, kTwoPi);
  return a < 0.0f ? a + kTwoPi : a;
}

float toSignedDeg(float a)
{
  float d = wrapTwoPi(a) * 180.0f / kPi;
  return d > 180.0f ? d - 360.0f : d;
}

}  // namespace

ObstacleGuard::ObstacleGuard(const ObstacleGuardConfig & cfg)
: cfg_(cfg)
{
  cfg_.sector_count = std::max(cfg_.sector_count, 4);
  cfg_.min_points = std::max(cfg_.min_points, 1);
  cfg_.debounce_scans = std::max(cfg_.debounce_scans, 1);
  reset();
}

void ObstacleGuard::reset()
{
  history_.assign(static_cast<std::size_t>(cfg_.sector_count), {});
  sector_ranges_.assign(static_cast<std::size_t>(cfg_.sector_count), {});
  consecutive_ = 0;
  last_ = ObstacleStatus{};
  last_.nearest_m = std::numeric_limits<float>::quiet_NaN();
}

float ObstacleGuard::triggerDistance(float v) const
{
  v = std::clamp(v, 0.0f, cfg_.max_closing_speed_m_s);
  return cfg_.stop_distance_m + v * cfg_.reaction_time_s +
         (v * v) / (2.0f * std::max(cfg_.brake_decel_m_s2, 0.1f));
}

ObstacleStatus ObstacleGuard::update(const ScanInput & in)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::size_t n_sectors = static_cast<std::size_t>(cfg_.sector_count);
  const float sector_width = kTwoPi / static_cast<float>(cfg_.sector_count);
  for (auto & v : sector_ranges_) {
    v.clear();
  }
  kept_.clear();
  groups_.clear();

  // Componente vertical (NED, positiva hacia abajo) de un rayo horizontal del
  // cuerpo tras aplicar roll y pitch: tercera fila de R = Rz Ry Rx, que no
  // depende del yaw. Rayo en FRD = (cos a, -sin a, 0), porque a viene en FLU.
  const float sp = std::sin(in.pitch_rad);
  const float cp = std::cos(in.pitch_rad);
  const float sr = std::sin(in.roll_rad);
  const bool floor_check = std::isfinite(in.laser_height_m) && in.laser_height_m > 0.0f;

  ObstacleStatus st;
  const float lo = std::max(in.range_min, cfg_.min_valid_range_m);
  std::size_t valid = 0;
  for (std::size_t i = 0; i < in.count; ++i) {
    const float r = in.ranges[i];
    if (!std::isfinite(r) || r < lo || r > in.range_max) {
      continue;
    }
    const float a = in.angle_min + static_cast<float>(i) * in.angle_increment + in.laser_yaw_rad;
    ++valid;

    if (cfg_.blind_radius_m > 0.0f && r <= cfg_.blind_radius_m) {
      continue;
    }

    if (floor_check) {
      const float dz = -sp * std::cos(a) + sr * cp * (-std::sin(a));
      if (dz > 1e-3f && r >= cfg_.floor_margin * (in.laser_height_m / dz)) {
        ++st.floor_rejected;
        continue;
      }
    }

    kept_.push_back({i, r});
  }
  st.valid_fraction = in.count > 0 ? static_cast<float>(valid) / static_cast<float>(in.count) : 0.0f;

  // Un objeto es un grupo de rayos contiguos de rango parecido. Se mide su
  // cuerda real y se descarta el que no llegue al ancho minimo: separa polvo de
  // obstaculo sin depender de la distancia. En los bags del 2026-10-16 el polvo
  // que levantan las helices da cuerdas de 0.03-0.09 m dentro de la banda que
  // dispara; el obstaculo real del bag del accidente da 1.18-1.83 m y un pilar
  // de 0.40 m a 1 m da 0.40 m.
  const bool width_check = cfg_.min_obstacle_width_m > 0.0f &&
    std::fabs(in.angle_increment) > 0.0f && !kept_.empty();
  if (width_check) {
    groups_.emplace_back();
    groups_.back().push_back(0);
    for (std::size_t k = 1; k < kept_.size(); ++k) {
      const bool join = kept_[k].first - kept_[k - 1].first <= kClusterIndexGap &&
        std::fabs(kept_[k].second - kept_[k - 1].second) <= kClusterGapM;
      if (!join) {
        groups_.emplace_back();
      }
      groups_.back().push_back(k);
    }
    // El barrido cierra el circulo: si el ultimo grupo se toca con el primero al
    // cruzar el indice 0, son el mismo objeto.
    if (groups_.size() > 1 && in.count > 0) {
      const auto & f = kept_.front();
      const auto & l = kept_.back();
      if ((in.count - 1 - l.first) + f.first + 1 <= kClusterIndexGap &&
        std::fabs(l.second - f.second) <= kClusterGapM)
      {
        auto & first = groups_.front();
        const auto & last = groups_.back();
        first.insert(first.end(), last.begin(), last.end());
        groups_.pop_back();
      }
    }
  }

  const auto sector_of = [&](std::size_t ray_index) {
      const float a = in.angle_min + static_cast<float>(ray_index) * in.angle_increment +
        in.laser_yaw_rad;
      const std::size_t s = static_cast<std::size_t>(wrapTwoPi(a) / sector_width);
      return s >= n_sectors ? n_sectors - 1 : s;
    };

  if (!width_check) {
    for (const auto & kr : kept_) {
      sector_ranges_[sector_of(kr.first)].push_back(kr.second);
    }
  } else {
    std::vector<float> rs;
    for (const auto & g : groups_) {
      rs.clear();
      for (const std::size_t k : g) {
        rs.push_back(kept_[k].second);
      }
      std::nth_element(rs.begin(), rs.begin() + static_cast<std::ptrdiff_t>(rs.size() / 2),
        rs.end());
      const float med = rs[rs.size() / 2];
      const float arc = static_cast<float>(g.size()) * std::fabs(in.angle_increment);
      const float chord = 2.0f * med * std::sin(std::min(arc, kPi) * 0.5f);
      if (chord < cfg_.min_obstacle_width_m) {
        st.width_rejected += static_cast<int>(g.size());
        continue;
      }
      for (const std::size_t k : g) {
        sector_ranges_[sector_of(kept_[k].first)].push_back(kept_[k].second);
      }
    }
  }

  st.nearest_m = nan;
  float worst_excess = std::numeric_limits<float>::infinity();  // distancia - umbral; <0 dispara
  float nearest_speed = 0.0f;
  for (std::size_t s = 0; s < n_sectors; ++s) {
    auto & h = history_[s];
    auto & v = sector_ranges_[s];
    float dist = nan;
    if (static_cast<int>(v.size()) >= cfg_.min_points) {
      const auto k = static_cast<std::size_t>(cfg_.min_points - 1);
      std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
      dist = v[k];
    }

    if (!std::isfinite(dist)) {
      h.clear();  // el sector quedo libre: no arrastrar una distancia vieja
      continue;
    }

    // Un cambio mas rapido de lo que el dron puede moverse no es velocidad: es
    // otro objeto que entro al sector (o salio), por ejemplo al girar un poco
    // con una pared en el borde entre dos sectores. Se empieza de cero. Sin
    // esto, en hover aparecian "acercamientos" de 3 m/s que disparaban la
    // parada con el dron quieto (reproducido con el bag del 2026-09-17).
    if (!h.empty()) {
      const double dt = std::max(in.stamp_s - h.back().first, 1e-3);
      if (std::fabs(h.back().second - dist) > cfg_.max_closing_speed_m_s * dt + kJumpToleranceM) {
        h.clear();
      }
    }
    while (!h.empty() && in.stamp_s - h.front().first > cfg_.closing_window_s) {
      h.pop_front();
    }
    h.emplace_back(in.stamp_s, dist);

    // Velocidad de acercamiento = -pendiente de la recta distancia(t) de la
    // ventana (minimos cuadrados), con al menos 3 barridos y 0.15 s de datos.
    float speed = 0.0f;
    if (h.size() >= 3 && h.back().first - h.front().first >= 0.15) {
      double mt = 0.0, md = 0.0;
      for (const auto & p : h) {
        mt += p.first;
        md += p.second;
      }
      mt /= static_cast<double>(h.size());
      md /= static_cast<double>(h.size());
      double stt = 0.0, std_ = 0.0;
      for (const auto & p : h) {
        stt += (p.first - mt) * (p.first - mt);
        std_ += (p.first - mt) * (p.second - md);
      }
      if (stt > 0.0) {
        speed = std::clamp(static_cast<float>(-std_ / stt), 0.0f, cfg_.max_closing_speed_m_s);
      }
    }

    const float center = (static_cast<float>(s) + 0.5f) * sector_width;
    if (!std::isfinite(st.nearest_m) || dist < st.nearest_m) {
      st.nearest_m = dist;
      st.nearest_bearing_deg = toSignedDeg(center);
      nearest_speed = speed;
    }
    const float thr = triggerDistance(speed);
    if (dist - thr < worst_excess) {
      worst_excess = dist - thr;
      st.trigger_range_m = dist;
      st.trigger_bearing_deg = toSignedDeg(center);
      st.closing_speed_m_s = speed;
      st.trigger_distance_m = thr;
    }
  }
  if (!std::isfinite(worst_excess)) {
    st.closing_speed_m_s = nearest_speed;
    st.trigger_distance_m = triggerDistance(0.0f);
    st.trigger_range_m = nan;
  }

  st.raw_trigger = worst_excess < 0.0f;
  consecutive_ = st.raw_trigger ? consecutive_ + 1 : 0;
  st.triggered = consecutive_ >= cfg_.debounce_scans;
  last_ = st;
  return st;
}

}  // namespace px4_drone
