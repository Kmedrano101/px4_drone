// Pruebas del filtro de ancho minimo de ObstacleGuard, sin ROS ni dron.
//   g++ -O2 -I include tools/obstacle_guard/test_min_width.cpp src/obstacle_guard.cpp -o /tmp/t && /tmp/t
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "px4_drone/obstacle_guard.hpp"

namespace
{
constexpr std::size_t kRays = 506;       // LD19: 506 rayos/vuelta, 0.713 grados
constexpr float kInc = 2.0f * 3.14159265358979f / static_cast<float>(kRays);

int failures = 0;

void check(bool ok, const char * what)
{
  std::printf("%s  %s\n", ok ? "ok  " : "FALLA", what);
  if (!ok) {
    ++failures;
  }
}

// Barrido vacio con un objeto de ancho `width_m` centrado en `bearing_deg` a `range_m`.
std::vector<float> scanWithObject(float range_m, float width_m, float bearing_deg)
{
  std::vector<float> r(kRays, std::numeric_limits<float>::infinity());
  if (width_m <= 0.0f) {
    return r;
  }
  const float half = std::asin(std::min(1.0f, width_m / (2.0f * range_m)));
  const int n = std::max(1, static_cast<int>(std::lround(2.0f * half / kInc)));
  const int c = static_cast<int>(std::lround(bearing_deg * 3.14159265f / 180.0f / kInc));
  for (int k = -n / 2; k <= n / 2; ++k) {
    const int i = ((c + k) % static_cast<int>(kRays) + static_cast<int>(kRays)) %
      static_cast<int>(kRays);
    r[static_cast<std::size_t>(i)] = range_m;
  }
  return r;
}

px4_drone::ScanInput makeInput(const std::vector<float> & r, double t)
{
  px4_drone::ScanInput in;
  in.stamp_s = t;
  in.angle_min = 0.0f;
  in.angle_increment = kInc;
  in.range_min = 0.02f;
  in.range_max = 12.0f;
  in.ranges = r.data();
  in.count = r.size();
  in.laser_height_m = std::numeric_limits<float>::quiet_NaN();  // sin filtro de suelo
  return in;
}

// Dispara tras alimentar el mismo barrido `n` veces (cubre el debounce).
bool fires(px4_drone::ObstacleGuardConfig cfg, const std::vector<float> & r, int n = 4)
{
  px4_drone::ObstacleGuard g(cfg);
  bool t = false;
  for (int k = 0; k < n; ++k) {
    t = g.update(makeInput(r, 0.1 * k)).triggered || t;
  }
  return t;
}

}  // namespace

int main()
{
  px4_drone::ObstacleGuardConfig base;
  base.stop_distance_m = 1.0f;
  base.blind_radius_m = 0.35f;
  base.debounce_scans = 3;
  base.min_obstacle_width_m = 0.15f;

  std::printf("== un pilar de 40 cm se ve a toda distancia dentro del umbral ==\n");
  for (float d : {0.45f, 0.6f, 0.8f, 0.95f}) {
    char msg[96];
    std::snprintf(msg, sizeof(msg), "pilar de 0.40 m a %.2f m dispara", d);
    check(fires(base, scanWithObject(d, 0.40f, 0.0f)), msg);
  }

  std::printf("\n== objetos reales mas estrechos ==\n");
  check(fires(base, scanWithObject(0.9f, 0.20f, 0.0f)), "objeto de 0.20 m a 0.90 m dispara");
  check(fires(base, scanWithObject(0.9f, 0.16f, 0.0f)), "objeto de 0.16 m a 0.90 m dispara");

  std::printf("\n== lo que el filtro descarta a proposito ==\n");
  check(!fires(base, scanWithObject(0.9f, 0.06f, 0.0f)),
    "grupo de 0.06 m a 0.90 m (polvo medido en los bags) NO dispara");
  check(!fires(base, scanWithObject(0.9f, 0.09f, 0.0f)),
    "grupo de 0.09 m a 0.90 m (polvo peor de los bags) NO dispara");

  std::printf("\n== el filtro escala con la distancia, min_points no ==\n");
  // Mismo objeto de 0.30 m: a 0.5 m son muchos rayos, a 0.95 m son pocos.
  check(fires(base, scanWithObject(0.5f, 0.30f, 0.0f)), "objeto de 0.30 m a 0.50 m dispara");
  check(fires(base, scanWithObject(0.95f, 0.30f, 0.0f)), "el mismo a 0.95 m dispara igual");

  std::printf("\n== el grupo que cruza el indice 0 es un solo objeto ==\n");
  // Centrado en el rayo 0: sin cerrar el circulo quedaria partido en dos mitades
  // de menos de 0.15 m y se descartaria.
  check(fires(base, scanWithObject(0.9f, 0.18f, 0.0f)),
    "objeto de 0.18 m centrado en el rayo 0 dispara");

  std::printf("\n== el filtro desactivado no cambia nada ==\n");
  px4_drone::ObstacleGuardConfig off = base;
  off.min_obstacle_width_m = 0.0f;
  check(fires(off, scanWithObject(0.9f, 0.06f, 0.0f)),
    "con ancho 0 el grupo de 0.06 m SI dispara (comportamiento anterior)");

  std::printf("\n== la zona ciega sigue por delante del filtro ==\n");
  check(!fires(base, scanWithObject(0.30f, 0.40f, 0.0f)),
    "objeto ancho a 0.30 m (dentro de la zona ciega) NO dispara");

  std::printf("\n== fuera del umbral no dispara ==\n");
  check(!fires(base, scanWithObject(2.5f, 0.40f, 0.0f)),
    "pilar de 0.40 m a 2.50 m NO dispara");

  std::printf("\n== barrido vacio ==\n");
  check(!fires(base, scanWithObject(1.0f, 0.0f, 0.0f)), "sin retornos NO dispara");

  std::printf("\n%s (%d fallos)\n", failures == 0 ? "TODAS PASAN" : "HAY FALLOS", failures);
  return failures == 0 ? 0 : 1;
}
