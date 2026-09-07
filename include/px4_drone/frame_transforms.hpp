#ifndef PX4_DRONE__FRAME_TRANSFORMS_HPP_
#define PX4_DRONE__FRAME_TRANSFORMS_HPP_

#include <array>
#include <cmath>

// Conversion entre las convenciones de frame de ROS 2 (ENU / FLU) y PX4 (NED / FRD).
//
// ROS 2 (REP-103): mundo ENU (X=Este, Y=Norte, Z=Arriba), cuerpo FLU
// (X=Adelante, Y=Izquierda, Z=Arriba).
// PX4: mundo NED (X=Norte, Y=Este, Z=Abajo), cuerpo FRD
// (X=Adelante, Y=Derecha, Z=Abajo).
//
// Todas las funciones de esta cabecera son involutivas (aplicarlas dos veces
// devuelve el valor original), asi que la misma funcion sirve para convertir
// en ambos sentidos.
namespace px4_drone::frames
{

using Vec3 = std::array<float, 3>;
using Quat = std::array<float, 4>;  // orden [w, x, y, z], igual que px4_msgs

// Posicion o velocidad: ENU <-> NED (permutacion de ejes + inversion de Z).
inline Vec3 enuNedSwap(const Vec3 & v)
{
  return {v[1], v[0], -v[2]};
}

// Yaw (heading) puro: ENU (0 = Este, antihorario) <-> NED (0 = Norte, horario).
inline float wrapToPi(float angle)
{
  while (angle > static_cast<float>(M_PI)) {angle -= 2.0f * static_cast<float>(M_PI);}
  while (angle < -static_cast<float>(M_PI)) {angle += 2.0f * static_cast<float>(M_PI);}
  return angle;
}

inline float yawEnuNedSwap(float yaw)
{
  return wrapToPi(static_cast<float>(M_PI) / 2.0f - yaw);
}

inline Quat quatMultiply(const Quat & a, const Quat & b)
{
  return {
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
  };
}

// Rotacion estatica mundo ENU<->NED (180 grados sobre el eje "diagonal"
// X+Y a 45 grados) y cuerpo FRD<->FLU (180 grados sobre X). Ambas son de
// 180 grados, por lo que son su propia inversa: la misma composicion
// convierte una orientacion completa (mundo+cuerpo) en cualquiera de los
// dos sentidos.
inline constexpr Quat kNedEnuQ{0.0f, 0.70710678f, 0.70710678f, 0.0f};
inline constexpr Quat kAircraftBaselinkQ{0.0f, 1.0f, 0.0f, 0.0f};

inline Quat quatFrameBodySwap(const Quat & q)
{
  return quatMultiply(quatMultiply(kNedEnuQ, q), kAircraftBaselinkQ);
}

}  // namespace px4_drone::frames

#endif  // PX4_DRONE__FRAME_TRANSFORMS_HPP_
