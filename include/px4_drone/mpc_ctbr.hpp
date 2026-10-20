#ifndef PX4_DRONE__MPC_CTBR_HPP_
#define PX4_DRONE__MPC_CTBR_HPP_

#include <array>
#include <string>

// Envoltorio del NMPC de empuje colectivo + velocidades angulares (CTBR),
// generado con scripts/mpc/generate_ctbr_mpc.py.
//
// A diferencia de MpcCycle (triple integrador cinematico que devuelve
// ACELERACION y deja a PX4 cerrar actitud y tasas), este modelo incluye la
// ACTITUD y devuelve lo que PX4 necesita para su lazo de tasas:
//   x = [p(3), v(3), q(4)]  (NED + cuaternion cuerpo->NED)
//   u = [T, wx, wy, wz]     T = empuje especifico (m/s^2), w en FRD (rad/s)
//
// ⚠️ Con offboard_control_mode.body_rate, PX4 1.14.3 no ejecuta mc_pos_control
// (control_mode.cpp:169-175): desaparecen la maquina de despegue, el limite de
// inclinacion y el estimador de empuje de hover. Ver docs/MPC_CICLO_COMPLETO.md.
namespace px4_drone
{

struct CtbrLimits
{
  float v_xy{1.0f};          // m/s
  float v_up{1.0f};          // m/s subiendo (positivo)
  float v_dn{0.8f};          // m/s bajando (positivo)
  float t_min{0.5f};         // empuje especifico minimo, en multiplos de g
  float t_max{1.6f};         // maximo, en multiplos de g
  float w_xy{1.5f};          // rad/s en roll y pitch
  float w_z{1.0f};           // rad/s en yaw
  float z_floor_ned{0.0f};   // pz <= z_floor (NED, z hacia abajo)
};

struct CtbrWeights
{
  float q_pos_xy{30.0f};
  float q_pos_z{60.0f};
  float q_vel_xy{8.0f};
  float q_vel_z{12.0f};
  float q_att_tilt{20.0f};   // qx, qy: inclinacion
  float q_att_yaw{5.0f};     // qw, qz
  float r_thrust{0.5f};
  float r_rate{2.0f};
};

struct CtbrResult
{
  bool ok{false};
  int status{-1};
  double solve_ms{0.0};
  float thrust_sp{9.80665f};                      // empuje especifico (m/s^2)
  std::array<float, 3> rates{{0.0f, 0.0f, 0.0f}};  // wx, wy, wz (FRD, rad/s)
};

class MpcCtbr
{
public:
  MpcCtbr();
  ~MpcCtbr();
  MpcCtbr(const MpcCtbr &) = delete;
  MpcCtbr & operator=(const MpcCtbr &) = delete;

  bool ready() const {return ready_;}
  const std::string & error() const {return error_;}
  int horizon() const;
  double dt() const;

  void setLimits(const CtbrLimits & lim);
  void setWeights(const CtbrWeights & w);

  // q en [w, x, y, z] (cuerpo FRD -> NED), como lo publica PX4 en
  // VehicleAttitude. yaw_ref se usa para la referencia de cuaternion.
  CtbrResult solve(
    const float p[3], const float v[3], const float q[4],
    const float p_ref[3], float yaw_ref);

private:
  struct Impl;
  Impl * impl_{nullptr};
  bool ready_{false};
  std::string error_;
};

}  // namespace px4_drone

#endif  // PX4_DRONE__MPC_CTBR_HPP_
