#ifndef PX4_DRONE__MPC_CYCLE_HPP_
#define PX4_DRONE__MPC_CYCLE_HPP_

#include <array>
#include <string>

// Envoltorio del solver MPC generado con acados (ver scripts/mpc/
// generate_cycle_mpc.py). Aisla al nodo de los tipos de acados, que cambian
// entre versiones: el solver se genera en cada maquina con su propia version.
//
// Modelo: triple integrador por eje en NED, con perturbacion como parametro.
//   x = [p(3), v(3), a(3)], u = jerk(3), p = d(3)
//   dinamica p'=v, v'=a+d, a'=j
// Con d = 0 (el valor por defecto si nadie llama a setDisturbance) el modelo es
// el triple integrador puro de siempre.
// La salida util es la ACELERACION del siguiente paso, que es lo que se publica
// al FC (ver TakeoffPositionHoldBase::publishAccelerationSetpoint).
namespace px4_drone
{

struct MpcLimits
{
  float v_xy{1.0f};        // m/s
  float v_up{1.0f};        // m/s subiendo (positivo)
  float v_dn{0.8f};        // m/s bajando (positivo)
  float a_xy{2.0f};        // m/s^2
  float a_up{2.0f};        // m/s^2
  float a_dn{2.0f};        // m/s^2
  float j_max{6.0f};       // m/s^3  (la que da la suavidad)
  float z_floor_ned{0.0f}; // pz <= z_floor: no bajar del suelo (NED, z hacia abajo)
};

struct MpcWeights
{
  float q_pos_xy{30.0f};
  float q_pos_z{60.0f};
  float q_vel_xy{8.0f};
  float q_vel_z{12.0f};
  float q_acc{1.0f};
  float r_jerk{0.5f};
};

struct MpcResult
{
  bool ok{false};
  int status{-1};             // 0 = resuelto; 2 = maxiter (aceptable en RTI)
  double solve_ms{0.0};
  std::array<float, 3> acc{{0.0f, 0.0f, 0.0f}};     // aceleracion a publicar (NED)
  std::array<float, 3> vel_pred{{0.0f, 0.0f, 0.0f}};  // velocidad prevista en dt
};

class MpcCycle
{
public:
  MpcCycle();
  ~MpcCycle();
  MpcCycle(const MpcCycle &) = delete;
  MpcCycle & operator=(const MpcCycle &) = delete;

  bool ready() const {return ready_;}
  const std::string & error() const {return error_;}
  int horizon() const;
  double dt() const;

  void setLimits(const MpcLimits & lim);
  void setWeights(const MpcWeights & w);

  // Perturbacion de aceleracion (NED, m/s^2) que el modelo suma a la
  // aceleracion comandada: v' = a + d. La estima un observador en el nodo. Con
  // d = 0 el comportamiento es el de antes, asi que el mismo solver sirve para
  // las dos variantes.
  void setDisturbance(const float d[3]);

  // p, v: estado medido (NED). a_prev: ultima aceleracion comandada, que hace
  // de tercer estado (el dron no mide su aceleracion deseada).
  // p_ref, v_ref: referencia constante en el horizonte.
  MpcResult solve(
    const float p[3], const float v[3], const float a_prev[3],
    const float p_ref[3], const float v_ref[3]);

private:
  struct Impl;
  Impl * impl_{nullptr};
  bool ready_{false};
  std::string error_;
};

}  // namespace px4_drone

#endif  // PX4_DRONE__MPC_CYCLE_HPP_
