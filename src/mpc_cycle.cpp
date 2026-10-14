#include "px4_drone/mpc_cycle.hpp"

#include <chrono>
#include <cstring>
#include <vector>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_px4_cycle_mpc.h"
}

namespace px4_drone
{

namespace
{
constexpr int NX = 9;   // [p(3), v(3), a(3)]
constexpr int NU = 3;   // jerk
constexpr int NY = NX + NU;
// Orden de constraints.idxbx del generador: [pz, vx, vy, vz, ax, ay, az]
constexpr int NBX = 7;
}  // namespace

struct MpcCycle::Impl
{
  px4_cycle_mpc_solver_capsule * capsule{nullptr};
  ocp_nlp_config * cfg{nullptr};
  ocp_nlp_dims * dims{nullptr};
  ocp_nlp_in * in{nullptr};
  ocp_nlp_out * out{nullptr};
  int N{0};
  double dt{0.0};
};

MpcCycle::MpcCycle()
: impl_(new Impl())
{
  impl_->capsule = px4_cycle_mpc_acados_create_capsule();
  const int status = px4_cycle_mpc_acados_create(impl_->capsule);
  if (status != 0) {
    error_ = "px4_cycle_mpc_acados_create devolvio " + std::to_string(status);
    return;
  }
  impl_->cfg = px4_cycle_mpc_acados_get_nlp_config(impl_->capsule);
  impl_->dims = px4_cycle_mpc_acados_get_nlp_dims(impl_->capsule);
  impl_->in = px4_cycle_mpc_acados_get_nlp_in(impl_->capsule);
  impl_->out = px4_cycle_mpc_acados_get_nlp_out(impl_->capsule);
  impl_->N = PX4_CYCLE_MPC_N;
  // El paso se lee del propio solver, no se fija a mano: asi el nodo no puede
  // quedar descuadrado respecto al solver que se haya generado en la maquina.
  double ts = 0.0;
  ocp_nlp_in_get(impl_->cfg, impl_->dims, impl_->in, 0, "Ts", &ts);
  impl_->dt = ts;
  ready_ = true;
}

MpcCycle::~MpcCycle()
{
  if (impl_ != nullptr) {
    if (impl_->capsule != nullptr) {
      px4_cycle_mpc_acados_free(impl_->capsule);
      px4_cycle_mpc_acados_free_capsule(impl_->capsule);
    }
    delete impl_;
  }
}

int MpcCycle::horizon() const {return impl_ != nullptr ? impl_->N : 0;}
double MpcCycle::dt() const {return impl_ != nullptr ? impl_->dt : 0.0;}

void MpcCycle::setLimits(const MpcLimits & l)
{
  if (!ready_) {
    return;
  }
  // NED: z hacia abajo. Subir es vz negativa, bajar vz positiva.
  const double lbx[NBX] = {-50.0, -l.v_xy, -l.v_xy, -l.v_up, -l.a_xy, -l.a_xy, -l.a_up};
  const double ubx[NBX] = {l.z_floor_ned, l.v_xy, l.v_xy, l.v_dn, l.a_xy, l.a_xy, l.a_dn};
  const double lbu[NU] = {-l.j_max, -l.j_max, -l.j_max};
  const double ubu[NU] = {l.j_max, l.j_max, l.j_max};

  for (int k = 1; k <= impl_->N; ++k) {   // la etapa 0 la fija el estado medido
    ocp_nlp_constraints_model_set(
      impl_->cfg, impl_->dims, impl_->in, impl_->out, k, "lbx", const_cast<double *>(lbx));
    ocp_nlp_constraints_model_set(
      impl_->cfg, impl_->dims, impl_->in, impl_->out, k, "ubx", const_cast<double *>(ubx));
  }
  for (int k = 0; k < impl_->N; ++k) {
    ocp_nlp_constraints_model_set(
      impl_->cfg, impl_->dims, impl_->in, impl_->out, k, "lbu", const_cast<double *>(lbu));
    ocp_nlp_constraints_model_set(
      impl_->cfg, impl_->dims, impl_->in, impl_->out, k, "ubu", const_cast<double *>(ubu));
  }
}

void MpcCycle::setWeights(const MpcWeights & w)
{
  if (!ready_) {
    return;
  }
  // W es diagonal, asi que el orden por filas o columnas da igual.
  std::vector<double> W(NY * NY, 0.0);
  const double diag[NY] = {
    w.q_pos_xy, w.q_pos_xy, w.q_pos_z,
    w.q_vel_xy, w.q_vel_xy, w.q_vel_z,
    w.q_acc, w.q_acc, w.q_acc,
    w.r_jerk, w.r_jerk, w.r_jerk};
  for (int i = 0; i < NY; ++i) {
    W[i * NY + i] = diag[i];
  }
  std::vector<double> We(NX * NX, 0.0);
  for (int i = 0; i < NX; ++i) {
    We[i * NX + i] = 3.0 * diag[i];   // coste terminal: margen de estabilidad
  }
  for (int k = 0; k < impl_->N; ++k) {
    ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, k, "W", W.data());
  }
  ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, impl_->N, "W", We.data());
}

MpcResult MpcCycle::solve(
  const float p[3], const float v[3], const float a_prev[3],
  const float p_ref[3], const float v_ref[3])
{
  MpcResult r;
  if (!ready_) {
    return r;
  }

  double x0[NX];
  for (int i = 0; i < 3; ++i) {
    x0[i] = p[i];
    x0[3 + i] = v[i];
    x0[6 + i] = a_prev[i];
  }
  ocp_nlp_constraints_model_set(impl_->cfg, impl_->dims, impl_->in, impl_->out, 0, "lbx", x0);
  ocp_nlp_constraints_model_set(impl_->cfg, impl_->dims, impl_->in, impl_->out, 0, "ubx", x0);

  double yref[NY] = {0.0};
  for (int i = 0; i < 3; ++i) {
    yref[i] = p_ref[i];
    yref[3 + i] = v_ref[i];
  }  // aceleracion y jerk de referencia a cero: queremos acabar quietos y suaves
  for (int k = 0; k < impl_->N; ++k) {
    ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, k, "yref", yref);
  }
  ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, impl_->N, "yref", yref);

  const auto t0 = std::chrono::steady_clock::now();
  r.status = px4_cycle_mpc_acados_solve(impl_->capsule);
  r.solve_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();

  double x1[NX];
  ocp_nlp_out_get(impl_->cfg, impl_->dims, impl_->out, 1, "x", x1);
  for (int i = 0; i < 3; ++i) {
    r.vel_pred[i] = static_cast<float>(x1[3 + i]);
    r.acc[i] = static_cast<float>(x1[6 + i]);
  }
  // 0 = convergido, 2 = maxiter. En SQP_RTI se hace UNA iteracion por ciclo, asi
  // que 2 es lo normal y la solucion sigue siendo utilizable.
  r.ok = (r.status == 0 || r.status == 2);
  return r;
}

}  // namespace px4_drone
