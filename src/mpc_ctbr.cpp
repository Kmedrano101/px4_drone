#include "px4_drone/mpc_ctbr.hpp"

#include <chrono>
#include <cmath>
#include <vector>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_px4_ctbr_mpc.h"
}

namespace px4_drone
{

namespace
{
constexpr int NX = 10;   // [p(3), v(3), q(4)]
constexpr int NU = 4;    // [T, wx, wy, wz]
constexpr int NY = NX + NU;
constexpr int NBX = 4;   // orden del generador: [pz, vx, vy, vz]
constexpr float kG = 9.80665f;
}  // namespace

struct MpcCtbr::Impl
{
  px4_ctbr_mpc_solver_capsule * capsule{nullptr};
  ocp_nlp_config * cfg{nullptr};
  ocp_nlp_dims * dims{nullptr};
  ocp_nlp_in * in{nullptr};
  ocp_nlp_out * out{nullptr};
  int N{0};
  double dt{0.0};
};

MpcCtbr::MpcCtbr()
: impl_(new Impl())
{
  impl_->capsule = px4_ctbr_mpc_acados_create_capsule();
  const int status = px4_ctbr_mpc_acados_create(impl_->capsule);
  if (status != 0) {
    error_ = "px4_ctbr_mpc_acados_create devolvio " + std::to_string(status);
    return;
  }
  impl_->cfg = px4_ctbr_mpc_acados_get_nlp_config(impl_->capsule);
  impl_->dims = px4_ctbr_mpc_acados_get_nlp_dims(impl_->capsule);
  impl_->in = px4_ctbr_mpc_acados_get_nlp_in(impl_->capsule);
  impl_->out = px4_ctbr_mpc_acados_get_nlp_out(impl_->capsule);
  impl_->N = PX4_CTBR_MPC_N;
  double ts = 0.0;
  ocp_nlp_in_get(impl_->cfg, impl_->dims, impl_->in, 0, "Ts", &ts);
  impl_->dt = ts;
  ready_ = true;
}

MpcCtbr::~MpcCtbr()
{
  if (impl_ != nullptr) {
    if (impl_->capsule != nullptr) {
      px4_ctbr_mpc_acados_free(impl_->capsule);
      px4_ctbr_mpc_acados_free_capsule(impl_->capsule);
    }
    delete impl_;
  }
}

int MpcCtbr::horizon() const {return impl_ != nullptr ? impl_->N : 0;}
double MpcCtbr::dt() const {return impl_ != nullptr ? impl_->dt : 0.0;}

void MpcCtbr::setLimits(const CtbrLimits & l)
{
  if (!ready_) {
    return;
  }
  // NED: subir es vz negativa, bajar vz positiva.
  const double lbx[NBX] = {-50.0, -l.v_xy, -l.v_xy, -l.v_up};
  const double ubx[NBX] = {l.z_floor_ned, l.v_xy, l.v_xy, l.v_dn};
  // El empuje nunca negativo: un multirrotor no tira hacia abajo.
  const double lbu[NU] = {l.t_min * kG, -l.w_xy, -l.w_xy, -l.w_z};
  const double ubu[NU] = {l.t_max * kG, l.w_xy, l.w_xy, l.w_z};

  for (int k = 1; k <= impl_->N; ++k) {
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

void MpcCtbr::setWeights(const CtbrWeights & w)
{
  if (!ready_) {
    return;
  }
  const double diag[NY] = {
    w.q_pos_xy, w.q_pos_xy, w.q_pos_z,
    w.q_vel_xy, w.q_vel_xy, w.q_vel_z,
    w.q_att_yaw, w.q_att_tilt, w.q_att_tilt, w.q_att_yaw,
    w.r_thrust, w.r_rate, w.r_rate, w.r_rate};
  std::vector<double> W(NY * NY, 0.0);
  for (int i = 0; i < NY; ++i) {
    W[i * NY + i] = diag[i];
  }
  std::vector<double> We(NX * NX, 0.0);
  for (int i = 0; i < NX; ++i) {
    We[i * NX + i] = 3.0 * diag[i];
  }
  for (int k = 0; k < impl_->N; ++k) {
    ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, k, "W", W.data());
  }
  ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, impl_->N, "W", We.data());
}

CtbrResult MpcCtbr::solve(
  const float p[3], const float v[3], const float q[4],
  const float p_ref[3], float yaw_ref)
{
  CtbrResult r;
  if (!ready_) {
    return r;
  }

  double x0[NX];
  for (int i = 0; i < 3; ++i) {
    x0[i] = p[i];
    x0[3 + i] = v[i];
  }
  // El cuaternion se normaliza antes de entrar: el solver integra q sin
  // reproyectarlo, y entrar con norma distinta de 1 sesga la direccion del
  // empuje (el modelo usa la tercera columna de R(q)).
  double n = 0.0;
  for (int i = 0; i < 4; ++i) {
    n += static_cast<double>(q[i]) * q[i];
  }
  n = (n > 1e-9) ? std::sqrt(n) : 1.0;
  const double qn[4] = {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
  // Signo canonico: q y -q son la misma actitud, pero el coste es lineal sobre
  // el cuaternion, asi que hay que quedarse en el hemisferio de qw >= 0 o el
  // coste empujaria a dar la vuelta.
  const double sgn = (qn[0] < 0.0) ? -1.0 : 1.0;
  for (int i = 0; i < 4; ++i) {
    x0[6 + i] = sgn * qn[i];
  }
  ocp_nlp_constraints_model_set(impl_->cfg, impl_->dims, impl_->in, impl_->out, 0, "lbx", x0);
  ocp_nlp_constraints_model_set(impl_->cfg, impl_->dims, impl_->in, impl_->out, 0, "ubx", x0);

  // Referencia: posicion pedida, velocidad cero, actitud nivelada con el yaw
  // pedido (q = [cos(yaw/2), 0, 0, sin(yaw/2)]), empuje de hover y tasas cero.
  double yref[NY] = {0.0};
  for (int i = 0; i < 3; ++i) {
    yref[i] = p_ref[i];
  }
  yref[6] = std::cos(0.5 * static_cast<double>(yaw_ref));
  yref[9] = std::sin(0.5 * static_cast<double>(yaw_ref));
  if (yref[6] < 0.0) {
    yref[6] = -yref[6];
    yref[9] = -yref[9];
  }
  yref[NX] = kG;
  for (int k = 0; k < impl_->N; ++k) {
    ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, k, "yref", yref);
  }
  ocp_nlp_cost_model_set(impl_->cfg, impl_->dims, impl_->in, impl_->N, "yref", yref);

  const auto t0 = std::chrono::steady_clock::now();
  r.status = px4_ctbr_mpc_acados_solve(impl_->capsule);
  r.solve_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();

  double u0[NU];
  ocp_nlp_out_get(impl_->cfg, impl_->dims, impl_->out, 0, "u", u0);
  r.thrust_sp = static_cast<float>(u0[0]);
  for (int i = 0; i < 3; ++i) {
    r.rates[i] = static_cast<float>(u0[1 + i]);
  }
  // 0 = convergido, 2 = maxiter: en SQP_RTI se hace una iteracion por ciclo.
  r.ok = (r.status == 0 || r.status == 2);
  return r;
}

}  // namespace px4_drone
