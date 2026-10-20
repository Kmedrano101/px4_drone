/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef ACADOS_SIM_px4_ctbr_mpc_H_
#define ACADOS_SIM_px4_ctbr_mpc_H_

#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

#define PX4_CTBR_MPC_NX     10
#define PX4_CTBR_MPC_NZ     0
#define PX4_CTBR_MPC_NU     4
#define PX4_CTBR_MPC_NP     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct px4_ctbr_mpc_sim_solver_capsule
{
    // acados objects
    sim_in *acados_sim_in;
    sim_out *acados_sim_out;
    sim_solver *acados_sim_solver;
    sim_opts *acados_sim_opts;
    sim_config *acados_sim_config;
    void *acados_sim_dims;
    void *acados_sim_mem;

    /* external functions */
    // ERK
    external_function_param_casadi * sim_expl_vde_forw;
    external_function_param_casadi * sim_vde_adj_casadi;
    external_function_param_casadi * sim_expl_ode_fun_casadi;
    external_function_param_casadi * sim_expl_ode_hess;
    external_function_param_casadi * sim_expl_vde_forw_p;

    // IRK
    external_function_param_casadi * sim_impl_dae_fun;
    external_function_param_casadi * sim_impl_dae_fun_jac_x_xdot_z;
    external_function_param_casadi * sim_impl_dae_jac_x_xdot_u_z;
    external_function_param_casadi * sim_impl_dae_hess;
    external_function_param_casadi * sim_impl_dae_jac_p;

    // GNSF
    external_function_param_casadi * sim_gnsf_phi_fun;
    external_function_param_casadi * sim_gnsf_phi_fun_jac_y;
    external_function_param_casadi * sim_gnsf_phi_jac_y_uhat;
    external_function_param_casadi * sim_gnsf_f_lo_jac_x1_x1dot_u_z;
    external_function_param_casadi * sim_gnsf_get_matrices_fun;

} px4_ctbr_mpc_sim_solver_capsule;


ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_sim_create(px4_ctbr_mpc_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_sim_solve(px4_ctbr_mpc_sim_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_sim_free(px4_ctbr_mpc_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_sim_update_params(px4_ctbr_mpc_sim_solver_capsule *capsule, double *value, int np);

ACADOS_SYMBOL_EXPORT sim_config * px4_ctbr_mpc_acados_get_sim_config(px4_ctbr_mpc_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT sim_in * px4_ctbr_mpc_acados_get_sim_in(px4_ctbr_mpc_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT sim_out * px4_ctbr_mpc_acados_get_sim_out(px4_ctbr_mpc_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT void * px4_ctbr_mpc_acados_get_sim_dims(px4_ctbr_mpc_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT sim_opts * px4_ctbr_mpc_acados_get_sim_opts(px4_ctbr_mpc_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT sim_solver * px4_ctbr_mpc_acados_get_sim_solver(px4_ctbr_mpc_sim_solver_capsule *capsule);
ACADOS_SYMBOL_EXPORT void * px4_ctbr_mpc_acados_get_sim_mem(px4_ctbr_mpc_sim_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT px4_ctbr_mpc_sim_solver_capsule * px4_ctbr_mpc_acados_sim_solver_create_capsule(void);
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_sim_solver_free_capsule(px4_ctbr_mpc_sim_solver_capsule *capsule);

#ifdef __cplusplus
}
#endif

#endif  // ACADOS_SIM_px4_ctbr_mpc_H_
