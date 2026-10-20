/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef ACADOS_SOLVER_px4_ctbr_mpc_H_
#define ACADOS_SOLVER_px4_ctbr_mpc_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define PX4_CTBR_MPC_NX     10
#define PX4_CTBR_MPC_NZ     0
#define PX4_CTBR_MPC_NU     4
#define PX4_CTBR_MPC_NP     0
#define PX4_CTBR_MPC_NP_GLOBAL     0
#define PX4_CTBR_MPC_NBX    4
#define PX4_CTBR_MPC_NBX0   10
#define PX4_CTBR_MPC_NBU    4
#define PX4_CTBR_MPC_NSBX   4
#define PX4_CTBR_MPC_NSBU   0
#define PX4_CTBR_MPC_NSH    0
#define PX4_CTBR_MPC_NSH0   0
#define PX4_CTBR_MPC_NSG    0
#define PX4_CTBR_MPC_NSPHI  0
#define PX4_CTBR_MPC_NSHN   0
#define PX4_CTBR_MPC_NSGN   0
#define PX4_CTBR_MPC_NSPHIN 0
#define PX4_CTBR_MPC_NSPHI0 0
#define PX4_CTBR_MPC_NSBXN  4
#define PX4_CTBR_MPC_NS     4
#define PX4_CTBR_MPC_NS0    0
#define PX4_CTBR_MPC_NSN    4
#define PX4_CTBR_MPC_NG     0
#define PX4_CTBR_MPC_NBXN   4
#define PX4_CTBR_MPC_NGN    0
#define PX4_CTBR_MPC_NY0    14
#define PX4_CTBR_MPC_NY     14
#define PX4_CTBR_MPC_NYN    10
#define PX4_CTBR_MPC_N      20
#define PX4_CTBR_MPC_NH     0
#define PX4_CTBR_MPC_NHN    0
#define PX4_CTBR_MPC_NH0    0
#define PX4_CTBR_MPC_NPHI0  0
#define PX4_CTBR_MPC_NPHI   0
#define PX4_CTBR_MPC_NPHIN  0
#define PX4_CTBR_MPC_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct px4_ctbr_mpc_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *expl_vde_forw;
    external_function_external_param_casadi *expl_vde_forw_p;
    external_function_external_param_casadi *expl_ode_fun;
    external_function_external_param_casadi *expl_vde_adj;




    // cost






    // constraints







} px4_ctbr_mpc_solver_capsule;

ACADOS_SYMBOL_EXPORT px4_ctbr_mpc_solver_capsule * px4_ctbr_mpc_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_free_capsule(px4_ctbr_mpc_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_create(px4_ctbr_mpc_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_reset(px4_ctbr_mpc_solver_capsule* capsule, int reset_qp_solver_mem, int reset_numerical_values, int reset_solver_options, int reset_x_to_x0_bar);

/**
 * Generic version of px4_ctbr_mpc_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_create_with_discretization(px4_ctbr_mpc_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_update_time_steps(px4_ctbr_mpc_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_update_qp_solver_cond_N(px4_ctbr_mpc_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_update_params(px4_ctbr_mpc_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_update_params_sparse(px4_ctbr_mpc_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_set_p_global_and_precompute_dependencies(px4_ctbr_mpc_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_solve(px4_ctbr_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_setup_qp_matrices_and_factorize(px4_ctbr_mpc_solver_capsule* capsule);



ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_free(px4_ctbr_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void px4_ctbr_mpc_acados_print_stats(px4_ctbr_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int px4_ctbr_mpc_acados_custom_update(px4_ctbr_mpc_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT ocp_nlp_in *px4_ctbr_mpc_acados_get_nlp_in(px4_ctbr_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *px4_ctbr_mpc_acados_get_nlp_out(px4_ctbr_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *px4_ctbr_mpc_acados_get_sens_out(px4_ctbr_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *px4_ctbr_mpc_acados_get_nlp_solver(px4_ctbr_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *px4_ctbr_mpc_acados_get_nlp_config(px4_ctbr_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *px4_ctbr_mpc_acados_get_nlp_opts(px4_ctbr_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *px4_ctbr_mpc_acados_get_nlp_dims(px4_ctbr_mpc_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *px4_ctbr_mpc_acados_get_nlp_plan(px4_ctbr_mpc_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_px4_ctbr_mpc_H_
