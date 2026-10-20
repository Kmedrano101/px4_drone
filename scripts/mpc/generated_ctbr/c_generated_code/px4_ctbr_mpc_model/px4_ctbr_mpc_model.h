/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef px4_ctbr_mpc_MODEL
#define px4_ctbr_mpc_MODEL

#ifdef __cplusplus
extern "C" {
#endif


/* explicit ODE */

// explicit ODE
int px4_ctbr_mpc_expl_ode_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int px4_ctbr_mpc_expl_ode_fun_work(int *, int *, int *, int *);
const int *px4_ctbr_mpc_expl_ode_fun_sparsity_in(int);
const int *px4_ctbr_mpc_expl_ode_fun_sparsity_out(int);
int px4_ctbr_mpc_expl_ode_fun_n_in(void);
int px4_ctbr_mpc_expl_ode_fun_n_out(void);

// explicit forward VDE
int px4_ctbr_mpc_expl_vde_forw(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int px4_ctbr_mpc_expl_vde_forw_work(int *, int *, int *, int *);
const int *px4_ctbr_mpc_expl_vde_forw_sparsity_in(int);
const int *px4_ctbr_mpc_expl_vde_forw_sparsity_out(int);
int px4_ctbr_mpc_expl_vde_forw_n_in(void);
int px4_ctbr_mpc_expl_vde_forw_n_out(void);

// explicit param-direction forward VDE


// explicit adjoint VDE
int px4_ctbr_mpc_expl_vde_adj(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int px4_ctbr_mpc_expl_vde_adj_work(int *, int *, int *, int *);
const int *px4_ctbr_mpc_expl_vde_adj_sparsity_in(int);
const int *px4_ctbr_mpc_expl_vde_adj_sparsity_out(int);
int px4_ctbr_mpc_expl_vde_adj_n_in(void);
int px4_ctbr_mpc_expl_vde_adj_n_out(void);



#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // px4_ctbr_mpc_MODEL
