#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# Licensed under the 2-Clause BSD License.

cimport acados_solver_common

cdef extern from "acados_solver_px4_ctbr_mpc.h":
    ctypedef struct nlp_solver_capsule "px4_ctbr_mpc_solver_capsule":
        pass

    nlp_solver_capsule * acados_create_capsule "px4_ctbr_mpc_acados_create_capsule"()
    int acados_free_capsule "px4_ctbr_mpc_acados_free_capsule"(nlp_solver_capsule *capsule)

    int acados_create "px4_ctbr_mpc_acados_create"(nlp_solver_capsule * capsule)

    int acados_create_with_discretization "px4_ctbr_mpc_acados_create_with_discretization"(nlp_solver_capsule * capsule, int n_time_steps, double* new_time_steps)
    int acados_update_time_steps "px4_ctbr_mpc_acados_update_time_steps"(nlp_solver_capsule * capsule, int N, double* new_time_steps)
    int acados_update_qp_solver_cond_N "px4_ctbr_mpc_acados_update_qp_solver_cond_N"(nlp_solver_capsule * capsule, int qp_solver_cond_N)

    int acados_update_params "px4_ctbr_mpc_acados_update_params"(nlp_solver_capsule * capsule, int stage, double *value, int np_)
    int acados_update_params_sparse "px4_ctbr_mpc_acados_update_params_sparse"(nlp_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
    int acados_set_p_global_and_precompute_dependencies "px4_ctbr_mpc_acados_set_p_global_and_precompute_dependencies"(nlp_solver_capsule * capsule, double *value, int data_len)
    int acados_solve "px4_ctbr_mpc_acados_solve"(nlp_solver_capsule * capsule)
    int acados_reset "px4_ctbr_mpc_acados_reset"(nlp_solver_capsule * capsule, int reset_qp_solver_mem, int reset_numerical_values, int reset_solver_options, int reset_x_to_x0_bar)
    int acados_free "px4_ctbr_mpc_acados_free"(nlp_solver_capsule * capsule)
    void acados_print_stats "px4_ctbr_mpc_acados_print_stats"(nlp_solver_capsule * capsule)

    int acados_custom_update "px4_ctbr_mpc_acados_custom_update"(nlp_solver_capsule* capsule, double * data, int data_len)

    acados_solver_common.ocp_nlp_in *acados_get_nlp_in "px4_ctbr_mpc_acados_get_nlp_in"(nlp_solver_capsule * capsule)
    acados_solver_common.ocp_nlp_out *acados_get_nlp_out "px4_ctbr_mpc_acados_get_nlp_out"(nlp_solver_capsule * capsule)
    acados_solver_common.ocp_nlp_out *acados_get_sens_out "px4_ctbr_mpc_acados_get_sens_out"(nlp_solver_capsule * capsule)
    acados_solver_common.ocp_nlp_solver *acados_get_nlp_solver "px4_ctbr_mpc_acados_get_nlp_solver"(nlp_solver_capsule * capsule)
    acados_solver_common.ocp_nlp_config *acados_get_nlp_config "px4_ctbr_mpc_acados_get_nlp_config"(nlp_solver_capsule * capsule)
    void *acados_get_nlp_opts "px4_ctbr_mpc_acados_get_nlp_opts"(nlp_solver_capsule * capsule)
    acados_solver_common.ocp_nlp_dims *acados_get_nlp_dims "px4_ctbr_mpc_acados_get_nlp_dims"(nlp_solver_capsule * capsule)
    acados_solver_common.ocp_nlp_plan *acados_get_nlp_plan "px4_ctbr_mpc_acados_get_nlp_plan"(nlp_solver_capsule * capsule)
