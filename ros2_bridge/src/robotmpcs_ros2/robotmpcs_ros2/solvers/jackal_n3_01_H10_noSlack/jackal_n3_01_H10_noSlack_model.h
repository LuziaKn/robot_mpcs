

#ifndef JACKAL_N3_01_H10_NOSLACK_MODEL_H
#include "include/jackal_n3_01_H10_noSlack.h"
#define JACKAL_N3_01_H10_NOSLACK_MODEL_H
/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real jackal_n3_01_H10_noSlack_float
#endif

#ifndef casadi_int
#define casadi_int solver_int32_default
#endif

int jackal_n3_01_H10_noSlack_objective_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
const casadi_int* jackal_n3_01_H10_noSlack_objective_0_sparsity_out(casadi_int i);
int jackal_n3_01_H10_noSlack_objective_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int jackal_n3_01_H10_noSlack_inequalities_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
const casadi_int* jackal_n3_01_H10_noSlack_inequalities_0_sparsity_out(casadi_int i);
int jackal_n3_01_H10_noSlack_inequalities_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int jackal_n3_01_H10_noSlack_cdyn_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
const casadi_int* jackal_n3_01_H10_noSlack_cdyn_0_sparsity_out(casadi_int i);
int jackal_n3_01_H10_noSlack_cdyn_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int jackal_n3_01_H10_noSlack_cdyn_0rd_0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
const casadi_int* jackal_n3_01_H10_noSlack_cdyn_0rd_0_sparsity_out(casadi_int i);
int jackal_n3_01_H10_noSlack_cdyn_0rd_0_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int jackal_n3_01_H10_noSlack_objective_1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
const casadi_int* jackal_n3_01_H10_noSlack_objective_1_sparsity_out(casadi_int i);
int jackal_n3_01_H10_noSlack_objective_1_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int jackal_n3_01_H10_noSlack_inequalities_1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
const casadi_int* jackal_n3_01_H10_noSlack_inequalities_1_sparsity_out(casadi_int i);
int jackal_n3_01_H10_noSlack_inequalities_1_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#ifdef __cplusplus
} /* extern "C" */
#endif
#endif