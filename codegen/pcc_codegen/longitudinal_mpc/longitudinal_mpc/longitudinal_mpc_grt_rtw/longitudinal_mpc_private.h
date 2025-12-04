/*
 * longitudinal_mpc_private.h
 *
 * Code generation for model "longitudinal_mpc".
 *
 * Model version              : 1.201
 * Simulink Coder version : 24.1 (R2024a) 19-Nov-2023
 * C++ source code generated on : Mon Aug 19 14:47:00 2024
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Execution efficiency
 * Validation result: Passed (8), Warning (1), Error (0)
 */

#ifndef longitudinal_mpc_private_h_
#define longitudinal_mpc_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "longitudinal_mpc_types.h"
#include "longitudinal_mpc.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"

extern real_T rt_powd_snf(real_T u0, real_T u1);
extern real_T rt_hypotd_snf(real_T u0, real_T u1);
extern int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);

#endif                                 /* longitudinal_mpc_private_h_ */
