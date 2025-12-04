/*
 *  rtmodel.h:
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

#ifndef rtmodel_h_
#define rtmodel_h_
#include "longitudinal_mpc.h"
#define MODEL_CLASSNAME                longitudinal_mpc
#define MODEL_STEPNAME                 step
#define GRTINTERFACE                   0

/*
 * ROOT_IO_FORMAT: 0 (Individual arguments)
 * ROOT_IO_FORMAT: 1 (Structure reference)
 * ROOT_IO_FORMAT: 2 (Part of model data structure)
 */
#define ROOT_IO_FORMAT                 2

/* Macros generated for backwards compatibility  */
#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((void*) 0)
#endif
#endif                                 /* rtmodel_h_ */
