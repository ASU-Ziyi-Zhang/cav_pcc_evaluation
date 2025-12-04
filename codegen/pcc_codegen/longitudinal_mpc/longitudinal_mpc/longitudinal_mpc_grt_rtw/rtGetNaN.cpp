/*
 * rtGetNaN.cpp
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

#include "rtwtypes.h"

extern "C"
{

#include "rtGetNaN.h"

}

extern "C"
{
  /* Return rtNaN needed by the generated code. */
  real_T rtGetNaN(void)
  {
    return rtNaN;
  }

  /* Return rtNaNF needed by the generated code. */
  real32_T rtGetNaNF(void)
  {
    return rtNaNF;
  }
}
