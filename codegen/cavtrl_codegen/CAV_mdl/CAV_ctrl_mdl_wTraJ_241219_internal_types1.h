//
// File: CAV_ctrl_mdl_wTraJ_241219_internal_types1.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef CAV_CTRL_MDL_WTRAJ_241219_INTERNAL_TYPES1_H
#define CAV_CTRL_MDL_WTRAJ_241219_INTERNAL_TYPES1_H

// Include Files
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types2.h"
#include "rtwtypes.h"
#include "coder_bounded_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
struct d_struct_T {
  coder::bounded_array<double, 25U, 2U> si_arr;
  double t0s;
  double tf0s;
  double vfs;
  double v0s;
  c_struct_T ViolInfo;
};

#endif
//
// File trailer for CAV_ctrl_mdl_wTraJ_241219_internal_types1.h
//
// [EOF]
//
