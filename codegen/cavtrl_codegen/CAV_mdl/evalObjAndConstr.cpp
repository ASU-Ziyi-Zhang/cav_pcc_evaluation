//
// File: evalObjAndConstr.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "evalObjAndConstr.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types1.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types2.h"
#include "RefTrjGnrtr_240503.h"
#include "anonymous_function.h"
#include "rt_nonfinite.h"
#include "stickyStruct.h"
#include "coder_bounded_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const ::coder::internal::i_stickyStruct &obj
//                const double x_data[]
//                const int x_size[2]
//                int &status
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace ObjNonlinEvaluator {
double evalObjAndConstr(const ::coder::internal::i_stickyStruct &obj,
                        const double x_data[], const int x_size[2], int &status)
{
  double fval;
  boolean_T b;
  fval = OptEntTimeSearch_v2_anonFcn1(
      obj.next.next.next.next.next.next.next.next.value.workspace.si_arr.data,
      obj.next.next.next.next.next.next.next.next.value.workspace.si_arr.size,
      obj.next.next.next.next.next.next.next.next.value.workspace.t0s,
      obj.next.next.next.next.next.next.next.next.value.workspace.tf0s,
      obj.next.next.next.next.next.next.next.next.value.workspace.vfs,
      obj.next.next.next.next.next.next.next.next.value.workspace.v0s,
      obj.next.next.next.next.next.next.next.next.value.workspace.ViolInfo
          .idxTLStp.data,
      obj.next.next.next.next.next.next.next.next.value.workspace.ViolInfo
          .idxTLStp.size,
      obj.next.next.next.next.next.next.next.next.value.workspace.ViolInfo.tint
          .data,
      obj.next.next.next.next.next.next.next.next.value.workspace.ViolInfo.tint
          .size,
      x_data, x_size);
  status = 1;
  b = std::isnan(fval);
  if (std::isinf(fval) || b) {
    if (b) {
      status = -3;
    } else if (fval < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }
  if (status == 1) {
    status = 1;
  }
  return fval;
}

} // namespace ObjNonlinEvaluator
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for evalObjAndConstr.cpp
//
// [EOF]
//
