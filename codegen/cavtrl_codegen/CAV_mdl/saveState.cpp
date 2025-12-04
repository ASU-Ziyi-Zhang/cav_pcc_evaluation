//
// File: saveState.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "saveState.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// Arguments    : i_struct_T &obj
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace TrialState {
void saveState(i_struct_T &obj)
{
  double y_data[49];
  int k;
  int nVar;
  obj.sqpFval_old = obj.sqpFval;
  nVar = obj.xstarsqp.size[1] - 1;
  for (k = 0; k <= nVar; k++) {
    obj.xstarsqp_old.data[k] = obj.xstarsqp.data[k];
    obj.grad_old.data[k] = obj.grad.data[k];
  }
  k = obj.cIneq_old.size[0];
  nVar = obj.cIneq_old.size[0];
  if (nVar - 1 >= 0) {
    std::copy(&obj.cIneq_old.data[0], &obj.cIneq_old.data[nVar], &y_data[0]);
  }
  nVar = static_cast<unsigned char>(obj.mIneq);
  if (nVar - 1 >= 0) {
    std::copy(&obj.cIneq.data[0], &obj.cIneq.data[nVar], &y_data[0]);
  }
  if (k - 1 >= 0) {
    std::copy(&y_data[0], &y_data[k], &obj.cIneq_old.data[0]);
  }
}

} // namespace TrialState
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for saveState.cpp
//
// [EOF]
//
