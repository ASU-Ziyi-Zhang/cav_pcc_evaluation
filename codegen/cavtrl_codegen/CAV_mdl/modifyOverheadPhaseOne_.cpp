//
// File: modifyOverheadPhaseOne_.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "modifyOverheadPhaseOne_.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : j_struct_T &obj
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void modifyOverheadPhaseOne_(j_struct_T &obj)
{
  int i;
  int idxStartIneq;
  i = static_cast<unsigned char>(obj.sizes[0]);
  for (int idx{0}; idx < i; idx++) {
    obj.ATwset.data[(obj.nVar + obj.ldA * idx) - 1] = 0.0;
  }
  i = static_cast<unsigned char>(obj.sizes[2]);
  for (int idx{0}; idx < i; idx++) {
    obj.Aineq.data[(obj.nVar + obj.ldA * idx) - 1] = -1.0;
  }
  obj.indexLB.data[obj.sizes[3] - 1] = obj.nVar;
  obj.lb.data[obj.nVar - 1] = 1.0E-5;
  idxStartIneq = obj.isActiveIdx[2];
  i = obj.nActiveConstr;
  for (int idx{idxStartIneq}; idx <= i; idx++) {
    obj.ATwset.data[(obj.nVar + obj.ldA * (idx - 1)) - 1] = -1.0;
  }
  idxStartIneq = obj.isActiveIdx[4] - 1;
  if (obj.nWConstr[4] > 0) {
    i = obj.sizesNormal[4] - 1;
    for (int idx{i}; idx >= 0; idx--) {
      int i1;
      i1 = idxStartIneq + idx;
      obj.isActiveConstr.data[i1] = obj.isActiveConstr.data[i1 - 1];
    }
  } else {
    obj.isActiveConstr.data[(obj.isActiveIdx[4] + obj.sizesNormal[4]) - 1] =
        false;
  }
  obj.isActiveConstr.data[obj.isActiveIdx[4] - 2] = false;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for modifyOverheadPhaseOne_.cpp
//
// [EOF]
//
