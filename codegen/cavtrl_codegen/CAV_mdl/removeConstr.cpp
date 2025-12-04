//
// File: removeConstr.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "removeConstr.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : j_struct_T &obj
//                int idx_global
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void removeConstr(j_struct_T &obj, int idx_global)
{
  int TYPE_tmp;
  TYPE_tmp = obj.Wid.data[idx_global - 1] - 1;
  obj.isActiveConstr
      .data[(obj.isActiveIdx[TYPE_tmp] + obj.Wlocalidx.data[idx_global - 1]) -
            2] = false;
  if (idx_global < obj.nActiveConstr) {
    int i;
    int i1;
    i = obj.nActiveConstr - 1;
    obj.Wid.data[idx_global - 1] = obj.Wid.data[i];
    obj.Wlocalidx.data[idx_global - 1] = obj.Wlocalidx.data[i];
    i1 = static_cast<unsigned char>(obj.nVar);
    for (int idx{0}; idx < i1; idx++) {
      obj.ATwset.data[idx + obj.ldA * (idx_global - 1)] =
          obj.ATwset.data[idx + obj.ldA * i];
    }
    obj.bwset.data[idx_global - 1] = obj.bwset.data[i];
  }
  obj.nActiveConstr--;
  obj.nWConstr[TYPE_tmp]--;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for removeConstr.cpp
//
// [EOF]
//
