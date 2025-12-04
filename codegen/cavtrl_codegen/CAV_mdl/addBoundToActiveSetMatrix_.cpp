//
// File: addBoundToActiveSetMatrix_.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "addBoundToActiveSetMatrix_.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : j_struct_T &obj
//                int TYPE
//                int idx_local
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void addBoundToActiveSetMatrix_(j_struct_T &obj, int TYPE, int idx_local)
{
  int colOffset;
  int i;
  int idx_bnd_local;
  obj.nWConstr[TYPE - 1]++;
  obj.isActiveConstr.data[(obj.isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj.nActiveConstr++;
  i = obj.nActiveConstr - 1;
  obj.Wid.data[i] = TYPE;
  obj.Wlocalidx.data[i] = idx_local;
  colOffset = obj.ldA * i - 1;
  if (TYPE == 5) {
    idx_bnd_local = obj.indexUB.data[idx_local - 1];
    obj.bwset.data[i] = obj.ub.data[idx_bnd_local - 1];
  } else {
    idx_bnd_local = obj.indexLB.data[idx_local - 1];
    obj.bwset.data[i] = obj.lb.data[idx_bnd_local - 1];
  }
  if (idx_bnd_local - 2 >= 0) {
    std::memset(&obj.ATwset.data[colOffset + 1], 0,
                static_cast<unsigned int>(
                    ((idx_bnd_local + colOffset) - colOffset) - 1) *
                    sizeof(double));
  }
  obj.ATwset.data[idx_bnd_local + colOffset] =
      2.0 * static_cast<double>(TYPE == 5) - 1.0;
  i = idx_bnd_local + 1;
  idx_bnd_local = obj.nVar;
  if (i <= idx_bnd_local) {
    std::memset(&obj.ATwset.data[i + colOffset], 0,
                static_cast<unsigned int>(
                    (((idx_bnd_local + colOffset) - i) - colOffset) + 1) *
                    sizeof(double));
  }
  switch (obj.probType) {
  case 3:
  case 2:
    break;
  default:
    obj.ATwset.data[obj.nVar + colOffset] = -1.0;
    break;
  }
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for addBoundToActiveSetMatrix_.cpp
//
// [EOF]
//
