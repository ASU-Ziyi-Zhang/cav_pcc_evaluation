//
// File: factoryConstruct2.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "factoryConstruct2.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : int mIneqMax
//                int nVar
//                int nVarMax
//                int mConstrMax
//                j_struct_T &obj
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void factoryConstruct(int mIneqMax, int nVar, int nVarMax, int mConstrMax,
                      j_struct_T &obj)
{
  obj.mConstr = 0;
  obj.mConstrOrig = 0;
  obj.mConstrMax = mConstrMax;
  obj.nVar = nVar;
  obj.nVarOrig = nVar;
  obj.nVarMax = nVarMax;
  obj.ldA = nVarMax;
  obj.Aineq.size[0] = mIneqMax * nVarMax;
  obj.bineq.size[0] = mIneqMax;
  obj.Aeq.size[0] = 0;
  obj.lb.size[0] = nVarMax;
  obj.ub.size[0] = nVarMax;
  obj.indexLB.size[0] = nVarMax;
  obj.indexUB.size[0] = nVarMax;
  obj.indexFixed.size[0] = nVarMax;
  obj.mEqRemoved = 0;
  obj.ATwset.size[0] = nVarMax * mConstrMax;
  obj.bwset.size[0] = mConstrMax;
  obj.nActiveConstr = 0;
  obj.maxConstrWorkspace.size[0] = mConstrMax;
  for (int i{0}; i < 5; i++) {
    obj.sizes[i] = 0;
    obj.sizesNormal[i] = 0;
    obj.sizesPhaseOne[i] = 0;
    obj.sizesRegularized[i] = 0;
    obj.sizesRegPhaseOne[i] = 0;
  }
  for (int i{0}; i < 6; i++) {
    obj.isActiveIdx[i] = 0;
    obj.isActiveIdxNormal[i] = 0;
    obj.isActiveIdxPhaseOne[i] = 0;
    obj.isActiveIdxRegularized[i] = 0;
    obj.isActiveIdxRegPhaseOne[i] = 0;
  }
  obj.isActiveConstr.size[0] = mConstrMax;
  obj.Wid.size[0] = mConstrMax;
  obj.Wlocalidx.size[0] = mConstrMax;
  for (int i{0}; i < 5; i++) {
    obj.nWConstr[i] = 0;
  }
  obj.probType = 3;
  obj.SLACK0 = 1.0E-5;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for factoryConstruct2.cpp
//
// [EOF]
//
