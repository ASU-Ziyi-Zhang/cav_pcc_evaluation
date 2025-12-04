//
// File: factoryConstruct.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "factoryConstruct.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : int nVarMax
//                int mConstrMax
//                int mIneq
//                const int x0_size[2]
//                i_struct_T &obj
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace TrialState {
void factoryConstruct(int nVarMax, int mConstrMax, int mIneq,
                      const int x0_size[2], i_struct_T &obj)
{
  obj.nVarMax = nVarMax;
  obj.mNonlinIneq = 0;
  obj.mNonlinEq = 0;
  obj.mIneq = mIneq;
  obj.mEq = 0;
  obj.iNonIneq0 = mIneq + 1;
  obj.iNonEq0 = 1;
  obj.sqpFval = 0.0;
  obj.sqpFval_old = 0.0;
  obj.xstarsqp.size[0] = 1;
  obj.xstarsqp.size[1] = x0_size[1];
  obj.xstarsqp_old.size[0] = 1;
  obj.xstarsqp_old.size[1] = x0_size[1];
  obj.cIneq.size[0] = mIneq;
  obj.cIneq_old.size[0] = mIneq;
  obj.grad.size[0] = nVarMax;
  obj.grad_old.size[0] = nVarMax;
  obj.FunctionEvaluations = 0;
  obj.sqpIterations = 0;
  obj.sqpExitFlag = 0;
  obj.lambdasqp.size[0] = mConstrMax;
  if (mConstrMax - 1 >= 0) {
    std::memset(&obj.lambdasqp.data[0], 0,
                static_cast<unsigned int>(mConstrMax) * sizeof(double));
  }
  obj.lambdaStopTest.size[0] = mConstrMax;
  obj.lambdaStopTestPrev.size[0] = mConstrMax;
  obj.steplength = 1.0;
  obj.delta_x.size[0] = nVarMax;
  if (nVarMax - 1 >= 0) {
    std::memset(&obj.delta_x.data[0], 0,
                static_cast<unsigned int>(nVarMax) * sizeof(double));
  }
  obj.socDirection.size[0] = nVarMax;
  obj.workingset_old.size[0] = mConstrMax;
  obj.gradLag.size[0] = nVarMax;
  obj.delta_gradLag.size[0] = nVarMax;
  obj.xstar.size[0] = nVarMax;
  obj.fstar = 0.0;
  obj.firstorderopt = 0.0;
  obj.lambda.size[0] = mConstrMax;
  if (mConstrMax - 1 >= 0) {
    std::memset(&obj.lambda.data[0], 0,
                static_cast<unsigned int>(mConstrMax) * sizeof(double));
  }
  obj.state = 0;
  obj.maxConstr = 0.0;
  obj.iterations = 0;
  obj.searchDir.size[0] = nVarMax;
}

} // namespace TrialState
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for factoryConstruct.cpp
//
// [EOF]
//
