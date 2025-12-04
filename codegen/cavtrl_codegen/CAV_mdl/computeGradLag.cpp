//
// File: computeGradLag.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "computeGradLag.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// Arguments    : double workspace_data[]
//                int ldA
//                int nVar
//                const double grad_data[]
//                int mIneq
//                const double AineqTrans_data[]
//                const int finiteFixed_data[]
//                int mFixed
//                const int finiteLB_data[]
//                int mLB
//                const int finiteUB_data[]
//                int mUB
//                const double lambda_data[]
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
void computeGradLag(double workspace_data[], int ldA, int nVar,
                    const double grad_data[], int mIneq,
                    const double AineqTrans_data[],
                    const int finiteFixed_data[], int mFixed,
                    const int finiteLB_data[], int mLB,
                    const int finiteUB_data[], int mUB,
                    const double lambda_data[])
{
  int i;
  int idx;
  int ix;
  i = static_cast<unsigned char>(nVar);
  if (i - 1 >= 0) {
    std::copy(&grad_data[0], &grad_data[i], &workspace_data[0]);
  }
  i = static_cast<unsigned char>(mFixed);
  for (idx = 0; idx < i; idx++) {
    workspace_data[finiteFixed_data[idx] - 1] += lambda_data[idx];
  }
  if ((nVar != 0) && (mIneq != 0)) {
    ix = mFixed;
    i = ldA * (mIneq - 1) + 1;
    for (int iac{1}; ldA < 0 ? iac >= i : iac <= i; iac += ldA) {
      idx = (iac + nVar) - 1;
      for (int ia{iac}; ia <= idx; ia++) {
        int i1;
        i1 = ia - iac;
        workspace_data[i1] += AineqTrans_data[ia - 1] * lambda_data[ix];
      }
      ix++;
    }
  }
  ix = mFixed + mIneq;
  i = static_cast<unsigned char>(mLB);
  for (idx = 0; idx < i; idx++) {
    workspace_data[finiteLB_data[idx] - 1] -= lambda_data[ix + idx];
  }
  if (static_cast<unsigned char>(mLB) - 1 >= 0) {
    ix += static_cast<unsigned char>(mLB);
  }
  i = static_cast<unsigned char>(mUB);
  for (idx = 0; idx < i; idx++) {
    workspace_data[finiteUB_data[idx] - 1] += lambda_data[ix + idx];
  }
}

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeGradLag.cpp
//
// [EOF]
//
