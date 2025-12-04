//
// File: computeComplError.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "computeComplError.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : int fscales_lineq_constraint_size
//                const double xCurrent_data[]
//                int mIneq
//                const double cIneq_data[]
//                const int finiteLB_data[]
//                int mLB
//                const double lb_data[]
//                const int finiteUB_data[]
//                int mUB
//                const double ub_data[]
//                const double lambda_data[]
//                int iL0
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
double computeComplError(int fscales_lineq_constraint_size,
                         const double xCurrent_data[], int mIneq,
                         const double cIneq_data[], const int finiteLB_data[],
                         int mLB, const double lb_data[],
                         const int finiteUB_data[], int mUB,
                         const double ub_data[], const double lambda_data[],
                         int iL0)
{
  double nlpComplError;
  nlpComplError = 0.0;
  if ((mIneq + mLB) + mUB > 0) {
    double lbDelta;
    double lbLambda;
    int i;
    int lbOffset;
    int ubOffset;
    for (int idx{0}; idx < fscales_lineq_constraint_size; idx++) {
      lbDelta = lambda_data[(iL0 + idx) - 1];
      lbLambda = cIneq_data[idx];
      nlpComplError = std::fmax(
          nlpComplError, std::fmin(std::abs(lbLambda * lbDelta),
                                   std::fmin(std::abs(lbLambda), lbDelta)));
    }
    lbOffset = (iL0 + mIneq) - 1;
    ubOffset = lbOffset + mLB;
    i = static_cast<unsigned char>(mLB);
    for (int idx{0}; idx < i; idx++) {
      lbDelta = xCurrent_data[finiteLB_data[idx] - 1] -
                lb_data[finiteLB_data[idx] - 1];
      lbLambda = lambda_data[lbOffset + idx];
      nlpComplError = std::fmax(
          nlpComplError, std::fmin(std::abs(lbDelta * lbLambda),
                                   std::fmin(std::abs(lbDelta), lbLambda)));
    }
    i = static_cast<unsigned char>(mUB);
    for (int idx{0}; idx < i; idx++) {
      lbDelta = ub_data[finiteUB_data[idx] - 1] -
                xCurrent_data[finiteUB_data[idx] - 1];
      lbLambda = lambda_data[ubOffset + idx];
      nlpComplError = std::fmax(
          nlpComplError, std::fmin(std::abs(lbDelta * lbLambda),
                                   std::fmin(std::abs(lbDelta), lbLambda)));
    }
  }
  return nlpComplError;
}

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeComplError.cpp
//
// [EOF]
//
