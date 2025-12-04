//
// File: computePrimalFeasError.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "computePrimalFeasError.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double x_data[]
//                int mLinIneq
//                const double cIneq_data[]
//                const int finiteLB_data[]
//                int mLB
//                const double lb_data[]
//                const int finiteUB_data[]
//                int mUB
//                const double ub_data[]
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
double computePrimalFeasError(const double x_data[], int mLinIneq,
                              const double cIneq_data[],
                              const int finiteLB_data[], int mLB,
                              const double lb_data[], const int finiteUB_data[],
                              int mUB, const double ub_data[])
{
  double feasError;
  int i;
  feasError = 0.0;
  i = static_cast<unsigned char>(mLinIneq);
  for (int idx{0}; idx < i; idx++) {
    feasError = std::fmax(feasError, cIneq_data[idx]);
  }
  i = static_cast<unsigned char>(mLB);
  for (int idx{0}; idx < i; idx++) {
    feasError = std::fmax(feasError, lb_data[finiteLB_data[idx] - 1] -
                                         x_data[finiteLB_data[idx] - 1]);
  }
  i = static_cast<unsigned char>(mUB);
  for (int idx{0}; idx < i; idx++) {
    feasError = std::fmax(feasError, x_data[finiteUB_data[idx] - 1] -
                                         ub_data[finiteUB_data[idx] - 1]);
  }
  return feasError;
}

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computePrimalFeasError.cpp
//
// [EOF]
//
