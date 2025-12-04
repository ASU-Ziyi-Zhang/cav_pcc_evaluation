//
// File: computePrimalFeasError.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef COMPUTEPRIMALFEASERROR_H
#define COMPUTEPRIMALFEASERROR_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
double computePrimalFeasError(const double x_data[], int mLinIneq,
                              const double cIneq_data[],
                              const int finiteLB_data[], int mLB,
                              const double lb_data[], const int finiteUB_data[],
                              int mUB, const double ub_data[]);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for computePrimalFeasError.h
//
// [EOF]
//
