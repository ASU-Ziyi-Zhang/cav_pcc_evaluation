//
// File: computeComplError.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef COMPUTECOMPLERROR_H
#define COMPUTECOMPLERROR_H

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
double computeComplError(int fscales_lineq_constraint_size,
                         const double xCurrent_data[], int mIneq,
                         const double cIneq_data[], const int finiteLB_data[],
                         int mLB, const double lb_data[],
                         const int finiteUB_data[], int mUB,
                         const double ub_data[], const double lambda_data[],
                         int iL0);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for computeComplError.h
//
// [EOF]
//
