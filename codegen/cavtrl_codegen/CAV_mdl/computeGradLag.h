//
// File: computeGradLag.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef COMPUTEGRADLAG_H
#define COMPUTEGRADLAG_H

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
void computeGradLag(double workspace_data[], int ldA, int nVar,
                    const double grad_data[], int mIneq,
                    const double AineqTrans_data[],
                    const int finiteFixed_data[], int mFixed,
                    const int finiteLB_data[], int mLB,
                    const int finiteUB_data[], int mUB,
                    const double lambda_data[]);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for computeGradLag.h
//
// [EOF]
//
