//
// File: BFGSUpdate.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef BFGSUPDATE_H
#define BFGSUPDATE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
boolean_T BFGSUpdate(int nvar, double Bk_data[], const int Bk_size[2],
                     const double sk_data[], double yk_data[],
                     double workspace_data[]);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for BFGSUpdate.h
//
// [EOF]
//
