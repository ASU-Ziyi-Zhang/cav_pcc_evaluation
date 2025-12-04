//
// File: computeDualFeasError.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "computeDualFeasError.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : int nVar
//                const double gradLag_data[]
//                double &val
// Return Type  : boolean_T
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
boolean_T computeDualFeasError(int nVar, const double gradLag_data[],
                               double &val)
{
  int idx;
  boolean_T exitg1;
  boolean_T gradOK;
  gradOK = true;
  val = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= static_cast<unsigned char>(nVar) - 1)) {
    gradOK =
        ((!std::isinf(gradLag_data[idx])) && (!std::isnan(gradLag_data[idx])));
    if (!gradOK) {
      exitg1 = true;
    } else {
      val = std::fmax(val, std::abs(gradLag_data[idx]));
      idx++;
    }
  }
  return gradOK;
}

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeDualFeasError.cpp
//
// [EOF]
//
