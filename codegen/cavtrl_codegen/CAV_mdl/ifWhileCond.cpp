//
// File: ifWhileCond.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "ifWhileCond.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : const boolean_T x_data[]
//                const int x_size[2]
// Return Type  : boolean_T
//
namespace coder {
namespace internal {
boolean_T ifWhileCond(const boolean_T x_data[], const int x_size[2])
{
  boolean_T y;
  y = (x_size[1] != 0);
  if (y) {
    int k;
    boolean_T exitg1;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= x_size[1] - 1)) {
      if (!x_data[k]) {
        y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  return y;
}

} // namespace internal
} // namespace coder

//
// File trailer for ifWhileCond.cpp
//
// [EOF]
//
