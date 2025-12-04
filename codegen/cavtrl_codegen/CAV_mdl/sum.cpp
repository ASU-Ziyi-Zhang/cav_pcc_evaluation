//
// File: sum.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "sum.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : const double x_data[]
//                const int x_size[2]
// Return Type  : double
//
namespace coder {
double sum(const double x_data[], const int x_size[2])
{
  double y;
  int vlen;
  vlen = x_size[1];
  if (x_size[1] == 0) {
    y = 0.0;
  } else {
    y = x_data[0];
    for (int k{2}; k <= vlen; k++) {
      y += x_data[k - 1];
    }
  }
  return y;
}

} // namespace coder

//
// File trailer for sum.cpp
//
// [EOF]
//
