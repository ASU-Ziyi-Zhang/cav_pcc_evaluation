//
// File: round.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "round.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : double x_data[]
//                const int x_size[2]
// Return Type  : void
//
namespace coder {
void b_round(double x_data[], const int x_size[2])
{
  int i;
  i = x_size[1];
  for (int k{0}; k < i; k++) {
    x_data[k] = std::round(x_data[k]);
  }
}

} // namespace coder

//
// File trailer for round.cpp
//
// [EOF]
//
