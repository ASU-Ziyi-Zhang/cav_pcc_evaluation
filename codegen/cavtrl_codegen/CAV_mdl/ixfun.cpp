//
// File: ixfun.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "ixfun.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double a_data[]
//                const int a_size[2]
//                const double b_data[]
//                const int b_size[2]
//                double c_data[]
//                int c_size[2]
// Return Type  : void
//
namespace coder {
namespace internal {
void expand_max(const double a_data[], const int a_size[2],
                const double b_data[], const int b_size[2], double c_data[],
                int c_size[2])
{
  signed char csz_idx_1_tmp;
  if (b_size[1] == 1) {
    csz_idx_1_tmp = static_cast<signed char>(a_size[1]);
  } else if (a_size[1] == 1) {
    csz_idx_1_tmp = static_cast<signed char>(b_size[1]);
  } else if (a_size[1] <= b_size[1]) {
    csz_idx_1_tmp = static_cast<signed char>(a_size[1]);
  } else {
    csz_idx_1_tmp = static_cast<signed char>(b_size[1]);
  }
  c_size[0] = 1;
  c_size[1] = csz_idx_1_tmp;
  if (csz_idx_1_tmp != 0) {
    int i;
    boolean_T b;
    boolean_T b1;
    b = (a_size[1] != 1);
    b1 = (b_size[1] != 1);
    i = csz_idx_1_tmp - 1;
    for (int k{0}; k <= i; k++) {
      c_data[k] = std::fmax(a_data[b * k], b_data[b1 * k]);
    }
  }
}

//
// Arguments    : const double a_data[]
//                const int a_size[2]
//                const double b_data[]
//                const int b_size[2]
//                double c_data[]
//                int c_size[2]
// Return Type  : void
//
void expand_min(const double a_data[], const int a_size[2],
                const double b_data[], const int b_size[2], double c_data[],
                int c_size[2])
{
  signed char csz_idx_1_tmp;
  if (b_size[1] == 1) {
    csz_idx_1_tmp = static_cast<signed char>(a_size[1]);
  } else if (a_size[1] == 1) {
    csz_idx_1_tmp = static_cast<signed char>(b_size[1]);
  } else if (a_size[1] <= b_size[1]) {
    csz_idx_1_tmp = static_cast<signed char>(a_size[1]);
  } else {
    csz_idx_1_tmp = static_cast<signed char>(b_size[1]);
  }
  c_size[0] = 1;
  c_size[1] = csz_idx_1_tmp;
  if (csz_idx_1_tmp != 0) {
    int i;
    boolean_T b;
    boolean_T b1;
    b = (a_size[1] != 1);
    b1 = (b_size[1] != 1);
    i = csz_idx_1_tmp - 1;
    for (int k{0}; k <= i; k++) {
      c_data[k] = std::fmin(a_data[b * k], b_data[b1 * k]);
    }
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for ixfun.cpp
//
// [EOF]
//
