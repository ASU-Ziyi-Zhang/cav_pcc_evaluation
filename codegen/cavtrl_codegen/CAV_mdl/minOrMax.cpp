//
// File: minOrMax.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "minOrMax.h"
#include "ixfun.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : double in1_data[]
//                const double in2_data[]
//                const int in2_size[2]
//                const double in3_data[]
//                int in4
//                int in5
//                const double in6_data[]
//                const int in6_size[2]
//                int in1_size[2]
// Return Type  : void
//
void binary_expand_op_5(double in1_data[], const double in2_data[],
                        const int in2_size[2], const double in3_data[], int in4,
                        int in5, const double in6_data[], const int in6_size[2],
                        int in1_size[2])
{
  double b_in3_data[12];
  int in3_size[2];
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in3_size[0] = 1;
  i = (in5 - in4) + 1;
  if (in6_size[1] == 1) {
    loop_ub = i;
  } else {
    loop_ub = in6_size[1];
  }
  in3_size[1] = loop_ub;
  stride_0_1 = (i != 1);
  stride_1_1 = (in6_size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    b_in3_data[i] = in3_data[in4 + i * stride_0_1] - in6_data[i * stride_1_1];
  }
  coder::internal::maximum2(in2_data, in2_size, b_in3_data, in3_size, in1_data,
                            in1_size);
}

//
// Arguments    : const double x_data[]
//                const int x_size[2]
//                const double y_data[]
//                const int y_size[2]
//                double ex_data[]
//                int ex_size[2]
// Return Type  : void
//
namespace coder {
namespace internal {
void maximum2(const double x_data[], const int x_size[2], const double y_data[],
              const int y_size[2], double ex_data[], int ex_size[2])
{
  if (x_size[1] == y_size[1]) {
    int loop_ub;
    ex_size[0] = 1;
    loop_ub = x_size[1];
    ex_size[1] = x_size[1];
    for (int i{0}; i < loop_ub; i++) {
      ex_data[i] = std::fmax(x_data[i], y_data[i]);
    }
  } else {
    expand_max(x_data, x_size, y_data, y_size, ex_data, ex_size);
  }
}

//
// Arguments    : const double x_data[]
//                const int x_size[2]
//                const double y_data[]
//                const int y_size[2]
//                double ex_data[]
//                int ex_size[2]
// Return Type  : void
//
void minimum2(const double x_data[], const int x_size[2], const double y_data[],
              const int y_size[2], double ex_data[], int ex_size[2])
{
  if (x_size[1] == y_size[1]) {
    int loop_ub;
    ex_size[0] = 1;
    loop_ub = x_size[1];
    ex_size[1] = x_size[1];
    for (int i{0}; i < loop_ub; i++) {
      ex_data[i] = std::fmin(x_data[i], y_data[i]);
    }
  } else {
    expand_min(x_data, x_size, y_data, y_size, ex_data, ex_size);
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for minOrMax.cpp
//
// [EOF]
//
