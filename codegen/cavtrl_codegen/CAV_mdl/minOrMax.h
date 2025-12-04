//
// File: minOrMax.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef MINORMAX_H
#define MINORMAX_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void binary_expand_op_5(double in1_data[], const double in2_data[],
                        const int in2_size[2], const double in3_data[], int in4,
                        int in5, const double in6_data[], const int in6_size[2],
                        int in1_size[2]);

namespace coder {
namespace internal {
void maximum2(const double x_data[], const int x_size[2], const double y_data[],
              const int y_size[2], double ex_data[], int ex_size[2]);

void minimum2(const double x_data[], const int x_size[2], const double y_data[],
              const int y_size[2], double ex_data[], int ex_size[2]);

} // namespace internal
} // namespace coder

#endif
//
// File trailer for minOrMax.h
//
// [EOF]
//
