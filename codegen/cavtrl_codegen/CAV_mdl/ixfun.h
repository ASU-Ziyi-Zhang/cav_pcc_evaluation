//
// File: ixfun.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef IXFUN_H
#define IXFUN_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void expand_max(const double a_data[], const int a_size[2],
                const double b_data[], const int b_size[2], double c_data[],
                int c_size[2]);

void expand_min(const double a_data[], const int a_size[2],
                const double b_data[], const int b_size[2], double c_data[],
                int c_size[2]);

} // namespace internal
} // namespace coder

#endif
//
// File trailer for ixfun.h
//
// [EOF]
//
