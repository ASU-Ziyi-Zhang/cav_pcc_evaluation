//
// File: xnrm2.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef XNRM2_H
#define XNRM2_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
double b_xnrm2(int n, const double x[3]);

double c_xnrm2(int n, const double x_data[]);

double xnrm2(int n, const double x_data[]);

double xnrm2(int n, const double x_data[], int ix0);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xnrm2.h
//
// [EOF]
//
