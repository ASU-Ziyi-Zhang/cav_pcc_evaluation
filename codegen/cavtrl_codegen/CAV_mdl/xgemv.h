//
// File: xgemv.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef XGEMV_H
#define XGEMV_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void b_xgemv(int m, int n, const double A_data[], int lda,
             const double x_data[], double y_data[]);

void c_xgemv(int m, int n, const double A_data[], int lda,
             const double x_data[], double y_data[]);

void xgemv(int m, int n, const double A_data[], int lda, const double x_data[],
           int ix0, double y_data[]);

void xgemv(int m, int n, const double A_data[], int lda, const double x_data[],
           double y_data[]);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xgemv.h
//
// [EOF]
//
