//
// File: xgemm.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef XGEMM_H
#define XGEMM_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void xgemm(int m, int n, int k, const double A_data[], int lda,
           const double B_data[], int ib0, int ldb, double C_data[], int ldc);

void xgemm(int m, int n, int k, const double A_data[], int ia0, int lda,
           const double B_data[], int ldb, double C_data[], int ldc);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xgemm.h
//
// [EOF]
//
