//
// File: xzgeqp3.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xzgeqp3.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <cstring>

// Function Definitions
//
// Arguments    : double A_data[]
//                const int A_size[2]
//                int m
//                int n
//                int nfxd
//                double tau_data[]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void qrf(double A_data[], const int A_size[2], int m, int n, int nfxd,
         double tau_data[])
{
  double work_data[49];
  int lda;
  int loop_ub;
  lda = A_size[0];
  loop_ub = A_size[1];
  if (loop_ub - 1 >= 0) {
    std::memset(&work_data[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(double));
  }
  loop_ub = static_cast<unsigned char>(nfxd);
  for (int i{0}; i < loop_ub; i++) {
    double atmp;
    double d;
    int ii;
    int mmi;
    ii = i * lda + i;
    mmi = m - i;
    if (i + 1 < m) {
      atmp = A_data[ii];
      d = xzlarfg(mmi, atmp, A_data, ii + 2);
      tau_data[i] = d;
      A_data[ii] = atmp;
    } else {
      d = 0.0;
      tau_data[i] = 0.0;
    }
    if (i + 1 < n) {
      atmp = A_data[ii];
      A_data[ii] = 1.0;
      xzlarf(mmi, (n - i) - 1, ii + 1, d, A_data, (ii + lda) + 1, lda,
             work_data);
      A_data[ii] = atmp;
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzgeqp3.cpp
//
// [EOF]
//
