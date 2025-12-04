//
// File: xpotrf.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xpotrf.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : int n
//                double A_data[]
//                int lda
// Return Type  : int
//
namespace coder {
namespace internal {
namespace lapack {
int xpotrf(int n, double A_data[], int lda)
{
  int info;
  int j;
  boolean_T exitg1;
  info = 0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j <= n - 1)) {
    double c;
    double ssq;
    int idxA1j;
    int idxAjj;
    int k;
    idxA1j = j * lda;
    idxAjj = idxA1j + j;
    ssq = 0.0;
    if (j >= 1) {
      for (k = 0; k < j; k++) {
        c = A_data[idxA1j + k];
        ssq += c * c;
      }
    }
    ssq = A_data[idxAjj] - ssq;
    if (ssq > 0.0) {
      ssq = std::sqrt(ssq);
      A_data[idxAjj] = ssq;
      if (j + 1 < n) {
        int i;
        int ia0;
        int idxAjjp1;
        int nmj;
        nmj = (n - j) - 2;
        ia0 = (idxA1j + lda) + 1;
        idxAjjp1 = idxAjj + lda;
        if ((j != 0) && (nmj + 1 != 0)) {
          idxAjj = idxAjjp1;
          i = ia0 + lda * nmj;
          for (int iac{ia0}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
            c = 0.0;
            k = (iac + j) - 1;
            for (int ia{iac}; ia <= k; ia++) {
              c += A_data[ia - 1] * A_data[(idxA1j + ia) - iac];
            }
            A_data[idxAjj] -= c;
            idxAjj += lda;
          }
        }
        ssq = 1.0 / ssq;
        i = (idxAjjp1 + lda * nmj) + 1;
        for (k = idxAjjp1 + 1; lda < 0 ? k >= i : k <= i; k += lda) {
          A_data[k - 1] *= ssq;
        }
      }
      j++;
    } else {
      A_data[idxAjj] = ssq;
      info = j + 1;
      exitg1 = true;
    }
  }
  return info;
}

} // namespace lapack
} // namespace internal
} // namespace coder

//
// File trailer for xpotrf.cpp
//
// [EOF]
//
