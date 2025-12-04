//
// File: xgerc.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xgerc.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : int m
//                int n
//                double alpha1
//                int ix0
//                const double y_data[]
//                double A_data[]
//                int ia0
//                int lda
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void xgerc(int m, int n, double alpha1, int ix0, const double y_data[],
           double A_data[], int ia0, int lda)
{
  if (!(alpha1 == 0.0)) {
    int jA;
    jA = ia0;
    for (int j{0}; j < n; j++) {
      double temp;
      temp = y_data[j];
      if (temp != 0.0) {
        int i;
        temp *= alpha1;
        i = m + jA;
        for (int ijA{jA}; ijA < i; ijA++) {
          A_data[ijA - 1] += A_data[((ix0 + ijA) - jA) - 1] * temp;
        }
      }
      jA += lda;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xgerc.cpp
//
// [EOF]
//
