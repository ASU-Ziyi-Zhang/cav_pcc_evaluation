//
// File: xgemm.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xgemm.h"
#include "rt_nonfinite.h"
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : int m
//                int n
//                int k
//                const double A_data[]
//                int lda
//                const double B_data[]
//                int ib0
//                int ldb
//                double C_data[]
//                int ldc
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void xgemm(int m, int n, int k, const double A_data[], int lda,
           const double B_data[], int ib0, int ldb, double C_data[], int ldc)
{
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    br = ib0;
    lastColC = ldc * (n - 1);
    for (int cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        std::memset(&C_data[i + -1], 0,
                    static_cast<unsigned int>((i1 - i) + 1) * sizeof(double));
      }
    }
    for (int cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      int ar;
      ar = -1;
      i = br + k;
      for (int ib{br}; ib < i; ib++) {
        int i2;
        int scalarLB;
        int vectorUB;
        i1 = cr + 1;
        i2 = cr + m;
        scalarLB = ((((i2 - cr) / 2) << 1) + cr) + 1;
        vectorUB = scalarLB - 2;
        for (int ic{i1}; ic <= vectorUB; ic += 2) {
          __m128d r;
          r = _mm_loadu_pd(&C_data[ic - 1]);
          _mm_storeu_pd(
              &C_data[ic - 1],
              _mm_add_pd(r, _mm_mul_pd(_mm_set1_pd(B_data[ib - 1]),
                                       _mm_loadu_pd(&A_data[(ar + ic) - cr]))));
        }
        for (int ic{scalarLB}; ic <= i2; ic++) {
          C_data[ic - 1] += B_data[ib - 1] * A_data[(ar + ic) - cr];
        }
        ar += lda;
      }
      br += ldb;
    }
  }
}

//
// Arguments    : int m
//                int n
//                int k
//                const double A_data[]
//                int ia0
//                int lda
//                const double B_data[]
//                int ldb
//                double C_data[]
//                int ldc
// Return Type  : void
//
void xgemm(int m, int n, int k, const double A_data[], int ia0, int lda,
           const double B_data[], int ldb, double C_data[], int ldc)
{
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    lastColC = ldc * (n - 1);
    for (int cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        std::memset(&C_data[i + -1], 0,
                    static_cast<unsigned int>((i1 - i) + 1) * sizeof(double));
      }
    }
    br = -1;
    for (int cr{0}; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      int ar;
      ar = ia0;
      i = cr + 1;
      i1 = cr + m;
      for (int ic{i}; ic <= i1; ic++) {
        double temp;
        temp = 0.0;
        for (int w{0}; w < k; w++) {
          temp += A_data[(w + ar) - 1] * B_data[(w + br) + 1];
        }
        C_data[ic - 1] += temp;
        ar += lda;
      }
      br += ldb;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xgemm.cpp
//
// [EOF]
//
