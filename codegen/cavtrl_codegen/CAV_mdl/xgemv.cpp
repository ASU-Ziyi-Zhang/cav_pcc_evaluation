//
// File: xgemv.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xgemv.h"
#include "rt_nonfinite.h"
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : int m
//                int n
//                const double A_data[]
//                int lda
//                const double x_data[]
//                double y_data[]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void b_xgemv(int m, int n, const double A_data[], int lda,
             const double x_data[], double y_data[])
{
  if ((m != 0) && (n != 0)) {
    int i;
    int iy;
    int scalarLB;
    int vectorUB;
    i = static_cast<unsigned char>(n);
    scalarLB = (static_cast<unsigned char>(n) >> 1) << 1;
    vectorUB = scalarLB - 2;
    for (iy = 0; iy <= vectorUB; iy += 2) {
      __m128d r;
      r = _mm_loadu_pd(&y_data[iy]);
      _mm_storeu_pd(&y_data[iy], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
    }
    for (iy = scalarLB; iy < i; iy++) {
      y_data[iy] = -y_data[iy];
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (vectorUB = 1; lda < 0 ? vectorUB >= i : vectorUB <= i;
         vectorUB += lda) {
      double c;
      c = 0.0;
      scalarLB = (vectorUB + m) - 1;
      for (int ia{vectorUB}; ia <= scalarLB; ia++) {
        c += A_data[ia - 1] * x_data[ia - vectorUB];
      }
      y_data[iy] += c;
      iy++;
    }
  }
}

//
// Arguments    : int m
//                int n
//                const double A_data[]
//                int lda
//                const double x_data[]
//                double y_data[]
// Return Type  : void
//
void c_xgemv(int m, int n, const double A_data[], int lda,
             const double x_data[], double y_data[])
{
  if ((m != 0) && (n != 0)) {
    int i;
    int ix;
    i = static_cast<unsigned char>(m);
    std::memset(&y_data[0], 0, static_cast<unsigned int>(i) * sizeof(double));
    ix = 0;
    i = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int i1;
      i1 = (iac + m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        y_data[i2] += A_data[ia - 1] * x_data[ix];
      }
      ix++;
    }
  }
}

//
// Arguments    : int m
//                int n
//                const double A_data[]
//                int lda
//                const double x_data[]
//                int ix0
//                double y_data[]
// Return Type  : void
//
void xgemv(int m, int n, const double A_data[], int lda, const double x_data[],
           int ix0, double y_data[])
{
  if ((m != 0) && (n != 0)) {
    int i;
    int iy;
    int scalarLB;
    int vectorUB;
    i = static_cast<unsigned char>(n);
    scalarLB = (static_cast<unsigned char>(n) >> 1) << 1;
    vectorUB = scalarLB - 2;
    for (iy = 0; iy <= vectorUB; iy += 2) {
      __m128d r;
      r = _mm_loadu_pd(&y_data[iy]);
      _mm_storeu_pd(&y_data[iy], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
    }
    for (iy = scalarLB; iy < i; iy++) {
      y_data[iy] = -y_data[iy];
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (vectorUB = 1; lda < 0 ? vectorUB >= i : vectorUB <= i;
         vectorUB += lda) {
      double c;
      c = 0.0;
      scalarLB = (vectorUB + m) - 1;
      for (int ia{vectorUB}; ia <= scalarLB; ia++) {
        c += A_data[ia - 1] * x_data[((ix0 + ia) - vectorUB) - 1];
      }
      y_data[iy] += c;
      iy++;
    }
  }
}

//
// Arguments    : int m
//                int n
//                const double A_data[]
//                int lda
//                const double x_data[]
//                double y_data[]
// Return Type  : void
//
void xgemv(int m, int n, const double A_data[], int lda, const double x_data[],
           double y_data[])
{
  if (m != 0) {
    int i;
    int iy;
    int scalarLB;
    int vectorUB;
    i = static_cast<unsigned char>(n);
    scalarLB = (static_cast<unsigned char>(n) >> 1) << 1;
    vectorUB = scalarLB - 2;
    for (iy = 0; iy <= vectorUB; iy += 2) {
      __m128d r;
      r = _mm_loadu_pd(&y_data[iy]);
      _mm_storeu_pd(&y_data[iy], _mm_mul_pd(r, _mm_set1_pd(-1.0)));
    }
    for (iy = scalarLB; iy < i; iy++) {
      y_data[iy] = -y_data[iy];
    }
    iy = 0;
    i = lda * (n - 1) + 1;
    for (vectorUB = 1; lda < 0 ? vectorUB >= i : vectorUB <= i;
         vectorUB += lda) {
      double c;
      c = 0.0;
      scalarLB = (vectorUB + m) - 1;
      for (int ia{vectorUB}; ia <= scalarLB; ia++) {
        c += A_data[ia - 1] * x_data[ia - vectorUB];
      }
      y_data[iy] += c;
      iy++;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xgemv.cpp
//
// [EOF]
//
