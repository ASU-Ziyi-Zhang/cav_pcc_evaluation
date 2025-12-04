//
// File: mrdivide_helper.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "mrdivide_helper.h"
#include "CAV_ctrl_mdl_wTraJ_241219_rtwutil.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                const double B_data[]
//                const int B_size[2]
// Return Type  : double
//
namespace coder {
namespace internal {
double mrdiv(const double A_data[], const int A_size[2], const double B_data[],
             const int B_size[2])
{
  double x_data[25];
  double b_B_data[12];
  double Y;
  if ((A_size[1] == 0) || (B_size[1] == 0)) {
    Y = 0.0;
  } else if (B_size[1] == 1) {
    Y = A_data[0] / B_data[0];
  } else {
    __m128d r;
    double tau_data;
    double wj;
    int knt;
    int loop_ub;
    int rankA;
    int vectorUB;
    int x_size;
    loop_ub = B_size[1];
    x_size = B_size[1];
    std::copy(&B_data[0], &B_data[loop_ub], &x_data[0]);
    knt = A_size[1];
    std::copy(&A_data[0], &A_data[knt], &b_B_data[0]);
    tau_data = 0.0;
    for (int i{0}; i < 1; i++) {
      double atmp;
      atmp = x_data[0];
      tau_data = 0.0;
      wj = blas::c_xnrm2(loop_ub - 1, x_data);
      if (wj != 0.0) {
        double beta1;
        beta1 = rt_hypotd_snf(x_data[0], wj);
        if (x_data[0] >= 0.0) {
          beta1 = -beta1;
        }
        if (std::abs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          do {
            knt++;
            rankA = (((loop_ub - 1) / 2) << 1) + 2;
            vectorUB = rankA - 2;
            for (int k{2}; k <= vectorUB; k += 2) {
              r = _mm_loadu_pd(&x_data[k - 1]);
              _mm_storeu_pd(&x_data[k - 1],
                            _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
            }
            for (int k{rankA}; k <= loop_ub; k++) {
              x_data[k - 1] *= 9.9792015476736E+291;
            }
            beta1 *= 9.9792015476736E+291;
            atmp *= 9.9792015476736E+291;
          } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
          beta1 = rt_hypotd_snf(atmp, blas::c_xnrm2(loop_ub - 1, x_data));
          if (atmp >= 0.0) {
            beta1 = -beta1;
          }
          tau_data = (beta1 - atmp) / beta1;
          wj = 1.0 / (atmp - beta1);
          for (int k{2}; k <= vectorUB; k += 2) {
            r = _mm_loadu_pd(&x_data[k - 1]);
            _mm_storeu_pd(&x_data[k - 1], _mm_mul_pd(_mm_set1_pd(wj), r));
          }
          for (int k{rankA}; k <= loop_ub; k++) {
            x_data[k - 1] *= wj;
          }
          for (int k{0}; k < knt; k++) {
            beta1 *= 1.0020841800044864E-292;
          }
          atmp = beta1;
        } else {
          tau_data = (beta1 - x_data[0]) / beta1;
          wj = 1.0 / (x_data[0] - beta1);
          knt = (((loop_ub - 1) / 2) << 1) + 2;
          vectorUB = knt - 2;
          for (int k{2}; k <= vectorUB; k += 2) {
            r = _mm_loadu_pd(&x_data[k - 1]);
            _mm_storeu_pd(&x_data[k - 1], _mm_mul_pd(_mm_set1_pd(wj), r));
          }
          for (int k{knt}; k <= loop_ub; k++) {
            x_data[k - 1] *= wj;
          }
          atmp = beta1;
        }
      }
      x_data[0] = atmp;
    }
    rankA = 0;
    wj = std::abs(x_data[0]);
    if (!(wj <= 2.2204460492503131E-15 * static_cast<double>(B_size[1]) * wj)) {
      rankA = 1;
    }
    Y = 0.0;
    if (tau_data != 0.0) {
      wj = b_B_data[0];
      for (int i{2}; i <= x_size; i++) {
        wj += x_data[i - 1] * b_B_data[i - 1];
      }
      wj *= tau_data;
      if (wj != 0.0) {
        b_B_data[0] -= wj;
        knt = (((B_size[1] - 1) / 2) << 1) + 2;
        vectorUB = knt - 2;
        for (int i{2}; i <= vectorUB; i += 2) {
          __m128d r1;
          r = _mm_loadu_pd(&x_data[i - 1]);
          r1 = _mm_loadu_pd(&b_B_data[i - 1]);
          _mm_storeu_pd(&b_B_data[i - 1],
                        _mm_sub_pd(r1, _mm_mul_pd(r, _mm_set1_pd(wj))));
        }
        for (int i{knt}; i <= x_size; i++) {
          b_B_data[i - 1] -= x_data[i - 1] * wj;
        }
      }
    }
    for (int i{0}; i < rankA; i++) {
      Y = b_B_data[0];
    }
    for (knt = rankA; knt >= 1; knt--) {
      Y /= x_data[0];
    }
  }
  return Y;
}

} // namespace internal
} // namespace coder

//
// File trailer for mrdivide_helper.cpp
//
// [EOF]
//
