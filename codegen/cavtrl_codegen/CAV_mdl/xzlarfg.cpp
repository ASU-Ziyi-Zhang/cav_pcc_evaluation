//
// File: xzlarfg.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xzlarfg.h"
#include "CAV_ctrl_mdl_wTraJ_241219_rtwutil.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : int n
//                double &alpha1
//                double x_data[]
//                int ix0
// Return Type  : double
//
namespace coder {
namespace internal {
namespace reflapack {
double xzlarfg(int n, double &alpha1, double x_data[], int ix0)
{
  double tau;
  tau = 0.0;
  if (n > 0) {
    double xnorm;
    xnorm = blas::xnrm2(n - 1, x_data, ix0);
    if (xnorm != 0.0) {
      double beta1;
      beta1 = rt_hypotd_snf(alpha1, xnorm);
      if (alpha1 >= 0.0) {
        beta1 = -beta1;
      }
      if (std::abs(beta1) < 1.0020841800044864E-292) {
        __m128d r;
        int i;
        int knt;
        int vectorUB;
        int vectorUB_tmp;
        knt = 0;
        i = (ix0 + n) - 2;
        do {
          knt++;
          vectorUB = ((((i - ix0) + 1) / 2) << 1) + ix0;
          vectorUB_tmp = vectorUB - 2;
          for (int k{ix0}; k <= vectorUB_tmp; k += 2) {
            r = _mm_loadu_pd(&x_data[k - 1]);
            _mm_storeu_pd(&x_data[k - 1],
                          _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
          }
          for (int k{vectorUB}; k <= i; k++) {
            x_data[k - 1] *= 9.9792015476736E+291;
          }
          beta1 *= 9.9792015476736E+291;
          alpha1 *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
        beta1 = rt_hypotd_snf(alpha1, blas::xnrm2(n - 1, x_data, ix0));
        if (alpha1 >= 0.0) {
          beta1 = -beta1;
        }
        tau = (beta1 - alpha1) / beta1;
        xnorm = 1.0 / (alpha1 - beta1);
        for (int k{ix0}; k <= vectorUB_tmp; k += 2) {
          r = _mm_loadu_pd(&x_data[k - 1]);
          _mm_storeu_pd(&x_data[k - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
        }
        for (int k{vectorUB}; k <= i; k++) {
          x_data[k - 1] *= xnorm;
        }
        for (int k{0}; k < knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }
        alpha1 = beta1;
      } else {
        int i;
        int knt;
        int vectorUB;
        tau = (beta1 - alpha1) / beta1;
        xnorm = 1.0 / (alpha1 - beta1);
        i = (ix0 + n) - 2;
        knt = ((((i - ix0) + 1) / 2) << 1) + ix0;
        vectorUB = knt - 2;
        for (int k{ix0}; k <= vectorUB; k += 2) {
          __m128d r;
          r = _mm_loadu_pd(&x_data[k - 1]);
          _mm_storeu_pd(&x_data[k - 1], _mm_mul_pd(_mm_set1_pd(xnorm), r));
        }
        for (int k{knt}; k <= i; k++) {
          x_data[k - 1] *= xnorm;
        }
        alpha1 = beta1;
      }
    }
  }
  return tau;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzlarfg.cpp
//
// [EOF]
//
