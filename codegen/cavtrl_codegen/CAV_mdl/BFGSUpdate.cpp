//
// File: BFGSUpdate.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "BFGSUpdate.h"
#include "rt_nonfinite.h"
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : int nvar
//                double Bk_data[]
//                const int Bk_size[2]
//                const double sk_data[]
//                double yk_data[]
//                double workspace_data[]
// Return Type  : boolean_T
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
boolean_T BFGSUpdate(int nvar, double Bk_data[], const int Bk_size[2],
                     const double sk_data[], double yk_data[],
                     double workspace_data[])
{
  __m128d r;
  __m128d r1;
  double curvatureS;
  double dotSY;
  double theta;
  int i;
  int i1;
  int i2;
  int ia;
  int ix;
  int ldBk;
  int vectorUB;
  boolean_T success;
  ldBk = Bk_size[0];
  dotSY = 0.0;
  if (nvar >= 1) {
    i = static_cast<unsigned char>(nvar);
    for (int k{0}; k < i; k++) {
      dotSY += sk_data[k] * yk_data[k];
    }
  }
  if (nvar != 0) {
    i = static_cast<unsigned char>(nvar);
    std::memset(&workspace_data[0], 0,
                static_cast<unsigned int>(i) * sizeof(double));
    ix = 0;
    i = Bk_size[0] * (nvar - 1) + 1;
    for (int k{1}; ldBk < 0 ? k >= i : k <= i; k += ldBk) {
      i1 = (k + nvar) - 1;
      for (ia = k; ia <= i1; ia++) {
        i2 = ia - k;
        workspace_data[i2] += Bk_data[ia - 1] * sk_data[ix];
      }
      ix++;
    }
  }
  curvatureS = 0.0;
  if (nvar >= 1) {
    i = static_cast<unsigned char>(nvar);
    for (int k{0}; k < i; k++) {
      curvatureS += sk_data[k] * workspace_data[k];
    }
  }
  if (dotSY < 0.2 * curvatureS) {
    theta = 0.8 * curvatureS / (curvatureS - dotSY);
    i = static_cast<unsigned char>(nvar);
    ia = (static_cast<unsigned char>(nvar) >> 1) << 1;
    vectorUB = ia - 2;
    for (int k{0}; k <= vectorUB; k += 2) {
      r = _mm_loadu_pd(&yk_data[k]);
      _mm_storeu_pd(&yk_data[k], _mm_mul_pd(_mm_set1_pd(theta), r));
    }
    for (int k{ia}; k < i; k++) {
      yk_data[k] *= theta;
    }
    if ((nvar >= 1) && (!(1.0 - theta == 0.0))) {
      ix = nvar - 1;
      ia = (nvar / 2) << 1;
      vectorUB = ia - 2;
      for (int k{0}; k <= vectorUB; k += 2) {
        r = _mm_loadu_pd(&workspace_data[k]);
        r1 = _mm_loadu_pd(&yk_data[k]);
        _mm_storeu_pd(&yk_data[k],
                      _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(1.0 - theta), r)));
      }
      for (int k{ia}; k <= ix; k++) {
        yk_data[k] += (1.0 - theta) * workspace_data[k];
      }
    }
    dotSY = 0.0;
    if (nvar >= 1) {
      for (int k{0}; k < i; k++) {
        dotSY += sk_data[k] * yk_data[k];
      }
    }
  }
  if ((curvatureS > 2.2204460492503131E-16) &&
      (dotSY > 2.2204460492503131E-16)) {
    success = true;
  } else {
    success = false;
  }
  if (success) {
    curvatureS = -1.0 / curvatureS;
    if (!(curvatureS == 0.0)) {
      ix = 0;
      i = static_cast<unsigned char>(nvar);
      for (int k{0}; k < i; k++) {
        theta = workspace_data[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = ix + 1;
          i2 = nvar + ix;
          ia = ((((i2 - ix) / 2) << 1) + ix) + 1;
          vectorUB = ia - 2;
          for (int ijA{i1}; ijA <= vectorUB; ijA += 2) {
            r = _mm_loadu_pd(&workspace_data[(ijA - ix) - 1]);
            r1 = _mm_loadu_pd(&Bk_data[ijA - 1]);
            _mm_storeu_pd(&Bk_data[ijA - 1],
                          _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(theta))));
          }
          for (int ijA{ia}; ijA <= i2; ijA++) {
            Bk_data[ijA - 1] += workspace_data[(ijA - ix) - 1] * theta;
          }
        }
        ix += ldBk;
      }
    }
    curvatureS = 1.0 / dotSY;
    if (!(curvatureS == 0.0)) {
      ix = 0;
      i = static_cast<unsigned char>(nvar);
      for (int k{0}; k < i; k++) {
        theta = yk_data[k];
        if (theta != 0.0) {
          theta *= curvatureS;
          i1 = ix + 1;
          i2 = nvar + ix;
          ia = ((((i2 - ix) / 2) << 1) + ix) + 1;
          vectorUB = ia - 2;
          for (int ijA{i1}; ijA <= vectorUB; ijA += 2) {
            r = _mm_loadu_pd(&yk_data[(ijA - ix) - 1]);
            r1 = _mm_loadu_pd(&Bk_data[ijA - 1]);
            _mm_storeu_pd(&Bk_data[ijA - 1],
                          _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(theta))));
          }
          for (int ijA{ia}; ijA <= i2; ijA++) {
            Bk_data[ijA - 1] += yk_data[(ijA - ix) - 1] * theta;
          }
        }
        ix += ldBk;
      }
    }
  }
  return success;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for BFGSUpdate.cpp
//
// [EOF]
//
