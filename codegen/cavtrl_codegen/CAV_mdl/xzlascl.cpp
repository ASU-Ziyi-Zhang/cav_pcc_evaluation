//
// File: xzlascl.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xzlascl.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : double cfrom
//                double cto
//                int m
//                double A_data[]
//                int iA0
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void xzlascl(double cfrom, double cto, int m, double A_data[], int iA0)
{
  double cfromc;
  double ctoc;
  boolean_T notdone;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    double cfrom1;
    double cto1;
    double mul;
    int b_i;
    int scalarLB;
    int vectorUB;
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }
    scalarLB = m / 2 * 2;
    vectorUB = scalarLB - 2;
    for (int i{0}; i <= vectorUB; i += 2) {
      __m128d r;
      b_i = (iA0 + i) - 1;
      r = _mm_loadu_pd(&A_data[b_i]);
      r = _mm_mul_pd(r, _mm_set1_pd(mul));
      _mm_storeu_pd(&A_data[b_i], r);
    }
    for (int i{scalarLB}; i < m; i++) {
      b_i = (iA0 + i) - 1;
      A_data[b_i] *= mul;
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzlascl.cpp
//
// [EOF]
//
