//
// File: xgeqp3.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xgeqp3.h"
#include "CAV_ctrl_mdl_wTraJ_241219_data.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xzgeqp3.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : double A_data[]
//                const int A_size[2]
//                int m
//                int n
//                int jpvt_data[]
//                double tau_data[]
// Return Type  : int
//
namespace coder {
namespace internal {
namespace lapack {
int xgeqp3(double A_data[], const int A_size[2], int m, int n, int jpvt_data[],
           double tau_data[])
{
  double vn1_data[49];
  double vn2_data[49];
  double work_data[49];
  int ix;
  int ma;
  int minmn_tmp;
  int tau_size;
  ma = A_size[0];
  ix = A_size[0];
  tau_size = A_size[1];
  if (ix <= tau_size) {
    tau_size = ix;
  }
  if (m <= n) {
    minmn_tmp = m;
  } else {
    minmn_tmp = n;
  }
  if (tau_size - 1 >= 0) {
    std::memset(&tau_data[0], 0,
                static_cast<unsigned int>(tau_size) * sizeof(double));
  }
  if ((A_size[0] == 0) || (A_size[1] == 0) || (minmn_tmp < 1)) {
    int iy;
    ix = (n / 4) << 2;
    iy = ix - 4;
    for (int pvt{0}; pvt <= iy; pvt += 4) {
      _mm_storeu_si128(
          (__m128i *)&jpvt_data[pvt],
          _mm_add_epi32(_mm_add_epi32(_mm_set1_epi32(pvt),
                                      _mm_loadu_si128((const __m128i *)&iv[0])),
                        _mm_set1_epi32(1)));
    }
    for (int pvt{ix}; pvt < n; pvt++) {
      jpvt_data[pvt] = pvt + 1;
    }
  } else {
    double temp;
    int i;
    int iy;
    int nfxd;
    int pvt;
    int temp_tmp;
    nfxd = 0;
    for (pvt = 0; pvt < n; pvt++) {
      if (jpvt_data[pvt] != 0) {
        nfxd++;
        if (pvt + 1 != nfxd) {
          ix = pvt * ma;
          iy = (nfxd - 1) * ma;
          for (int k{0}; k < m; k++) {
            temp_tmp = ix + k;
            temp = A_data[temp_tmp];
            i = iy + k;
            A_data[temp_tmp] = A_data[i];
            A_data[i] = temp;
          }
          jpvt_data[pvt] = jpvt_data[nfxd - 1];
          jpvt_data[nfxd - 1] = pvt + 1;
        } else {
          jpvt_data[pvt] = pvt + 1;
        }
      } else {
        jpvt_data[pvt] = pvt + 1;
      }
    }
    if (nfxd > minmn_tmp) {
      nfxd = minmn_tmp;
    }
    reflapack::qrf(A_data, A_size, m, n, nfxd, tau_data);
    if (nfxd < minmn_tmp) {
      double d;
      ma = A_size[0];
      ix = A_size[1];
      if (ix - 1 >= 0) {
        std::memset(&work_data[0], 0,
                    static_cast<unsigned int>(ix) * sizeof(double));
        std::memset(&vn1_data[0], 0,
                    static_cast<unsigned int>(ix) * sizeof(double));
        std::memset(&vn2_data[0], 0,
                    static_cast<unsigned int>(ix) * sizeof(double));
      }
      i = nfxd + 1;
      for (pvt = i; pvt <= n; pvt++) {
        d = blas::xnrm2(m - nfxd, A_data, (nfxd + (pvt - 1) * ma) + 1);
        vn1_data[pvt - 1] = d;
        vn2_data[pvt - 1] = d;
      }
      for (int b_i{i}; b_i <= minmn_tmp; b_i++) {
        double s;
        int ii;
        int ip1;
        int mmi;
        int nmi;
        ip1 = b_i + 1;
        nfxd = (b_i - 1) * ma;
        ii = (nfxd + b_i) - 1;
        nmi = (n - b_i) + 1;
        mmi = m - b_i;
        if (nmi < 1) {
          iy = -2;
        } else {
          iy = -1;
          if (nmi > 1) {
            temp = std::abs(vn1_data[b_i - 1]);
            for (int k{2}; k <= nmi; k++) {
              s = std::abs(vn1_data[(b_i + k) - 2]);
              if (s > temp) {
                iy = k - 2;
                temp = s;
              }
            }
          }
        }
        pvt = b_i + iy;
        if (pvt + 1 != b_i) {
          ix = pvt * ma;
          for (int k{0}; k < m; k++) {
            temp_tmp = ix + k;
            temp = A_data[temp_tmp];
            iy = nfxd + k;
            A_data[temp_tmp] = A_data[iy];
            A_data[iy] = temp;
          }
          ix = jpvt_data[pvt];
          jpvt_data[pvt] = jpvt_data[b_i - 1];
          jpvt_data[b_i - 1] = ix;
          vn1_data[pvt] = vn1_data[b_i - 1];
          vn2_data[pvt] = vn2_data[b_i - 1];
        }
        if (b_i < m) {
          temp = A_data[ii];
          d = reflapack::xzlarfg(mmi + 1, temp, A_data, ii + 2);
          tau_data[b_i - 1] = d;
          A_data[ii] = temp;
        } else {
          d = 0.0;
          tau_data[b_i - 1] = 0.0;
        }
        if (b_i < n) {
          temp = A_data[ii];
          A_data[ii] = 1.0;
          reflapack::xzlarf(mmi + 1, nmi - 1, ii + 1, d, A_data, (ii + ma) + 1,
                            ma, work_data);
          A_data[ii] = temp;
        }
        for (pvt = ip1; pvt <= n; pvt++) {
          ix = b_i + (pvt - 1) * ma;
          d = vn1_data[pvt - 1];
          if (d != 0.0) {
            temp = std::abs(A_data[ix - 1]) / d;
            temp = 1.0 - temp * temp;
            if (temp < 0.0) {
              temp = 0.0;
            }
            s = d / vn2_data[pvt - 1];
            s = temp * (s * s);
            if (s <= 1.4901161193847656E-8) {
              if (b_i < m) {
                d = blas::xnrm2(mmi, A_data, ix + 1);
                vn1_data[pvt - 1] = d;
                vn2_data[pvt - 1] = d;
              } else {
                vn1_data[pvt - 1] = 0.0;
                vn2_data[pvt - 1] = 0.0;
              }
            } else {
              vn1_data[pvt - 1] = d * std::sqrt(temp);
            }
          }
        }
      }
    }
  }
  return tau_size;
}

} // namespace lapack
} // namespace internal
} // namespace coder

//
// File trailer for xgeqp3.cpp
//
// [EOF]
//
