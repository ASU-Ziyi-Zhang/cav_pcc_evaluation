//
// File: qrsolve.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "qrsolve.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                const double B_data[]
//                int B_size
//                double Y_data[]
// Return Type  : int
//
namespace coder {
namespace internal {
int qrsolve(const double A_data[], const int A_size[2], const double B_data[],
            int B_size, double Y_data[])
{
  double x_data[2401];
  double work_data[49];
  double tau_data[13];
  double smax;
  int Y_size;
  int b_i;
  int b_k;
  int ii;
  int ij;
  int kend;
  int loop_ub;
  int m;
  int pvt;
  int u1;
  int x_size_idx_0;
  int x_size_idx_1;
  signed char jpvt_data[13];
  m = A_size[0];
  x_size_idx_0 = A_size[0];
  loop_ub = A_size[1];
  x_size_idx_1 = A_size[1];
  ij = A_size[0] * A_size[1];
  if (ij - 1 >= 0) {
    std::copy(&A_data[0], &A_data[ij], &x_data[0]);
  }
  ij = A_size[0];
  u1 = A_size[1];
  if (ij <= u1) {
    u1 = ij;
  }
  if (u1 - 1 >= 0) {
    std::memset(&tau_data[0], 0,
                static_cast<unsigned int>(u1) * sizeof(double));
  }
  if ((A_size[0] == 0) || (A_size[1] == 0)) {
    for (ii = 0; ii < loop_ub; ii++) {
      jpvt_data[ii] = static_cast<signed char>(ii + 1);
    }
  } else {
    double vn1_data[13];
    double vn2_data[13];
    double absxk;
    double scale;
    for (int k{0}; k < loop_ub; k++) {
      jpvt_data[k] = static_cast<signed char>(k + 1);
      work_data[k] = 0.0;
      vn1_data[k] = 0.0;
      vn2_data[k] = 0.0;
      ij = k * m;
      smax = 0.0;
      if (m >= 1) {
        if (m == 1) {
          smax = std::abs(A_data[ij]);
        } else {
          scale = 3.3121686421112381E-170;
          kend = ij + m;
          for (b_k = ij + 1; b_k <= kend; b_k++) {
            absxk = std::abs(A_data[b_k - 1]);
            if (absxk > scale) {
              double t;
              t = scale / absxk;
              smax = smax * t * t + 1.0;
              scale = absxk;
            } else {
              double t;
              t = absxk / scale;
              smax += t * t;
            }
          }
          smax = scale * std::sqrt(smax);
        }
      }
      vn1_data[k] = smax;
      vn2_data[k] = smax;
    }
    for (int i{0}; i < u1; i++) {
      int ip1;
      int mmi;
      int nmi;
      ip1 = i + 2;
      b_k = i * m;
      ii = b_k + i;
      nmi = loop_ub - i;
      mmi = m - i;
      if (nmi < 1) {
        kend = -1;
      } else {
        kend = 0;
        if (nmi > 1) {
          smax = std::abs(vn1_data[i]);
          for (int k{2}; k <= nmi; k++) {
            scale = std::abs(vn1_data[(i + k) - 1]);
            if (scale > smax) {
              kend = k - 1;
              smax = scale;
            }
          }
        }
      }
      pvt = i + kend;
      if (pvt != i) {
        kend = pvt * m;
        for (int k{0}; k < m; k++) {
          ij = kend + k;
          smax = x_data[ij];
          b_i = b_k + k;
          x_data[ij] = x_data[b_i];
          x_data[b_i] = smax;
        }
        kend = jpvt_data[pvt];
        jpvt_data[pvt] = jpvt_data[i];
        jpvt_data[i] = static_cast<signed char>(kend);
        vn1_data[pvt] = vn1_data[i];
        vn2_data[pvt] = vn2_data[i];
      }
      if (i + 1 < m) {
        smax = x_data[ii];
        absxk = reflapack::xzlarfg(mmi, smax, x_data, ii + 2);
        tau_data[i] = absxk;
        x_data[ii] = smax;
      } else {
        absxk = 0.0;
        tau_data[i] = 0.0;
      }
      if (i + 1 < loop_ub) {
        smax = x_data[ii];
        x_data[ii] = 1.0;
        reflapack::xzlarf(mmi, nmi - 1, ii + 1, absxk, x_data, (ii + m) + 1, m,
                          work_data);
        x_data[ii] = smax;
      }
      for (ii = ip1; ii <= loop_ub; ii++) {
        ij = i + (ii - 1) * m;
        absxk = vn1_data[ii - 1];
        if (absxk != 0.0) {
          smax = std::abs(x_data[ij]) / absxk;
          smax = 1.0 - smax * smax;
          if (smax < 0.0) {
            smax = 0.0;
          }
          scale = absxk / vn2_data[ii - 1];
          scale = smax * (scale * scale);
          if (scale <= 1.4901161193847656E-8) {
            if (i + 1 < m) {
              absxk = blas::xnrm2(mmi - 1, x_data, ij + 2);
              vn1_data[ii - 1] = absxk;
              vn2_data[ii - 1] = absxk;
            } else {
              vn1_data[ii - 1] = 0.0;
              vn2_data[ii - 1] = 0.0;
            }
          } else {
            vn1_data[ii - 1] = absxk * std::sqrt(smax);
          }
        }
      }
    }
  }
  pvt = 0;
  if (A_size[0] < A_size[1]) {
    ij = A_size[0];
    kend = A_size[1];
  } else {
    ij = A_size[1];
    kend = A_size[0];
  }
  if (ij > 0) {
    smax = 2.2204460492503131E-15 * static_cast<double>(kend) *
           std::abs(x_data[0]);
    while ((pvt < ij) &&
           (!(std::abs(x_data[pvt + x_size_idx_0 * pvt]) <= smax))) {
      pvt++;
    }
  }
  if (B_size - 1 >= 0) {
    std::copy(&B_data[0], &B_data[B_size], &work_data[0]);
  }
  Y_size = A_size[1];
  if (x_size_idx_1 - 1 >= 0) {
    std::memset(&Y_data[0], 0,
                static_cast<unsigned int>(x_size_idx_1) * sizeof(double));
  }
  if (A_size[0] < A_size[1]) {
    b_i = A_size[0];
  } else {
    b_i = A_size[1];
  }
  for (ii = 0; ii < b_i; ii++) {
    if (tau_data[ii] != 0.0) {
      smax = work_data[ii];
      ij = ii + 2;
      for (int i{ij}; i <= x_size_idx_0; i++) {
        smax += x_data[(i + x_size_idx_0 * ii) - 1] * work_data[i - 1];
      }
      smax *= tau_data[ii];
      if (smax != 0.0) {
        work_data[ii] -= smax;
        for (int i{ij}; i <= x_size_idx_0; i++) {
          work_data[i - 1] -= x_data[(i + x_size_idx_0 * ii) - 1] * smax;
        }
      }
    }
  }
  for (int i{0}; i < pvt; i++) {
    Y_data[jpvt_data[i] - 1] = work_data[i];
  }
  for (ii = pvt; ii >= 1; ii--) {
    ij = jpvt_data[ii - 1] - 1;
    kend = x_size_idx_0 * (ii - 1);
    Y_data[jpvt_data[ii - 1] - 1] = Y_data[ij] / x_data[(ii + kend) - 1];
    for (int i{0}; i <= ii - 2; i++) {
      b_k = jpvt_data[i] - 1;
      Y_data[b_k] -= Y_data[ij] * x_data[i + kend];
    }
  }
  return Y_size;
}

} // namespace internal
} // namespace coder

//
// File trailer for qrsolve.cpp
//
// [EOF]
//
