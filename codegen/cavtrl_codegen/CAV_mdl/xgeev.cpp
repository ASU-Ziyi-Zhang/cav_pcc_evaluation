//
// File: xgeev.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xgeev.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xzgebal.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include "xzlascl.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                creal_T W_data[]
//                int &W_size
// Return Type  : int
//
namespace coder {
namespace internal {
namespace lapack {
int xgeev(const double A_data[], const int A_size[2], creal_T W_data[],
          int &W_size)
{
  double a_data[2401];
  double wi_data[49];
  double work_data[49];
  double tau_data[48];
  double scale_data[3];
  double absxk;
  double anrm;
  double ctoc;
  int a_size[2];
  int ihi;
  int info;
  int jA;
  int loop_ub;
  int ntau;
  boolean_T exitg1;
  loop_ub = A_size[0];
  a_size[0] = A_size[0];
  a_size[1] = A_size[1];
  ntau = A_size[0] * A_size[1];
  if (ntau - 1 >= 0) {
    std::copy(&A_data[0], &A_data[ntau], &a_data[0]);
  }
  info = 0;
  anrm = 0.0;
  jA = 0;
  exitg1 = false;
  while ((!exitg1) && (jA <= ntau - 1)) {
    absxk = std::abs(A_data[jA]);
    if (std::isnan(absxk)) {
      anrm = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }
      jA++;
    }
  }
  if (std::isinf(anrm) || std::isnan(anrm)) {
    W_size = A_size[0];
    for (int i{0}; i < loop_ub; i++) {
      W_data[i].re = rtNaN;
      W_data[i].im = 0.0;
    }
  } else {
    __m128d r;
    double cfrom1;
    double cscale;
    int i;
    int ia;
    int ic0;
    int ilo;
    int lastv;
    int n_tmp;
    boolean_T guard1;
    boolean_T scalea;
    cscale = anrm;
    scalea = false;
    guard1 = false;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      scalea = true;
      cscale = 6.7178761075670888E-139;
      guard1 = true;
    } else if (anrm > 1.4885657073574029E+138) {
      scalea = true;
      cscale = 1.4885657073574029E+138;
      guard1 = true;
    }
    if (guard1) {
      boolean_T notdone;
      absxk = anrm;
      ctoc = cscale;
      a_size[0] = A_size[0];
      a_size[1] = A_size[1];
      if (ntau - 1 >= 0) {
        std::copy(&A_data[0], &A_data[ntau], &a_data[0]);
      }
      notdone = true;
      while (notdone) {
        double cto1;
        double mul;
        cfrom1 = absxk * 2.0041683600089728E-292;
        cto1 = ctoc / 4.9896007738368E+291;
        if ((cfrom1 > ctoc) && (ctoc != 0.0)) {
          mul = 2.0041683600089728E-292;
          absxk = cfrom1;
        } else if (cto1 > absxk) {
          mul = 4.9896007738368E+291;
          ctoc = cto1;
        } else {
          mul = ctoc / absxk;
          notdone = false;
        }
        for (lastv = 0; lastv < loop_ub; lastv++) {
          jA = lastv * loop_ub - 1;
          ic0 = loop_ub / 2 * 2;
          ia = ic0 - 2;
          for (int b_i{0}; b_i <= ia; b_i += 2) {
            i = (jA + b_i) + 1;
            r = _mm_loadu_pd(&a_data[i]);
            r = _mm_mul_pd(r, _mm_set1_pd(mul));
            _mm_storeu_pd(&a_data[i], r);
          }
          for (int b_i{ic0}; b_i < loop_ub; b_i++) {
            i = (jA + b_i) + 1;
            a_data[i] *= mul;
          }
        }
      }
    }
    ilo = reflapack::xzgebal(a_data, a_size, ihi, scale_data, ntau);
    n_tmp = a_size[0];
    if (a_size[0] < 1) {
      ntau = 0;
    } else {
      ntau = a_size[0] - 1;
    }
    if ((ihi - ilo) + 1 > 1) {
      i = static_cast<unsigned char>(ilo - 1);
      if (i - 1 >= 0) {
        std::memset(&tau_data[0], 0,
                    static_cast<unsigned int>(i) * sizeof(double));
      }
      if (ihi <= ntau) {
        std::memset(&tau_data[ihi + -1], 0,
                    static_cast<unsigned int>((ntau - ihi) + 1) *
                        sizeof(double));
      }
      if (n_tmp - 1 >= 0) {
        std::memset(&work_data[0], 0,
                    static_cast<unsigned int>(n_tmp) * sizeof(double));
      }
      i = ihi - 1;
      for (int b_i{ilo}; b_i <= i; b_i++) {
        int in;
        int iv0_tmp;
        int lastc;
        int n;
        ntau = (b_i - 1) * n_tmp;
        in = b_i * n_tmp;
        jA = b_i + a_size[0] * (b_i - 1);
        ctoc = a_data[jA];
        n = ihi - b_i;
        loop_ub = b_i + 2;
        if (loop_ub > n_tmp) {
          loop_ub = n_tmp;
        }
        cfrom1 = reflapack::xzlarfg(n, ctoc, a_data, loop_ub + ntau);
        tau_data[b_i - 1] = cfrom1;
        a_data[jA] = 1.0;
        iv0_tmp = (b_i + ntau) + 1;
        ic0 = in + 1;
        if (cfrom1 != 0.0) {
          lastv = n;
          ntau = (iv0_tmp + n) - 1;
          while ((lastv > 0) && (a_data[ntau - 1] == 0.0)) {
            lastv--;
            ntau--;
          }
          lastc = ihi;
          exitg1 = false;
          while ((!exitg1) && (lastc > 0)) {
            int exitg2;
            ntau = in + lastc;
            ia = ntau;
            do {
              exitg2 = 0;
              if ((n_tmp > 0) && (ia <= ntau + (lastv - 1) * n_tmp)) {
                if (a_data[ia - 1] != 0.0) {
                  exitg2 = 1;
                } else {
                  ia += n_tmp;
                }
              } else {
                lastc--;
                exitg2 = 2;
              }
            } while (exitg2 == 0);
            if (exitg2 == 1) {
              exitg1 = true;
            }
          }
        } else {
          lastv = 0;
          lastc = 0;
        }
        if (lastv > 0) {
          int i1;
          int i2;
          if (lastc != 0) {
            i1 = static_cast<unsigned char>(lastc);
            if (i1 - 1 >= 0) {
              std::memset(&work_data[0], 0,
                          static_cast<unsigned int>(i1) * sizeof(double));
            }
            ntau = iv0_tmp - 1;
            i1 = (in + n_tmp * (lastv - 1)) + 1;
            for (loop_ub = ic0; n_tmp < 0 ? loop_ub >= i1 : loop_ub <= i1;
                 loop_ub += n_tmp) {
              i2 = (loop_ub + lastc) - 1;
              for (ia = loop_ub; ia <= i2; ia++) {
                jA = ia - loop_ub;
                work_data[jA] += a_data[ia - 1] * a_data[ntau];
              }
              ntau++;
            }
          }
          cfrom1 = -tau_data[b_i - 1];
          if (!(cfrom1 == 0.0)) {
            jA = in;
            i1 = static_cast<unsigned char>(lastv);
            for (lastv = 0; lastv < i1; lastv++) {
              absxk = a_data[(iv0_tmp + lastv) - 1];
              if (absxk != 0.0) {
                absxk *= cfrom1;
                i2 = jA + 1;
                ntau = lastc + jA;
                ic0 = ((((ntau - jA) / 2) << 1) + jA) + 1;
                ia = ic0 - 2;
                for (loop_ub = i2; loop_ub <= ia; loop_ub += 2) {
                  __m128d r1;
                  r = _mm_loadu_pd(&work_data[(loop_ub - jA) - 1]);
                  r1 = _mm_loadu_pd(&a_data[loop_ub - 1]);
                  _mm_storeu_pd(
                      &a_data[loop_ub - 1],
                      _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(absxk))));
                }
                for (loop_ub = ic0; loop_ub <= ntau; loop_ub++) {
                  a_data[loop_ub - 1] += work_data[(loop_ub - jA) - 1] * absxk;
                }
              }
              jA += n_tmp;
            }
          }
        }
        reflapack::xzlarf(n, n_tmp - b_i, iv0_tmp, tau_data[b_i - 1], a_data,
                          (b_i + in) + 1, n_tmp, work_data);
        a_data[b_i + a_size[0] * (b_i - 1)] = ctoc;
      }
    }
    info = reflapack::xdlahqr(ilo, ihi, a_data, a_size, work_data, W_size,
                              wi_data, jA);
    if (scalea) {
      i = A_size[0] - info;
      reflapack::xzlascl(cscale, anrm, i, work_data, info + 1);
      reflapack::xzlascl(cscale, anrm, i, wi_data, info + 1);
      if (info != 0) {
        reflapack::xzlascl(cscale, anrm, ilo - 1, work_data, 1);
        reflapack::xzlascl(cscale, anrm, ilo - 1, wi_data, 1);
      }
    }
    if (info != 0) {
      for (int b_i{ilo}; b_i <= info; b_i++) {
        work_data[b_i - 1] = rtNaN;
        wi_data[b_i - 1] = 0.0;
      }
    }
    for (i = 0; i < W_size; i++) {
      W_data[i].re = work_data[i];
      W_data[i].im = wi_data[i];
    }
  }
  return info;
}

} // namespace lapack
} // namespace internal
} // namespace coder

//
// File trailer for xgeev.cpp
//
// [EOF]
//
