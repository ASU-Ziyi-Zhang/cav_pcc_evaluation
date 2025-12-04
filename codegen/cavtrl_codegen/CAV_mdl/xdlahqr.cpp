//
// File: xdlahqr.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xdlahqr.h"
#include "CAV_ctrl_mdl_wTraJ_241219_rtwutil.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include "xnrm2.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : int ilo
//                int ihi
//                double h_data[]
//                const int h_size[2]
//                double wr_data[]
//                int &wr_size
//                double wi_data[]
//                int &wi_size
// Return Type  : int
//
namespace coder {
namespace internal {
namespace reflapack {
int xdlahqr(int ilo, int ihi, double h_data[], const int h_size[2],
            double wr_data[], int &wr_size, double wi_data[], int &wi_size)
{
  double v[3];
  double d;
  double h21;
  double h22;
  double rt1r;
  double rt2r;
  double tr;
  double tst;
  int info;
  wr_size = h_size[0];
  wi_size = wr_size;
  info = 0;
  if ((h_size[0] != 0) && (h_size[1] != 0)) {
    int b_i;
    int i;
    i = static_cast<unsigned char>(ilo - 1);
    for (b_i = 0; b_i < i; b_i++) {
      wr_data[b_i] = h_data[b_i + h_size[0] * b_i];
      wi_data[b_i] = 0.0;
    }
    i = ihi + 1;
    for (b_i = i; b_i <= wr_size; b_i++) {
      wr_data[b_i - 1] = h_data[(b_i + h_size[0] * (b_i - 1)) - 1];
      wi_data[b_i - 1] = 0.0;
    }
    if (ilo == ihi) {
      wr_data[ilo - 1] = h_data[(ilo + h_size[0] * (ilo - 1)) - 1];
      wi_data[ilo - 1] = 0.0;
    } else {
      double smlnum;
      int kdefl;
      int nr;
      boolean_T exitg1;
      if (ilo <= ihi - 2) {
        h_data[ihi - 1] = 0.0;
      }
      smlnum = 2.2250738585072014E-308 *
               (static_cast<double>((ihi - ilo) + 1) / 2.2204460492503131E-16);
      kdefl = 0;
      b_i = ihi - 1;
      exitg1 = false;
      while ((!exitg1) && (b_i + 1 >= ilo)) {
        int its;
        int knt;
        int l;
        boolean_T converged;
        boolean_T exitg2;
        l = ilo;
        converged = false;
        its = 0;
        exitg2 = false;
        while ((!exitg2) && (its < 301)) {
          double aa;
          double s;
          int k;
          boolean_T exitg3;
          k = b_i;
          exitg3 = false;
          while ((!exitg3) && (k + 1 > l)) {
            i = k + h_size[0] * (k - 1);
            d = std::abs(h_data[i]);
            if (d <= smlnum) {
              exitg3 = true;
            } else {
              knt = k + h_size[0] * k;
              h21 = std::abs(h_data[knt]);
              aa = h_data[i - 1];
              tst = std::abs(aa) + h21;
              if (tst == 0.0) {
                if (k - 1 >= ilo) {
                  tst = std::abs(h_data[k - 1]);
                }
                if (k + 2 <= ihi) {
                  tst += std::abs(h_data[knt + 1]);
                }
              }
              if (d <= 2.2204460492503131E-16 * tst) {
                tr = std::abs(h_data[knt - 1]);
                tst = std::abs(aa - h_data[knt]);
                aa = std::fmax(h21, tst);
                tst = std::fmin(h21, tst);
                s = aa + tst;
                if (std::fmin(d, tr) * (std::fmax(d, tr) / s) <=
                    std::fmax(smlnum,
                              2.2204460492503131E-16 * (tst * (aa / s)))) {
                  exitg3 = true;
                } else {
                  k--;
                }
              } else {
                k--;
              }
            }
          }
          l = k + 1;
          if (k + 1 > ilo) {
            h_data[k + h_size[0] * (k - 1)] = 0.0;
          }
          if (k + 1 >= b_i) {
            converged = true;
            exitg2 = true;
          } else {
            __m128d r;
            int m;
            kdefl++;
            if (kdefl - kdefl / 20 * 20 == 0) {
              s = std::abs(h_data[b_i + h_size[0] * (b_i - 1)]) +
                  std::abs(h_data[b_i - 1]);
              tst = 0.75 * s + h_data[b_i + h_size[0] * b_i];
              aa = -0.4375 * s;
              h21 = s;
              h22 = tst;
            } else if (kdefl - kdefl / 10 * 10 == 0) {
              s = std::abs(h_data[1]) + std::abs(h_data[h_size[0] + 2]);
              tst = 0.75 * s + h_data[0];
              aa = -0.4375 * s;
              h21 = s;
              h22 = tst;
            } else {
              knt = b_i + h_size[0] * (b_i - 1);
              tst = h_data[knt - 1];
              h21 = h_data[knt];
              aa = h_data[(b_i + h_size[0] * b_i) - 1];
              h22 = h_data[b_i + h_size[0] * b_i];
            }
            s = ((std::abs(tst) + std::abs(aa)) + std::abs(h21)) +
                std::abs(h22);
            if (s == 0.0) {
              rt1r = 0.0;
              tst = 0.0;
              rt2r = 0.0;
              h21 = 0.0;
            } else {
              tst /= s;
              h21 /= s;
              aa /= s;
              h22 /= s;
              tr = (tst + h22) / 2.0;
              tst = (tst - tr) * (h22 - tr) - aa * h21;
              h21 = std::sqrt(std::abs(tst));
              if (tst >= 0.0) {
                rt1r = tr * s;
                rt2r = rt1r;
                tst = h21 * s;
                h21 = -tst;
              } else {
                rt1r = tr + h21;
                rt2r = tr - h21;
                if (std::abs(rt1r - h22) <= std::abs(rt2r - h22)) {
                  rt1r *= s;
                  rt2r = rt1r;
                } else {
                  rt2r *= s;
                  rt1r = rt2r;
                }
                tst = 0.0;
                h21 = 0.0;
              }
            }
            m = b_i - 1;
            if (b_i - 1 >= 1) {
              aa = h_data[0] - rt2r;
              s = (std::abs(aa) + std::abs(h21)) + std::abs(h_data[1]);
              tr = h_data[1] / s;
              v[0] = (tr * h_data[h_size[0]] + aa * (aa / s)) - tst * (h21 / s);
              v[1] = tr * (((h_data[0] + h_data[h_size[0] + 1]) - rt1r) - rt2r);
              v[2] = tr * h_data[h_size[0] + 2];
              s = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
              r = _mm_loadu_pd(&v[0]);
              _mm_storeu_pd(&v[0], _mm_div_pd(r, _mm_set1_pd(s)));
              v[2] /= s;
            }
            for (k = m; k <= b_i; k++) {
              int scalarLB;
              int scalarLB_tmp;
              int vectorUB;
              int vectorUB_tmp;
              knt = (b_i - k) + 2;
              if (knt >= 3) {
                nr = 3;
              } else {
                nr = knt;
              }
              if (k > b_i - 1) {
                knt = ((k - 2) * wr_size + k) - 1;
                i = static_cast<unsigned char>(nr);
                for (int b_k{0}; b_k < i; b_k++) {
                  v[b_k] = h_data[knt + b_k];
                }
              }
              aa = v[0];
              s = 0.0;
              if (nr > 0) {
                tst = blas::b_xnrm2(nr - 1, v);
                if (tst != 0.0) {
                  h21 = rt_hypotd_snf(v[0], tst);
                  if (v[0] >= 0.0) {
                    h21 = -h21;
                  }
                  if (std::abs(h21) < 1.0020841800044864E-292) {
                    knt = 0;
                    do {
                      knt++;
                      scalarLB_tmp = (((nr - 1) / 2) << 1) + 2;
                      vectorUB_tmp = scalarLB_tmp - 2;
                      for (int b_k{2}; b_k <= vectorUB_tmp; b_k += 2) {
                        r = _mm_loadu_pd(&v[b_k - 1]);
                        _mm_storeu_pd(
                            &v[b_k - 1],
                            _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
                      }
                      for (int b_k{scalarLB_tmp}; b_k <= nr; b_k++) {
                        v[b_k - 1] *= 9.9792015476736E+291;
                      }
                      h21 *= 9.9792015476736E+291;
                      aa *= 9.9792015476736E+291;
                    } while ((std::abs(h21) < 1.0020841800044864E-292) &&
                             (knt < 20));
                    h21 = rt_hypotd_snf(aa, blas::b_xnrm2(nr - 1, v));
                    if (aa >= 0.0) {
                      h21 = -h21;
                    }
                    s = (h21 - aa) / h21;
                    tst = 1.0 / (aa - h21);
                    for (int b_k{2}; b_k <= vectorUB_tmp; b_k += 2) {
                      r = _mm_loadu_pd(&v[b_k - 1]);
                      _mm_storeu_pd(&v[b_k - 1],
                                    _mm_mul_pd(_mm_set1_pd(tst), r));
                    }
                    for (int b_k{scalarLB_tmp}; b_k <= nr; b_k++) {
                      v[b_k - 1] *= tst;
                    }
                    for (int b_k{0}; b_k < knt; b_k++) {
                      h21 *= 1.0020841800044864E-292;
                    }
                    aa = h21;
                  } else {
                    s = (h21 - v[0]) / h21;
                    tst = 1.0 / (v[0] - h21);
                    scalarLB = (((nr - 1) / 2) << 1) + 2;
                    vectorUB = scalarLB - 2;
                    for (int b_k{2}; b_k <= vectorUB; b_k += 2) {
                      r = _mm_loadu_pd(&v[b_k - 1]);
                      _mm_storeu_pd(&v[b_k - 1],
                                    _mm_mul_pd(_mm_set1_pd(tst), r));
                    }
                    for (int b_k{scalarLB}; b_k <= nr; b_k++) {
                      v[b_k - 1] *= tst;
                    }
                    aa = h21;
                  }
                }
              }
              if (k > b_i - 1) {
                h_data[k - 1] = aa;
                h_data[k] = 0.0;
                if (k < b_i) {
                  // Check node always fails. would cause program termination
                  // and was eliminated
                }
              }
              d = v[1];
              tst = s * v[1];
              if (nr == 3) {
                tr = v[2];
                aa = s * v[2];
                for (nr = k; nr <= b_i + 1; nr++) {
                  i = k + h_size[0] * (nr - 1);
                  rt2r = h_data[i - 1];
                  h22 = h_data[i];
                  rt1r = h_data[i + 1];
                  h21 = (rt2r + d * h22) + tr * rt1r;
                  rt2r -= h21 * s;
                  h_data[i - 1] = rt2r;
                  h22 -= h21 * tst;
                  h_data[i] = h22;
                  rt1r -= h21 * aa;
                  h_data[i + 1] = rt1r;
                }
                if (k + 3 <= b_i + 1) {
                  i = k;
                } else {
                  i = b_i - 2;
                }
                scalarLB = (((i + 3) / 2) << 1) + 1;
                vectorUB = scalarLB - 2;
                for (nr = 1; nr <= vectorUB; nr += 2) {
                  __m128d r1;
                  __m128d r2;
                  __m128d r3;
                  knt = (nr + h_size[0] * k) - 1;
                  r = _mm_loadu_pd(&h_data[knt]);
                  scalarLB_tmp = (nr + h_size[0] * (k + 1)) - 1;
                  r1 = _mm_loadu_pd(&h_data[scalarLB_tmp]);
                  vectorUB_tmp = (nr + h_size[0] * (k - 1)) - 1;
                  r2 = _mm_loadu_pd(&h_data[vectorUB_tmp]);
                  r3 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(d), r)),
                                  _mm_mul_pd(_mm_set1_pd(tr), r1));
                  _mm_storeu_pd(&h_data[vectorUB_tmp],
                                _mm_sub_pd(r2, _mm_mul_pd(r3, _mm_set1_pd(s))));
                  _mm_storeu_pd(
                      &h_data[knt],
                      _mm_sub_pd(r, _mm_mul_pd(r3, _mm_set1_pd(tst))));
                  _mm_storeu_pd(
                      &h_data[scalarLB_tmp],
                      _mm_sub_pd(r1, _mm_mul_pd(r3, _mm_set1_pd(aa))));
                }
                for (nr = scalarLB; nr <= i + 3; nr++) {
                  knt = (nr + h_size[0] * (k - 1)) - 1;
                  rt2r = h_data[knt];
                  scalarLB_tmp = (nr + h_size[0] * k) - 1;
                  h22 = h_data[scalarLB_tmp];
                  vectorUB_tmp = (nr + h_size[0] * (k + 1)) - 1;
                  rt1r = h_data[vectorUB_tmp];
                  h21 = (rt2r + d * h22) + tr * rt1r;
                  rt2r -= h21 * s;
                  h_data[knt] = rt2r;
                  h22 -= h21 * tst;
                  h_data[scalarLB_tmp] = h22;
                  rt1r -= h21 * aa;
                  h_data[vectorUB_tmp] = rt1r;
                }
              } else if (nr == 2) {
                for (nr = k; nr <= b_i + 1; nr++) {
                  i = k + h_size[0] * (nr - 1);
                  tr = h_data[i - 1];
                  rt2r = h_data[i];
                  h21 = tr + d * rt2r;
                  tr -= h21 * s;
                  h_data[i - 1] = tr;
                  rt2r -= h21 * tst;
                  h_data[i] = rt2r;
                }
                scalarLB = (((b_i + 1) / 2) << 1) + 1;
                vectorUB = scalarLB - 2;
                for (nr = 1; nr <= vectorUB; nr += 2) {
                  __m128d r1;
                  __m128d r2;
                  i = (nr + h_size[0] * k) - 1;
                  r = _mm_loadu_pd(&h_data[i]);
                  knt = (nr + h_size[0] * (k - 1)) - 1;
                  r1 = _mm_loadu_pd(&h_data[knt]);
                  r2 = _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(d), r));
                  _mm_storeu_pd(&h_data[knt],
                                _mm_sub_pd(r1, _mm_mul_pd(r2, _mm_set1_pd(s))));
                  _mm_storeu_pd(
                      &h_data[i],
                      _mm_sub_pd(r, _mm_mul_pd(r2, _mm_set1_pd(tst))));
                }
                for (nr = scalarLB; nr <= b_i + 1; nr++) {
                  i = (nr + h_size[0] * (k - 1)) - 1;
                  tr = h_data[i];
                  knt = (nr + h_size[0] * k) - 1;
                  rt2r = h_data[knt];
                  h21 = tr + d * rt2r;
                  tr -= h21 * s;
                  h_data[i] = tr;
                  rt2r -= h21 * tst;
                  h_data[knt] = rt2r;
                }
              }
            }
            its++;
          }
        }
        if (!converged) {
          info = b_i + 1;
          exitg1 = true;
        } else {
          if (l == b_i + 1) {
            wr_data[b_i] = h_data[b_i + h_size[0] * b_i];
            wi_data[b_i] = 0.0;
          } else if (l == b_i) {
            i = b_i + h_size[0] * b_i;
            d = h_data[i - 1];
            knt = b_i + h_size[0] * (b_i - 1);
            tr = h_data[knt];
            rt2r = h_data[i];
            wr_data[b_i - 1] =
                xdlanv2(h_data[(b_i + h_size[0] * (b_i - 1)) - 1], d, tr, rt2r,
                        wi_data[b_i - 1], h22, rt1r, tst, h21);
            wr_data[b_i] = h22;
            wi_data[b_i] = rt1r;
            h_data[i - 1] = d;
            h_data[knt] = tr;
            h_data[i] = rt2r;
          }
          kdefl = 0;
          b_i = l - 2;
        }
      }
      if ((info != 0) && (wr_size > 2)) {
        for (nr = 3; nr <= wr_size; nr++) {
          for (b_i = nr; b_i <= wr_size; b_i++) {
            h_data[(b_i + h_size[0] * (nr - 3)) - 1] = 0.0;
          }
        }
      }
    }
  }
  return info;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xdlahqr.cpp
//
// [EOF]
//
