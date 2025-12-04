/*
 * longitudinal_mpc.cpp
 *
 * Code generation for model "longitudinal_mpc".
 *
 * Model version              : 1.201
 * Simulink Coder version : 24.1 (R2024a) 19-Nov-2023
 * C++ source code generated on : Mon Aug 19 14:47:00 2024
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Execution efficiency
 * Validation result: Passed (8), Warning (1), Error (0)
 */

#include "longitudinal_mpc.h"
#include "rtwtypes.h"
#include "longitudinal_mpc_types.h"
#include "coder_array.h"
#include <cstring>
#include <cmath>
#include <emmintrin.h>
#include "longitudinal_mpc_private.h"
#include "cmath"

int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

real_T longitudinal_mpc::longitudinal_mpc_norm(const real_T x[16])
{
  real_T y;
  int32_T b_j;
  boolean_T exitg1;
  y = 0.0;
  b_j = 0;
  exitg1 = false;
  while ((!exitg1) && (b_j < 4)) {
    real_T s;
    int32_T s_tmp;
    s_tmp = b_j << 2;
    s = ((std::abs(x[s_tmp + 1]) + std::abs(x[s_tmp])) + std::abs(x[s_tmp + 2]))
      + std::abs(x[s_tmp + 3]);
    if (std::isnan(s)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (s > y) {
        y = s;
      }

      b_j++;
    }
  }

  return y;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (std::isinf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = (rtNaN);
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

void longitudinal_mpc::longitudinal_mpc_mpower(const real_T a[16], real_T b,
  real_T c[16])
{
  real_T aBuffer[16];
  real_T a_0[16];
  real_T cBuffer[16];
  real_T cBuffer_0[16];
  real_T cBuffer_1[16];
  real_T tmp_3[2];
  real_T e;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  e = std::abs(b);
  if (e <= 2.147483647E+9) {
    int32_T n;
    int32_T n_0;
    int32_T nb;
    int32_T nbitson;
    std::memcpy(&a_0[0], &a[0], sizeof(real_T) << 4U);
    n = static_cast<int32_T>(e);
    n_0 = static_cast<int32_T>(e);
    nbitson = 0;
    nb = -1;
    while (n_0 > 0) {
      nb++;
      if ((static_cast<uint32_T>(n_0) & 1U) != 0U) {
        nbitson++;
      }

      n_0 >>= 1;
    }

    if (static_cast<int32_T>(e) <= 2) {
      if (b == 2.0) {
        for (nb = 0; nb < 4; nb++) {
          for (int32_T i{0}; i <= 2; i += 2) {
            int32_T tmp_4;
            tmp_4 = nb << 2;
            _mm_storeu_pd(&c[i + tmp_4], _mm_add_pd(_mm_add_pd(_mm_add_pd
              (_mm_mul_pd(_mm_set1_pd(a[tmp_4 + 1]), _mm_loadu_pd(&a[i + 4])),
               _mm_mul_pd(_mm_set1_pd(a[tmp_4]), _mm_loadu_pd(&a[i]))),
              _mm_mul_pd(_mm_set1_pd(a[tmp_4 + 2]), _mm_loadu_pd(&a[i + 8]))),
              _mm_mul_pd(_mm_set1_pd(a[tmp_4 + 3]), _mm_loadu_pd(&a[i + 12]))));
          }
        }
      } else {
        boolean_T firstmult;
        firstmult = false;
        for (n = 0; n < 16; n++) {
          if (firstmult || std::isnan(a[n])) {
            firstmult = true;
          }
        }

        if (firstmult) {
          for (nb = 0; nb < 16; nb++) {
            c[nb] = (rtNaN);
          }
        } else {
          std::memset(&c[0], 0, sizeof(real_T) << 4U);
          c[0] = 1.0;
          c[5] = 1.0;
          c[10] = 1.0;
          c[15] = 1.0;
        }
      }
    } else {
      real_T c_0;
      real_T c_1;
      real_T c_2;
      real_T ed2;
      int32_T tmp_4;
      boolean_T aBufferInUse;
      boolean_T cBufferInUse;
      boolean_T firstmult;
      firstmult = true;
      aBufferInUse = false;
      cBufferInUse = ((static_cast<uint32_T>(nbitson) & 1U) == 0U);
      n_0 = nb - 1;
      for (nbitson = 0; nbitson <= n_0; nbitson++) {
        int32_T tmp_5;
        if ((static_cast<uint32_T>(n) & 1U) != 0U) {
          if (firstmult) {
            firstmult = false;
            if (cBufferInUse) {
              if (aBufferInUse) {
                std::memcpy(&cBuffer[0], &aBuffer[0], sizeof(real_T) << 4U);
              } else {
                std::memcpy(&cBuffer[0], &a_0[0], sizeof(real_T) << 4U);
              }
            } else if (aBufferInUse) {
              std::memcpy(&c[0], &aBuffer[0], sizeof(real_T) << 4U);
            } else {
              std::memcpy(&c[0], &a_0[0], sizeof(real_T) << 4U);
            }
          } else {
            if (aBufferInUse) {
              if (cBufferInUse) {
                for (nb = 0; nb < 4; nb++) {
                  ed2 = cBuffer[nb + 4];
                  e = cBuffer[nb];
                  c_0 = cBuffer[nb + 8];
                  c_1 = cBuffer[nb + 12];
                  for (int32_T i{0}; i <= 2; i += 2) {
                    tmp_4 = (i + 1) << 2;
                    tmp_5 = i << 2;
                    _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                      (_mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 1], aBuffer[tmp_5 +
                      1]), _mm_set1_pd(ed2)), _mm_mul_pd(_mm_set_pd
                      (aBuffer[tmp_4], aBuffer[tmp_5]), _mm_set1_pd(e))),
                      _mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 2], aBuffer[tmp_5 +
                      2]), _mm_set1_pd(c_0))), _mm_mul_pd(_mm_set_pd
                      (aBuffer[tmp_4 + 3], aBuffer[tmp_5 + 3]), _mm_set1_pd(c_1))));
                    c[nb + tmp_5] = tmp_3[0];
                    c[nb + tmp_4] = tmp_3[1];
                  }
                }
              } else {
                for (nb = 0; nb < 4; nb++) {
                  e = c[nb + 4];
                  c_0 = c[nb];
                  c_1 = c[nb + 8];
                  c_2 = c[nb + 12];
                  for (int32_T i{0}; i <= 2; i += 2) {
                    tmp_4 = (i + 1) << 2;
                    tmp_5 = i << 2;
                    _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                      (_mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 1], aBuffer[tmp_5 +
                      1]), _mm_set1_pd(e)), _mm_mul_pd(_mm_set_pd(aBuffer[tmp_4],
                      aBuffer[tmp_5]), _mm_set1_pd(c_0))), _mm_mul_pd(_mm_set_pd
                      (aBuffer[tmp_4 + 2], aBuffer[tmp_5 + 2]), _mm_set1_pd(c_1))),
                      _mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 3], aBuffer[tmp_5 +
                      3]), _mm_set1_pd(c_2))));
                    cBuffer[nb + tmp_5] = tmp_3[0];
                    cBuffer[nb + tmp_4] = tmp_3[1];
                  }
                }
              }
            } else if (cBufferInUse) {
              for (nb = 0; nb < 4; nb++) {
                ed2 = cBuffer[nb + 4];
                e = cBuffer[nb];
                c_0 = cBuffer[nb + 8];
                c_1 = cBuffer[nb + 12];
                for (int32_T i{0}; i <= 2; i += 2) {
                  tmp_4 = (i + 1) << 2;
                  tmp_5 = i << 2;
                  _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                    (_mm_mul_pd(_mm_set_pd(a_0[tmp_4 + 1], a_0[tmp_5 + 1]),
                                _mm_set1_pd(ed2)), _mm_mul_pd(_mm_set_pd
                    (a_0[tmp_4], a_0[tmp_5]), _mm_set1_pd(e))), _mm_mul_pd
                    (_mm_set_pd(a_0[tmp_4 + 2], a_0[tmp_5 + 2]), _mm_set1_pd(c_0))),
                    _mm_mul_pd(_mm_set_pd(a_0[tmp_4 + 3], a_0[tmp_5 + 3]),
                               _mm_set1_pd(c_1))));
                  c[nb + tmp_5] = tmp_3[0];
                  c[nb + tmp_4] = tmp_3[1];
                }
              }
            } else {
              for (nb = 0; nb < 4; nb++) {
                e = c[nb + 4];
                c_0 = c[nb];
                c_1 = c[nb + 8];
                c_2 = c[nb + 12];
                for (int32_T i{0}; i <= 2; i += 2) {
                  tmp_4 = (i + 1) << 2;
                  tmp_5 = i << 2;
                  _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                    (_mm_mul_pd(_mm_set_pd(a_0[tmp_4 + 1], a_0[tmp_5 + 1]),
                                _mm_set1_pd(e)), _mm_mul_pd(_mm_set_pd(a_0[tmp_4],
                    a_0[tmp_5]), _mm_set1_pd(c_0))), _mm_mul_pd(_mm_set_pd
                    (a_0[tmp_4 + 2], a_0[tmp_5 + 2]), _mm_set1_pd(c_1))),
                    _mm_mul_pd(_mm_set_pd(a_0[tmp_4 + 3], a_0[tmp_5 + 3]),
                               _mm_set1_pd(c_2))));
                  cBuffer[nb + tmp_5] = tmp_3[0];
                  cBuffer[nb + tmp_4] = tmp_3[1];
                }
              }
            }

            cBufferInUse = !cBufferInUse;
          }
        }

        n >>= 1;
        if (aBufferInUse) {
          for (nb = 0; nb < 4; nb++) {
            for (int32_T i{0}; i <= 2; i += 2) {
              tmp_4 = (i + 1) << 2;
              tmp_5 = i << 2;
              _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                (_mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 1], aBuffer[tmp_5 + 1]),
                            _mm_set1_pd(aBuffer[nb + 4])), _mm_mul_pd(_mm_set_pd
                (aBuffer[tmp_4], aBuffer[tmp_5]), _mm_set1_pd(aBuffer[nb]))),
                _mm_mul_pd(_mm_set_pd(aBuffer[tmp_4 + 2], aBuffer[tmp_5 + 2]),
                           _mm_set1_pd(aBuffer[nb + 8]))), _mm_mul_pd(_mm_set_pd
                (aBuffer[tmp_4 + 3], aBuffer[tmp_5 + 3]), _mm_set1_pd(aBuffer[nb
                + 12]))));
              a_0[nb + tmp_5] = tmp_3[0];
              a_0[nb + tmp_4] = tmp_3[1];
            }
          }
        } else {
          for (nb = 0; nb < 4; nb++) {
            for (int32_T i{0}; i <= 2; i += 2) {
              tmp_4 = (i + 1) << 2;
              tmp_5 = i << 2;
              _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                (_mm_mul_pd(_mm_set_pd(a_0[tmp_4 + 1], a_0[tmp_5 + 1]),
                            _mm_set1_pd(a_0[nb + 4])), _mm_mul_pd(_mm_set_pd
                (a_0[tmp_4], a_0[tmp_5]), _mm_set1_pd(a_0[nb]))), _mm_mul_pd
                (_mm_set_pd(a_0[tmp_4 + 2], a_0[tmp_5 + 2]), _mm_set1_pd(a_0[nb
                + 8]))), _mm_mul_pd(_mm_set_pd(a_0[tmp_4 + 3], a_0[tmp_5 + 3]),
                                    _mm_set1_pd(a_0[nb + 12]))));
              aBuffer[nb + tmp_5] = tmp_3[0];
              aBuffer[nb + tmp_4] = tmp_3[1];
            }
          }
        }

        aBufferInUse = !aBufferInUse;
      }

      for (nb = 0; nb < 4; nb++) {
        real_T a_1;
        real_T a_2;
        real_T a_3;
        n = nb << 2;
        ed2 = aBuffer[n];
        e = aBuffer[n + 1];
        c_0 = aBuffer[n + 2];
        c_1 = aBuffer[n + 3];
        c_2 = a_0[n];
        a_1 = a_0[n + 1];
        a_2 = a_0[n + 2];
        a_3 = a_0[n + 3];
        for (int32_T i{0}; i <= 2; i += 2) {
          __m128d tmp;
          __m128d tmp_0;
          __m128d tmp_1;
          __m128d tmp_2;
          tmp = _mm_loadu_pd(&cBuffer[i]);
          tmp_0 = _mm_loadu_pd(&cBuffer[i + 4]);
          tmp_1 = _mm_loadu_pd(&cBuffer[i + 8]);
          tmp_2 = _mm_loadu_pd(&cBuffer[i + 12]);
          tmp_4 = i + n;
          _mm_storeu_pd(&cBuffer_1[tmp_4], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(a_3),
            tmp_2), _mm_add_pd(_mm_mul_pd(_mm_set1_pd(a_2), tmp_1), _mm_add_pd
                               (_mm_mul_pd(_mm_set1_pd(a_1), tmp_0), _mm_mul_pd
                                (_mm_set1_pd(c_2), tmp)))));
          _mm_storeu_pd(&cBuffer_0[tmp_4], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(c_1),
            tmp_2), _mm_add_pd(_mm_mul_pd(_mm_set1_pd(c_0), tmp_1), _mm_add_pd
                               (_mm_mul_pd(_mm_set1_pd(e), tmp_0), _mm_mul_pd
                                (_mm_set1_pd(ed2), tmp)))));
        }
      }

      for (nb = 0; nb < 16; nb++) {
        if (firstmult) {
          if (aBufferInUse) {
            c[nb] = aBuffer[nb];
          } else {
            c[nb] = a_0[nb];
          }
        } else if (aBufferInUse) {
          c[nb] = cBuffer_0[nb];
        } else {
          c[nb] = cBuffer_1[nb];
        }
      }
    }
  } else {
    std::memcpy(&a_0[0], &a[0], sizeof(real_T) << 4U);
    if ((!std::isinf(b)) && (!std::isnan(b))) {
      boolean_T firstmult;
      firstmult = true;
      real_T ed2;
      int32_T exitg1;
      do {
        int32_T tmp_4;
        int32_T tmp_5;
        exitg1 = 0;
        ed2 = std::floor(e / 2.0);
        if (2.0 * ed2 != e) {
          if (firstmult) {
            std::memcpy(&c[0], &a_0[0], sizeof(real_T) << 4U);
            firstmult = false;
          } else {
            for (int32_T nb{0}; nb < 4; nb++) {
              real_T c_0;
              real_T c_1;
              real_T c_2;
              e = c[nb + 4];
              c_0 = c[nb];
              c_1 = c[nb + 8];
              c_2 = c[nb + 12];
              for (int32_T i{0}; i <= 2; i += 2) {
                tmp_4 = (i + 1) << 2;
                tmp_5 = i << 2;
                _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                  (_mm_mul_pd(_mm_set_pd(a_0[tmp_4 + 1], a_0[tmp_5 + 1]),
                              _mm_set1_pd(e)), _mm_mul_pd(_mm_set_pd(a_0[tmp_4],
                  a_0[tmp_5]), _mm_set1_pd(c_0))), _mm_mul_pd(_mm_set_pd
                  (a_0[tmp_4 + 2], a_0[tmp_5 + 2]), _mm_set1_pd(c_1))),
                  _mm_mul_pd(_mm_set_pd(a_0[tmp_4 + 3], a_0[tmp_5 + 3]),
                             _mm_set1_pd(c_2))));
                cBuffer[nb + tmp_5] = tmp_3[0];
                cBuffer[nb + tmp_4] = tmp_3[1];
              }
            }

            std::memcpy(&c[0], &cBuffer[0], sizeof(real_T) << 4U);
          }
        }

        if (ed2 == 0.0) {
          exitg1 = 1;
        } else {
          e = ed2;
          for (int32_T nb{0}; nb < 4; nb++) {
            for (int32_T i{0}; i <= 2; i += 2) {
              tmp_4 = (i + 1) << 2;
              tmp_5 = i << 2;
              _mm_storeu_pd(&tmp_3[0], _mm_add_pd(_mm_add_pd(_mm_add_pd
                (_mm_mul_pd(_mm_set_pd(a_0[tmp_4 + 1], a_0[tmp_5 + 1]),
                            _mm_set1_pd(a_0[nb + 4])), _mm_mul_pd(_mm_set_pd
                (a_0[tmp_4], a_0[tmp_5]), _mm_set1_pd(a_0[nb]))), _mm_mul_pd
                (_mm_set_pd(a_0[tmp_4 + 2], a_0[tmp_5 + 2]), _mm_set1_pd(a_0[nb
                + 8]))), _mm_mul_pd(_mm_set_pd(a_0[tmp_4 + 3], a_0[tmp_5 + 3]),
                                    _mm_set1_pd(a_0[nb + 12]))));
              cBuffer[nb + tmp_5] = tmp_3[0];
              cBuffer[nb + tmp_4] = tmp_3[1];
            }
          }

          std::memcpy(&a_0[0], &cBuffer[0], sizeof(real_T) << 4U);
        }
      } while (exitg1 == 0);
    } else {
      for (int32_T nb{0}; nb < 16; nb++) {
        c[nb] = (rtNaN);
      }
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

real_T longitudinal_mpc::longitudinal_mpc_log2(real_T x)
{
  real_T f;
  int32_T eint;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (x == 0.0) {
    f = (rtMinusInf);
  } else if ((!std::isinf(x)) && (!std::isnan(x))) {
    real_T t;
    t = std::frexp(x, &eint);
    if (t == 0.5) {
      f = static_cast<real_T>(eint) - 1.0;
    } else if ((eint == 1) && (t < 0.75)) {
      f = std::log(2.0 * t) / 0.69314718055994529;
    } else {
      f = std::log(t) / 0.69314718055994529 + static_cast<real_T>(eint);
    }
  } else {
    f = x;
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
  return f;
}

void longitudinal_mpc::longitudinal_mpc_norminv(const coder::array<real_T, 1U>
  &mu, const coder::array<real_T, 1U> &sigma, real_T x[100])
{
  real_T R;
  real_T absx;
  real_T s;
  real_T sigma_0;
  real_T x_0;
  real_T y;
  real_T z;
  int32_T b_e;
  int32_T b_k;
  int32_T eint;
  int32_T eint_0;
  static const real_T p[100]{ 0.9999, 0.99485151515151515, 0.9898030303030303,
    0.98475454545454544, 0.97970606060606058, 0.97465757575757572,
    0.96960909090909086, 0.96456060606060612, 0.95951212121212126,
    0.9544636363636364, 0.94941515151515155, 0.94436666666666669,
    0.93931818181818183, 0.934269696969697, 0.92922121212121211,
    0.92417272727272726, 0.9191242424242424, 0.91407575757575754,
    0.90902727272727268, 0.90397878787878794, 0.89893030303030308,
    0.89388181818181822, 0.88883333333333336, 0.88378484848484851,
    0.87873636363636365, 0.87368787878787879, 0.86863939393939393,
    0.86359090909090908, 0.85854242424242422, 0.85349393939393936,
    0.8484454545454545, 0.84339696969696965, 0.83834848484848479,
    0.83329999999999993, 0.82825151515151518, 0.82320303030303033,
    0.81815454545454547, 0.81310606060606061, 0.80805757575757575,
    0.80300909090909089, 0.797960606060606, 0.79291212121212118,
    0.78786363636363632, 0.78281515151515146, 0.77776666666666672,
    0.77271818181818186, 0.767669696969697, 0.76262121212121214,
    0.75757272727272729, 0.75252424242424243, 0.74747575757575757,
    0.74242727272727271, 0.73737878787878786, 0.732330303030303,
    0.72728181818181814, 0.72223333333333328, 0.71718484848484843,
    0.71213636363636357, 0.70708787878787871, 0.70203939393939385,
    0.696990909090909, 0.69194242424242425, 0.68689393939393939,
    0.68184545454545453, 0.67679696969696967, 0.67174848484848482, 0.6667,
    0.6616515151515151, 0.65660303030303035, 0.6515545454545455,
    0.64650606060606064, 0.64145757575757578, 0.63640909090909092,
    0.63136060606060607, 0.62631212121212121, 0.62126363636363635,
    0.61621515151515149, 0.61116666666666664, 0.60611818181818178,
    0.60106969696969692, 0.59602121212121206, 0.59097272727272721,
    0.58592424242424235, 0.58087575757575749, 0.57582727272727263,
    0.57077878787878777, 0.56573030303030292, 0.56068181818181817,
    0.55563333333333331, 0.55058484848484845, 0.5455363636363636,
    0.54048787878787874, 0.53543939393939388, 0.53039090909090914,
    0.52534242424242428, 0.52029393939393942, 0.51524545454545456,
    0.5101969696969697, 0.50514848484848485, 0.5001 };

  for (b_k = 0; b_k < 100; b_k++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    sigma_0 = sigma[b_k];
    if (sigma_0 > 0.0) {
      y = 2.0 * p[b_k];
      if (y > 1.7) {
        z = std::sqrt(-std::log((2.0 - y) / 2.0));
        x_0 = -(((1.641345311 * z + 3.429567803) * z - 1.624906493) * z -
                1.970840454) / ((1.6370678 * z + 3.5438892) * z + 1.0);

        /* ========================== COPYRIGHT NOTICE ============================ */
        /*  The algorithms for calculating ERF(X) and ERFC(X) are derived           */
        /*  from FDLIBM, which has the following notice:                            */
        /*                                                                          */
        /*  Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.       */
        /*                                                                          */
        /*  Developed at SunSoft, a Sun Microsystems, Inc. business.                */
        /*  Permission to use, copy, modify, and distribute this                    */
        /*  software is freely granted, provided that this notice                   */
        /*  is preserved.                                                           */
        /* =============================    END    ================================ */
        if (-x_0 < 0.84375) {
          z = -x_0 * -x_0;
          if (-x_0 < 0.25) {
            absx = 1.0 - (((((z * -2.3763016656650163E-5 - 0.0057702702964894416)
                             * z - 0.02848174957559851) * z - 0.3250421072470015)
                           * z + 0.12837916709551256) / (((((z *
              -3.9602282787753681E-6 + 0.00013249473800432164) * z +
              0.0050813062818757656) * z + 0.0650222499887673) * z +
              0.39791722395915535) * z + 1.0) * -x_0 - x_0);
          } else {
            absx = 0.5 - (((((z * -2.3763016656650163E-5 - 0.0057702702964894416)
                             * z - 0.02848174957559851) * z - 0.3250421072470015)
                           * z + 0.12837916709551256) / (((((z *
              -3.9602282787753681E-6 + 0.00013249473800432164) * z +
              0.0050813062818757656) * z + 0.0650222499887673) * z +
              0.39791722395915535) * z + 1.0) * -x_0 + (-x_0 - 0.5));
          }
        } else if (-x_0 < 1.25) {
          absx = 0.15493708848953247 - (((((((-x_0 - 1.0) *
            -0.0021663755948687908 + 0.035478304325618236) * (-x_0 - 1.0) -
            0.11089469428239668) * (-x_0 - 1.0) + 0.31834661990116175) * (-x_0 -
            1.0) - 0.37220787603570132) * (-x_0 - 1.0) + 0.41485611868374833) *
            (-x_0 - 1.0) - 0.0023621185607526594) / (((((((-x_0 - 1.0) *
            0.011984499846799107 + 0.013637083912029051) * (-x_0 - 1.0) +
            0.12617121980876164) * (-x_0 - 1.0) + 0.071828654414196266) * (-x_0
            - 1.0) + 0.540397917702171) * (-x_0 - 1.0) + 0.10642088040084423) *
            (-x_0 - 1.0) + 1.0);
        } else {
          s = 1.0 / (-x_0 * -x_0);
          if (-x_0 < 2.8571414947509766) {
            R = ((((((s * -9.8143293441691455 - 81.2874355063066) * s -
                     184.60509290671104) * s - 162.39666946257347) * s -
                   62.375332450326006) * s - 10.558626225323291) * s -
                 0.69385857270718176) * s - 0.0098649440348471482;
            s = (((((((s * -0.0604244152148581 + 6.5702497703192817) * s +
                      108.63500554177944) * s + 429.00814002756783) * s +
                    645.38727173326788) * s + 434.56587747522923) * s +
                  137.65775414351904) * s + 19.651271667439257) * s + 1.0;
          } else {
            R = (((((s * -483.5191916086514 - 1025.0951316110772) * s -
                    637.56644336838963) * s - 160.63638485582192) * s -
                  17.757954917754752) * s - 0.799283237680523) * s -
              0.0098649429247001;
            s = ((((((s * -22.440952446585818 + 474.52854120695537) * s +
                     2553.0504064331644) * s + 3199.8582195085955) * s +
                   1536.729586084437) * s + 325.79251299657392) * s +
                 30.338060743482458) * s + 1.0;
          }

          z = std::frexp(-x_0, &eint_0);
          z = std::floor(z * 2.097152E+6) / 2.097152E+6 * rt_powd_snf(2.0,
            static_cast<real_T>(eint_0));
          absx = std::exp((z - (-x_0)) * (z - x_0) + R / s) * std::exp(-z * z -
            0.5625) / -x_0;
        }

        absx = (absx - (2.0 - y)) / (std::exp(-x_0 * x_0) * 1.1283791670955126);
        x_0 -= absx / (x_0 * absx + 1.0);

        /* ========================== COPYRIGHT NOTICE ============================ */
        /*  The algorithms for calculating ERF(X) and ERFC(X) are derived           */
        /*  from FDLIBM, which has the following notice:                            */
        /*                                                                          */
        /*  Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.       */
        /*                                                                          */
        /*  Developed at SunSoft, a Sun Microsystems, Inc. business.                */
        /*  Permission to use, copy, modify, and distribute this                    */
        /*  software is freely granted, provided that this notice                   */
        /*  is preserved.                                                           */
        /* =============================    END    ================================ */
        absx = std::abs(-x_0);
        if (std::isnan(-x_0)) {
          absx = -x_0;
        } else if (std::isinf(-x_0)) {
          if (-x_0 < 0.0) {
            absx = 2.0;
          } else {
            absx = 0.0;
          }
        } else if (absx < 0.84375) {
          if (absx < 1.3877787807814457E-17) {
            absx = 1.0 - (-x_0);
          } else {
            z = -x_0 * -x_0;
            if (-x_0 < 0.25) {
              absx = 1.0 - (((((z * -2.3763016656650163E-5 -
                                0.0057702702964894416) * z - 0.02848174957559851)
                              * z - 0.3250421072470015) * z +
                             0.12837916709551256) / (((((z *
                -3.9602282787753681E-6 + 0.00013249473800432164) * z +
                0.0050813062818757656) * z + 0.0650222499887673) * z +
                0.39791722395915535) * z + 1.0) * -x_0 - x_0);
            } else {
              absx = 0.5 - (((((z * -2.3763016656650163E-5 -
                                0.0057702702964894416) * z - 0.02848174957559851)
                              * z - 0.3250421072470015) * z +
                             0.12837916709551256) / (((((z *
                -3.9602282787753681E-6 + 0.00013249473800432164) * z +
                0.0050813062818757656) * z + 0.0650222499887673) * z +
                0.39791722395915535) * z + 1.0) * -x_0 + (-x_0 - 0.5));
            }
          }
        } else if (absx < 1.25) {
          if (-x_0 >= 0.0) {
            absx = 0.15493708848953247 - (((((((absx - 1.0) *
              -0.0021663755948687908 + 0.035478304325618236) * (absx - 1.0) -
              0.11089469428239668) * (absx - 1.0) + 0.31834661990116175) * (absx
              - 1.0) - 0.37220787603570132) * (absx - 1.0) + 0.41485611868374833)
              * (absx - 1.0) - 0.0023621185607526594) / (((((((absx - 1.0) *
              0.011984499846799107 + 0.013637083912029051) * (absx - 1.0) +
              0.12617121980876164) * (absx - 1.0) + 0.071828654414196266) *
              (absx - 1.0) + 0.540397917702171) * (absx - 1.0) +
              0.10642088040084423) * (absx - 1.0) + 1.0);
          } else {
            absx = ((((((((absx - 1.0) * -0.0021663755948687908 +
                          0.035478304325618236) * (absx - 1.0) -
                         0.11089469428239668) * (absx - 1.0) +
                        0.31834661990116175) * (absx - 1.0) -
                       0.37220787603570132) * (absx - 1.0) + 0.41485611868374833)
                     * (absx - 1.0) - 0.0023621185607526594) / (((((((absx - 1.0)
              * 0.011984499846799107 + 0.013637083912029051) * (absx - 1.0) +
              0.12617121980876164) * (absx - 1.0) + 0.071828654414196266) *
                       (absx - 1.0) + 0.540397917702171) * (absx - 1.0) +
                      0.10642088040084423) * (absx - 1.0) + 1.0) +
                    0.84506291151046753) + 1.0;
          }
        } else if (-x_0 < -6.0) {
          absx = 2.0;
        } else if (-x_0 >= 28.0) {
          absx = 0.0;
        } else {
          s = 1.0 / (absx * absx);
          if (absx < 2.8571414947509766) {
            R = ((((((s * -9.8143293441691455 - 81.2874355063066) * s -
                     184.60509290671104) * s - 162.39666946257347) * s -
                   62.375332450326006) * s - 10.558626225323291) * s -
                 0.69385857270718176) * s - 0.0098649440348471482;
            s = (((((((s * -0.0604244152148581 + 6.5702497703192817) * s +
                      108.63500554177944) * s + 429.00814002756783) * s +
                    645.38727173326788) * s + 434.56587747522923) * s +
                  137.65775414351904) * s + 19.651271667439257) * s + 1.0;
          } else {
            R = (((((s * -483.5191916086514 - 1025.0951316110772) * s -
                    637.56644336838963) * s - 160.63638485582192) * s -
                  17.757954917754752) * s - 0.799283237680523) * s -
              0.0098649429247001;
            s = ((((((s * -22.440952446585818 + 474.52854120695537) * s +
                     2553.0504064331644) * s + 3199.8582195085955) * s +
                   1536.729586084437) * s + 325.79251299657392) * s +
                 30.338060743482458) * s + 1.0;
          }

          if ((!std::isinf(absx)) && (!std::isnan(absx))) {
            z = std::frexp(absx, &eint_0);
            b_e = eint_0;
          } else {
            z = absx;
            b_e = 0;
          }

          z = std::floor(z * 2.097152E+6) / 2.097152E+6 * rt_powd_snf(2.0,
            static_cast<real_T>(b_e));
          absx = std::exp((z - absx) * (z + absx) + R / s) * std::exp(-z * z -
            0.5625) / absx;
          if (-x_0 < 0.0) {
            absx = 2.0 - absx;
          }
        }

        absx = (absx - (2.0 - y)) / (std::exp(-x_0 * x_0) * 1.1283791670955126);
        x_0 -= absx / (x_0 * absx + 1.0);
      } else {
        z = (1.0 - y) * (1.0 - y);
        x_0 = (((-0.140543331 * z + 0.914624893) * z - 1.645349621) * z +
               0.886226899) * (1.0 - y) / ((((0.012229801 * z - 0.329097515) * z
          + 1.442710462) * z - 2.118377725) * z + 1.0);

        /* ========================== COPYRIGHT NOTICE ============================ */
        /*  The algorithms for calculating ERF(X) and ERFC(X) are derived           */
        /*  from FDLIBM, which has the following notice:                            */
        /*                                                                          */
        /*  Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.       */
        /*                                                                          */
        /*  Developed at SunSoft, a Sun Microsystems, Inc. business.                */
        /*  Permission to use, copy, modify, and distribute this                    */
        /*  software is freely granted, provided that this notice                   */
        /*  is preserved.                                                           */
        /* =============================    END    ================================ */
        absx = std::abs(x_0);
        if (std::isinf(x_0)) {
          if (x_0 < 0.0) {
            absx = -1.0;
          } else {
            absx = 1.0;
          }
        } else if (absx < 0.84375) {
          if (absx < 3.7252902984619141E-9) {
            if (absx < 2.8480945388892178E-306) {
              absx = (8.0 * x_0 + 1.0270333367641007 * x_0) * 0.125;
            } else {
              absx = 0.12837916709551259 * x_0 + x_0;
            }
          } else {
            z = x_0 * x_0;
            absx = ((((z * -2.3763016656650163E-5 - 0.0057702702964894416) * z -
                      0.02848174957559851) * z - 0.3250421072470015) * z +
                    0.12837916709551256) / (((((z * -3.9602282787753681E-6 +
              0.00013249473800432164) * z + 0.0050813062818757656) * z +
              0.0650222499887673) * z + 0.39791722395915535) * z + 1.0) * x_0 +
              x_0;
          }
        } else if (absx < 1.25) {
          if (x_0 >= 0.0) {
            absx = (((((((absx - 1.0) * -0.0021663755948687908 +
                         0.035478304325618236) * (absx - 1.0) -
                        0.11089469428239668) * (absx - 1.0) +
                       0.31834661990116175) * (absx - 1.0) - 0.37220787603570132)
                     * (absx - 1.0) + 0.41485611868374833) * (absx - 1.0) -
                    0.0023621185607526594) / (((((((absx - 1.0) *
              0.011984499846799107 + 0.013637083912029051) * (absx - 1.0) +
              0.12617121980876164) * (absx - 1.0) + 0.071828654414196266) *
              (absx - 1.0) + 0.540397917702171) * (absx - 1.0) +
              0.10642088040084423) * (absx - 1.0) + 1.0) + 0.84506291151046753;
          } else {
            absx = -0.84506291151046753 - (((((((absx - 1.0) *
              -0.0021663755948687908 + 0.035478304325618236) * (absx - 1.0) -
              0.11089469428239668) * (absx - 1.0) + 0.31834661990116175) * (absx
              - 1.0) - 0.37220787603570132) * (absx - 1.0) + 0.41485611868374833)
              * (absx - 1.0) - 0.0023621185607526594) / (((((((absx - 1.0) *
              0.011984499846799107 + 0.013637083912029051) * (absx - 1.0) +
              0.12617121980876164) * (absx - 1.0) + 0.071828654414196266) *
              (absx - 1.0) + 0.540397917702171) * (absx - 1.0) +
              0.10642088040084423) * (absx - 1.0) + 1.0);
          }
        } else if (absx > 6.0) {
          if (x_0 < 0.0) {
            absx = -1.0;
          } else {
            absx = 1.0;
          }
        } else {
          s = 1.0 / (absx * absx);
          if (absx < 2.8571434020996094) {
            R = ((((((s * -9.8143293441691455 - 81.2874355063066) * s -
                     184.60509290671104) * s - 162.39666946257347) * s -
                   62.375332450326006) * s - 10.558626225323291) * s -
                 0.69385857270718176) * s - 0.0098649440348471482;
            s = (((((((s * -0.0604244152148581 + 6.5702497703192817) * s +
                      108.63500554177944) * s + 429.00814002756783) * s +
                    645.38727173326788) * s + 434.56587747522923) * s +
                  137.65775414351904) * s + 19.651271667439257) * s + 1.0;
          } else {
            R = (((((s * -483.5191916086514 - 1025.0951316110772) * s -
                    637.56644336838963) * s - 160.63638485582192) * s -
                  17.757954917754752) * s - 0.799283237680523) * s -
              0.0098649429247001;
            s = ((((((s * -22.440952446585818 + 474.52854120695537) * s +
                     2553.0504064331644) * s + 3199.8582195085955) * s +
                   1536.729586084437) * s + 325.79251299657392) * s +
                 30.338060743482458) * s + 1.0;
          }

          z = std::frexp(absx, &eint);
          z = std::floor(z * 2.097152E+6) / 2.097152E+6 * rt_powd_snf(2.0,
            static_cast<real_T>(eint));
          absx = std::exp((z - absx) * (z + absx) + R / s) * std::exp(-z * z -
            0.5625) / absx;
          if (x_0 < 0.0) {
            absx--;
          } else {
            absx = 1.0 - absx;
          }
        }

        absx = (absx - (1.0 - y)) / (std::exp(-x_0 * x_0) * 1.1283791670955126);
        x_0 -= absx / (x_0 * absx + 1.0);

        /* ========================== COPYRIGHT NOTICE ============================ */
        /*  The algorithms for calculating ERF(X) and ERFC(X) are derived           */
        /*  from FDLIBM, which has the following notice:                            */
        /*                                                                          */
        /*  Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.       */
        /*                                                                          */
        /*  Developed at SunSoft, a Sun Microsystems, Inc. business.                */
        /*  Permission to use, copy, modify, and distribute this                    */
        /*  software is freely granted, provided that this notice                   */
        /*  is preserved.                                                           */
        /* =============================    END    ================================ */
        absx = std::abs(x_0);
        if (std::isnan(x_0)) {
          absx = (rtNaN);
        } else if (std::isinf(x_0)) {
          if (x_0 < 0.0) {
            absx = -1.0;
          } else {
            absx = 1.0;
          }
        } else if (absx < 0.84375) {
          if (absx < 3.7252902984619141E-9) {
            if (absx < 2.8480945388892178E-306) {
              absx = (8.0 * x_0 + 1.0270333367641007 * x_0) * 0.125;
            } else {
              absx = 0.12837916709551259 * x_0 + x_0;
            }
          } else {
            z = x_0 * x_0;
            absx = ((((z * -2.3763016656650163E-5 - 0.0057702702964894416) * z -
                      0.02848174957559851) * z - 0.3250421072470015) * z +
                    0.12837916709551256) / (((((z * -3.9602282787753681E-6 +
              0.00013249473800432164) * z + 0.0050813062818757656) * z +
              0.0650222499887673) * z + 0.39791722395915535) * z + 1.0) * x_0 +
              x_0;
          }
        } else if (absx < 1.25) {
          if (x_0 >= 0.0) {
            absx = (((((((absx - 1.0) * -0.0021663755948687908 +
                         0.035478304325618236) * (absx - 1.0) -
                        0.11089469428239668) * (absx - 1.0) +
                       0.31834661990116175) * (absx - 1.0) - 0.37220787603570132)
                     * (absx - 1.0) + 0.41485611868374833) * (absx - 1.0) -
                    0.0023621185607526594) / (((((((absx - 1.0) *
              0.011984499846799107 + 0.013637083912029051) * (absx - 1.0) +
              0.12617121980876164) * (absx - 1.0) + 0.071828654414196266) *
              (absx - 1.0) + 0.540397917702171) * (absx - 1.0) +
              0.10642088040084423) * (absx - 1.0) + 1.0) + 0.84506291151046753;
          } else {
            absx = -0.84506291151046753 - (((((((absx - 1.0) *
              -0.0021663755948687908 + 0.035478304325618236) * (absx - 1.0) -
              0.11089469428239668) * (absx - 1.0) + 0.31834661990116175) * (absx
              - 1.0) - 0.37220787603570132) * (absx - 1.0) + 0.41485611868374833)
              * (absx - 1.0) - 0.0023621185607526594) / (((((((absx - 1.0) *
              0.011984499846799107 + 0.013637083912029051) * (absx - 1.0) +
              0.12617121980876164) * (absx - 1.0) + 0.071828654414196266) *
              (absx - 1.0) + 0.540397917702171) * (absx - 1.0) +
              0.10642088040084423) * (absx - 1.0) + 1.0);
          }
        } else if (absx > 6.0) {
          if (x_0 < 0.0) {
            absx = -1.0;
          } else {
            absx = 1.0;
          }
        } else {
          s = 1.0 / (absx * absx);
          if (absx < 2.8571434020996094) {
            R = ((((((s * -9.8143293441691455 - 81.2874355063066) * s -
                     184.60509290671104) * s - 162.39666946257347) * s -
                   62.375332450326006) * s - 10.558626225323291) * s -
                 0.69385857270718176) * s - 0.0098649440348471482;
            s = (((((((s * -0.0604244152148581 + 6.5702497703192817) * s +
                      108.63500554177944) * s + 429.00814002756783) * s +
                    645.38727173326788) * s + 434.56587747522923) * s +
                  137.65775414351904) * s + 19.651271667439257) * s + 1.0;
          } else {
            R = (((((s * -483.5191916086514 - 1025.0951316110772) * s -
                    637.56644336838963) * s - 160.63638485582192) * s -
                  17.757954917754752) * s - 0.799283237680523) * s -
              0.0098649429247001;
            s = ((((((s * -22.440952446585818 + 474.52854120695537) * s +
                     2553.0504064331644) * s + 3199.8582195085955) * s +
                   1536.729586084437) * s + 325.79251299657392) * s +
                 30.338060743482458) * s + 1.0;
          }

          if (!std::isnan(absx)) {
            z = std::frexp(absx, &eint);
            b_e = eint;
          } else {
            z = (rtNaN);
            b_e = 0;
          }

          z = std::floor(z * 2.097152E+6) / 2.097152E+6 * rt_powd_snf(2.0,
            static_cast<real_T>(b_e));
          absx = std::exp((z - absx) * (z + absx) + R / s) * std::exp(-z * z -
            0.5625) / absx;
          if (x_0 < 0.0) {
            absx--;
          } else {
            absx = 1.0 - absx;
          }
        }

        absx = (absx - (1.0 - y)) / (std::exp(-x_0 * x_0) * 1.1283791670955126);
        x_0 -= absx / (x_0 * absx + 1.0);
      }

      x[b_k] = -1.4142135623730951 * x_0 * sigma_0 + mu[b_k];
    } else {
      x[b_k] = (rtNaN);
    }

    /* End of Start for MATLABSystem: '<S1>/MPC System' */
  }
}

void longitudinal_mpc::longitudinal_m_PCCMPC_getChance(real_T s_p[100], real_T
  v_p[100])
{
  coder::array<real_T, 1U> b_x;
  real_T PhiP[400];
  real_T GammaB[200];
  int32_T b;
  int32_T b_k;
  int32_T c;
  int32_T d;
  int32_T kidx;
  static const real_T tmp[4]{ 1.0, 0.0, 0.1, 1.0 };

  __m128d tmp_1;
  coder::array<real_T, 1U> Mean;
  real_T PhiP_data[400];
  real_T PhiP_data_0[396];
  real_T tmp_0[4];
  real_T Gamma;
  real_T PhiP_0;
  int32_T c_tmp;
  int32_T i;
  int32_T loop_ub;
  int16_T h[2];
  int16_T h_0;
  int8_T A;
  static const int8_T A_0[10000]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
  };

  static const real_T tmp_2[4]{ 1.0, 0.0, 0.1, 1.0 };

  static const int8_T B[4]{ 1, 0, 0, 0 };

  static const int8_T B_0[4]{ 0, 0, 0, 1 };

  __m128d tmp_3;

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* %% Tyler Ard                %%% */
  /* %% Argonne National Lab     %%% */
  /* %% Vehicle Mobility Systems %%% */
  /* %% tard(at)anl.gov          %%% */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  GETCHANCE */
  /* %% Prediction model */
  /*  \hat{x} = Ax + Bu */
  /*  u = \bar{u} + \var{u} */
  /*  [s] */
  /* %% Forward stages */
  /*  Parameters */
  /*  Model forward */
  std::memset(&PhiP[0], 0, 400U * sizeof(real_T));
  PhiP[0] = 1.0;
  PhiP[201] = 1.0;
  for (b_k = 0; b_k < 99; b_k++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b = (b_k << 1) + 1;
    c_tmp = (b_k + 1) << 1;
    if (b > c_tmp) {
      b = 0;
      d = 0;
    } else {
      b--;
      d = c_tmp;
    }

    c = (b_k + 2) << 1;
    if (c_tmp + 1 > c) {
      kidx = 0;
      c = 0;
    } else {
      kidx = c_tmp;
    }

    loop_ub = d - b;
    for (c_tmp = 0; c_tmp < 2; c_tmp++) {
      for (i = 0; i < loop_ub; i++) {
        PhiP_data_0[i + loop_ub * c_tmp] = PhiP[(b + i) + 200 * c_tmp];
      }
    }

    for (c_tmp = 0; c_tmp <= 0; c_tmp += 2) {
      tmp_1 = _mm_set_pd(static_cast<real_T>(static_cast<int32_T>(tmp_2[c_tmp +
        1])), static_cast<real_T>(static_cast<int32_T>(tmp_2[c_tmp])));
      tmp_3 = _mm_loadu_pd(&tmp[c_tmp + 2]);
      _mm_storeu_pd(&tmp_0[c_tmp], _mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (PhiP_data_0[0]), tmp_1), _mm_mul_pd(tmp_3, _mm_set1_pd(PhiP_data_0[1]))));
      _mm_storeu_pd(&tmp_0[c_tmp + 2], _mm_add_pd(_mm_mul_pd(_mm_set1_pd
        (PhiP_data_0[2]), tmp_1), _mm_mul_pd(tmp_3, _mm_set1_pd(PhiP_data_0[3]))));
    }

    h_0 = static_cast<int16_T>(c - kidx);
    h[0] = h_0;
    loop_ub = h_0;
    for (c_tmp = 0; c_tmp < 2; c_tmp++) {
      for (i = 0; i < loop_ub; i++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        PhiP[(kidx + i) + 200 * c_tmp] = tmp_0[h[0] * c_tmp + i];
      }
    }
  }

  std::memset(&longitudinal_mpc_B.GammaP[0], 0, 40000U * sizeof(real_T));
  for (b_k = 0; b_k < 200; b_k++) {
    longitudinal_mpc_B.GammaP[b_k + 200 * b_k] = 1.0;
  }

  for (kidx = 0; kidx < 99; kidx++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_k = (kidx << 1) + 1;
    c = (kidx + 1) << 1;
    if (b_k > c) {
      b = 0;
      d = 0;
    } else {
      b = b_k - 1;
      d = c;
    }

    loop_ub = (100 - kidx) << 1;
    for (c_tmp = 0; c_tmp < 2; c_tmp++) {
      for (i = 0; i < loop_ub; i++) {
        PhiP_data[i + loop_ub * c_tmp] = PhiP[200 * c_tmp + i];
      }
    }

    h[0] = static_cast<int16_T>(201 - b_k);
    h[1] = static_cast<int16_T>(d - b);
    loop_ub = h[1];
    for (c_tmp = 0; c_tmp < loop_ub; c_tmp++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      d = 201 - b_k;
      for (i = 0; i < d; i++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        longitudinal_mpc_B.GammaP[((b_k + i) + 200 * (b + c_tmp)) - 1] =
          PhiP_data[h[0] * c_tmp + i];
      }
    }
  }

  kidx = -1;
  for (b_k = 0; b_k < 100; b_k++) {
    for (d = 0; d < 100; d++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      A = A_0[100 * b_k + d];
      longitudinal_mpc_B.b[kidx + 1] = static_cast<real_T>(A) *
        0.005000000000000001;
      longitudinal_mpc_B.b[kidx + 2] = static_cast<real_T>(A) * 0.1;
      kidx += 2;
    }
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Mean */
  for (c_tmp = 0; c_tmp < 200; c_tmp++) {
    for (i = 0; i < 100; i++) {
      Gamma = 0.0;
      for (b = 0; b < 200; b++) {
        Gamma += longitudinal_mpc_B.GammaP[200 * b + c_tmp] *
          longitudinal_mpc_B.b[200 * i + b];
      }

      longitudinal_mpc_B.Gamma[c_tmp + 200 * i] = Gamma;
    }

    Gamma = PhiP[c_tmp + 200];
    PhiP_0 = PhiP[c_tmp];
    GammaB[c_tmp] = (PhiP_0 * 0.1 + Gamma) * 0.0 + (Gamma * 0.0 + PhiP_0) * 0.0;
  }

  for (c_tmp = 0; c_tmp < 100; c_tmp++) {
    for (i = 0; i <= 198; i += 2) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      b = 200 * c_tmp + i;
      tmp_1 = _mm_loadu_pd(&longitudinal_mpc_B.Gamma[b]);
      tmp_3 = _mm_loadu_pd(&GammaB[i]);

      /* Start for MATLABSystem: '<S1>/MPC System' */
      _mm_storeu_pd(&longitudinal_mpc_B.Mean[b], _mm_add_pd(_mm_mul_pd(tmp_1,
        _mm_set1_pd(0.0)), tmp_3));
    }
  }

  /*  Covariance */
  for (c_tmp = 0; c_tmp < 200; c_tmp++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    Gamma = 0.0;
    for (i = 0; i < 100; i++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      Gamma += longitudinal_mpc_B.Gamma[200 * i + c_tmp] * 0.94339811320566036;
    }

    /* Start for MATLABSystem: '<S1>/MPC System' */
    GammaB[c_tmp] = Gamma;
  }

  for (c_tmp = 0; c_tmp < 200; c_tmp++) {
    for (i = 0; i <= 198; i += 2) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      tmp_1 = _mm_loadu_pd(&GammaB[i]);
      _mm_storeu_pd(&longitudinal_mpc_B.Cov[i + 200 * c_tmp], _mm_mul_pd(tmp_1,
        _mm_set1_pd(GammaB[c_tmp])));
    }
  }

  /* %% Uncertainty */
  kidx = -1;
  for (b_k = 0; b_k < 100; b_k++) {
    for (d = 0; d < 100; d++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      longitudinal_mpc_B.b[kidx + 1] = A_0[100 * b_k + d];
      longitudinal_mpc_B.b[kidx + 2] = 0.0;
      kidx += 2;
    }
  }

  kidx = -1;
  for (b_k = 0; b_k < 100; b_k++) {
    for (b = 0; b < 2; b++) {
      for (d = 0; d < 100; d++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        longitudinal_mpc_B.GammaP[kidx + 1] = A_0[100 * b_k + d] * B[b << 1];
        longitudinal_mpc_B.GammaP[kidx + 2] = 0.0;
        kidx += 2;
      }
    }
  }

  b_k = 0;
  for (b = 0; b < 40000; b++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    if (longitudinal_mpc_B.GammaP[b] != 0.0) {
      b_k++;
    }
  }

  b_x.set_size(b_k);
  c_tmp = 0;
  b_k = 0;
  for (b = 0; b < 40000; b++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    if (longitudinal_mpc_B.GammaP[b] != 0.0) {
      b_x[c_tmp] = longitudinal_mpc_B.Cov[b];
      b_k = c_tmp + 1;
      c_tmp++;
    }
  }

  b = b_k - 1;
  c_tmp = (b_k / 2) << 1;
  d = c_tmp - 2;
  for (b_k = 0; b_k <= d; b_k += 2) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    tmp_1 = _mm_loadu_pd(&b_x[b_k]);
    _mm_storeu_pd(&b_x[b_k], _mm_sqrt_pd(tmp_1));
  }

  for (b_k = c_tmp; b_k <= b; b_k++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_x[b_k] = std::sqrt(b_x[b_k]);
  }

  b_k = 0;
  for (b = 0; b < 20000; b++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    if (longitudinal_mpc_B.b[b] != 0.0) {
      b_k++;
    }
  }

  c_tmp = 0;
  for (b = 0; b < 20000; b++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    if (longitudinal_mpc_B.b[b] != 0.0) {
      longitudinal_mpc_B.tmp_data[c_tmp] = static_cast<int16_T>(b);
      c_tmp++;
    }
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  Mean.set_size(b_k);
  for (c_tmp = 0; c_tmp < b_k; c_tmp++) {
    Mean[c_tmp] = longitudinal_mpc_B.Mean[longitudinal_mpc_B.tmp_data[c_tmp]];
  }

  longitudinal_mpc_norminv(Mean, b_x, s_p);
  kidx = -1;
  for (b_k = 0; b_k < 100; b_k++) {
    for (d = 0; d < 100; d++) {
      longitudinal_mpc_B.b[kidx + 1] = 0.0;

      /* Start for MATLABSystem: '<S1>/MPC System' */
      longitudinal_mpc_B.b[kidx + 2] = A_0[100 * b_k + d];
      kidx += 2;
    }
  }

  kidx = -1;
  for (b_k = 0; b_k < 100; b_k++) {
    for (b = 0; b < 2; b++) {
      for (d = 0; d < 100; d++) {
        longitudinal_mpc_B.GammaP[kidx + 1] = 0.0;

        /* Start for MATLABSystem: '<S1>/MPC System' */
        longitudinal_mpc_B.GammaP[kidx + 2] = A_0[100 * b_k + d] * B_0[(b << 1)
          + 1];
        kidx += 2;
      }
    }
  }

  b_k = 0;
  for (b = 0; b < 40000; b++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    if (longitudinal_mpc_B.GammaP[b] != 0.0) {
      b_k++;
    }
  }

  b_x.set_size(b_k);
  c_tmp = 0;
  b_k = 0;
  for (b = 0; b < 40000; b++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    if (longitudinal_mpc_B.GammaP[b] != 0.0) {
      b_x[c_tmp] = longitudinal_mpc_B.Cov[b];
      b_k = c_tmp + 1;
      c_tmp++;
    }
  }

  b = b_k - 1;
  c_tmp = (b_k / 2) << 1;
  d = c_tmp - 2;
  for (b_k = 0; b_k <= d; b_k += 2) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    tmp_1 = _mm_loadu_pd(&b_x[b_k]);
    _mm_storeu_pd(&b_x[b_k], _mm_sqrt_pd(tmp_1));
  }

  for (b_k = c_tmp; b_k <= b; b_k++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_x[b_k] = std::sqrt(b_x[b_k]);
  }

  b_k = 0;
  for (b = 0; b < 20000; b++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    if (longitudinal_mpc_B.b[b] != 0.0) {
      b_k++;
    }
  }

  c_tmp = 0;
  for (b = 0; b < 20000; b++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    if (longitudinal_mpc_B.b[b] != 0.0) {
      longitudinal_mpc_B.tmp_data_c[c_tmp] = static_cast<int16_T>(b);
      c_tmp++;
    }
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  Mean.set_size(b_k);
  for (c_tmp = 0; c_tmp < b_k; c_tmp++) {
    Mean[c_tmp] = longitudinal_mpc_B.Mean[longitudinal_mpc_B.tmp_data_c[c_tmp]];
  }

  longitudinal_mpc_norminv(Mean, b_x, v_p);
}

void longitudinal_mpc::longitudinal_mpc_interp1(const real_T varargin_2[100],
  real_T Vq[33])
{
  real_T pp_coefs[396];
  real_T slopes[100];
  real_T del[99];
  real_T h[99];
  real_T del_0;
  real_T h_0;
  real_T u;
  real_T w1;
  real_T w2;
  int32_T low_i;
  for (int32_T b_k{0}; b_k < 99; b_k++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    w1 = (static_cast<real_T>(b_k) + 1.0) * 0.1 - 0.1 * static_cast<real_T>(b_k);
    h[b_k] = w1;

    /* Start for MATLABSystem: '<S1>/MPC System' */
    del[b_k] = (varargin_2[b_k + 1] - varargin_2[b_k]) / w1;
  }

  for (int32_T b_k{0}; b_k < 98; b_k++) {
    h_0 = h[b_k + 1];
    w2 = h[b_k];
    w1 = h_0 * 2.0 + w2;
    w2 = w2 * 2.0 + h_0;
    slopes[b_k + 1] = 0.0;

    /* Start for MATLABSystem: '<S1>/MPC System' */
    del_0 = del[b_k];
    h_0 = del[b_k + 1];
    u = del_0 * h_0;
    if (std::isnan(u)) {
      u = (rtNaN);
    } else if (u < 0.0) {
      u = -1.0;
    } else {
      u = (u > 0.0);
    }

    if (u > 0.0) {
      slopes[b_k + 1] = (w1 + w2) / (w1 / del_0 + w2 / h_0);
    }
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  w1 = ((2.0 * h[0] + h[1]) * del[0] - h[0] * del[1]) / (h[0] + h[1]);
  if (std::isnan(del[0])) {
    w2 = (rtNaN);
  } else if (del[0] < 0.0) {
    w2 = -1.0;
  } else {
    w2 = (del[0] > 0.0);
  }

  if (std::isnan(w1)) {
    u = (rtNaN);
  } else if (w1 < 0.0) {
    u = -1.0;
  } else {
    u = (w1 > 0.0);
  }

  if (u != w2) {
    w1 = 0.0;
  } else {
    if (std::isnan(del[1])) {
      u = (rtNaN);
    } else if (del[1] < 0.0) {
      u = -1.0;
    } else {
      u = (del[1] > 0.0);
    }

    if ((w2 != u) && (std::abs(w1) > std::abs(3.0 * del[0]))) {
      w1 = 3.0 * del[0];
    }
  }

  slopes[0] = w1;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  w1 = ((2.0 * h[98] + h[97]) * del[98] - del[97] * h[98]) / (h[97] + h[98]);
  if (std::isnan(del[98])) {
    w2 = (rtNaN);
  } else if (del[98] < 0.0) {
    w2 = -1.0;
  } else {
    w2 = (del[98] > 0.0);
  }

  if (std::isnan(w1)) {
    u = (rtNaN);
  } else if (w1 < 0.0) {
    u = -1.0;
  } else {
    u = (w1 > 0.0);
  }

  if (u != w2) {
    w1 = 0.0;
  } else {
    if (std::isnan(del[97])) {
      u = (rtNaN);
    } else if (del[97] < 0.0) {
      u = -1.0;
    } else {
      u = (del[97] > 0.0);
    }

    if ((w2 != u) && (std::abs(w1) > std::abs(3.0 * del[98]))) {
      w1 = 3.0 * del[98];
    }
  }

  slopes[99] = w1;
  for (low_i = 0; low_i <= 96; low_i += 2) {
    __m128d tmp;
    __m128d tmp_0;
    __m128d tmp_1;
    __m128d tmp_2;
    __m128d tmp_3;

    /* Start for MATLABSystem: '<S1>/MPC System' */
    tmp = _mm_loadu_pd(&del[low_i]);
    tmp_0 = _mm_loadu_pd(&slopes[low_i]);
    tmp_1 = _mm_loadu_pd(&h[low_i]);
    tmp_2 = _mm_div_pd(_mm_sub_pd(tmp, tmp_0), tmp_1);

    /* Start for MATLABSystem: '<S1>/MPC System' */
    tmp_3 = _mm_loadu_pd(&slopes[low_i + 1]);
    tmp = _mm_div_pd(_mm_sub_pd(tmp_3, tmp), tmp_1);

    /* Start for MATLABSystem: '<S1>/MPC System' */
    _mm_storeu_pd(&pp_coefs[low_i], _mm_div_pd(_mm_sub_pd(tmp, tmp_2), tmp_1));
    _mm_storeu_pd(&pp_coefs[low_i + 99], _mm_sub_pd(_mm_mul_pd(_mm_set1_pd(2.0),
      tmp_2), tmp));
    _mm_storeu_pd(&pp_coefs[low_i + 198], tmp_0);
    _mm_storeu_pd(&pp_coefs[low_i + 297], _mm_loadu_pd(&varargin_2[low_i]));
  }

  for (low_i = 98; low_i < 99; low_i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    del_0 = del[low_i];
    u = slopes[low_i];
    h_0 = h[low_i];
    w1 = (del_0 - u) / h_0;
    w2 = (slopes[low_i + 1] - del_0) / h_0;
    pp_coefs[low_i] = (w2 - w1) / h_0;
    pp_coefs[low_i + 99] = 2.0 * w1 - w2;
    pp_coefs[low_i + 198] = u;
    pp_coefs[low_i + 297] = varargin_2[low_i];
  }

  for (int32_T b_k{0}; b_k < 33; b_k++) {
    int32_T high_i;
    int32_T low_ip1;
    low_i = 0;
    low_ip1 = 1;
    high_i = 100;
    while (high_i > low_ip1 + 1) {
      int32_T mid_i;

      /* Start for MATLABSystem: '<S1>/MPC System' */
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (0.5 * static_cast<real_T>(b_k) >= (static_cast<real_T>(mid_i) - 1.0) *
          0.1) {
        low_i = mid_i - 1;
        low_ip1 = mid_i;
      } else {
        high_i = mid_i;
      }
    }

    /* Start for MATLABSystem: '<S1>/MPC System' */
    w1 = 0.5 * static_cast<real_T>(b_k) - 0.1 * static_cast<real_T>(low_i);
    Vq[b_k] = ((w1 * pp_coefs[low_i] + pp_coefs[low_i + 99]) * w1 +
               pp_coefs[low_i + 198]) * w1 + pp_coefs[low_i + 297];
  }
}

void longitudinal_mpc::longitudinal_mpc_PCCMPC_initMPC(solver_longitudinal_mpc_T
  *b_this)
{
  real_T PhiP[288];
  real_T p[100];
  real_T p_v[100];
  real_T dchance[33];
  real_T bkj;
  int32_T b_i1;
  int32_T b_i2;
  int32_T b_j1;
  int32_T coffset;
  int32_T i1;
  int32_T kidx;
  int8_T K[1280];
  static const real_T B[30]{ -0.0, 0.0, -1.0, -0.0, -0.0, 1.0, 0.0, 0.0, 1.0,
    0.0, -0.0, 0.0, -1.4, -1.0, -0.0, 1.4, 1.0, 0.0, 0.6, -1.0, -0.0, 0.0, -0.0,
    -0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0 };

  __m128d tmp_0;
  __m128d tmp_1;
  __m128d tmp_3;
  real_T PhiP_data[288];
  real_T PhiP_data_0[279];
  real_T b[96];
  real_T tmp[9];
  real_T tmp_2[2];
  real_T B_0;
  real_T B_1;
  int32_T i;
  int32_T loop_ub;
  int8_T coffset_0[2];
  int8_T coffset_1;
  static const int8_T A[1024]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const int8_T tmp_4[32]{ 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const int8_T A_0[992]{ 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const real_T tmp_5[24]{ -1.0, -0.0, -0.0, 1.0, 0.0, 0.0, 1.0, 0.0, -1.4,
    -1.0, -0.0, 1.4, 1.0, 0.0, 0.6, -1.0, -0.0, -0.0, -1.0, 0.0, 0.0, 1.0, 0.0,
    0.0 };

  static const int8_T tmp_6[10]{ -1, 1, 0, 0, 0, 0, 0, 0, 0, -1 };

  static const int8_T tmp_7[32]{ 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 };

  static const int8_T B_2[40]{ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 };

  static const real_T tmp_8[9]{ 1.0, 0.0, 0.0, 0.5, 1.0, 0.0,
    0.074150496220627277, 0.23036183192499177, 0.16232061118184818 };

  static const real_T B_3[9]{ 1.0, 0.0, 0.0, 1.4, 1.0, 0.0, 0.0, 0.0, 1.0 };

  /*         %% MPC functions */
  /* %% INITMPC Perform one-time calculations for controller */
  /* %% Set output signals */
  b_this->u = 0.0;
  std::memset(&b_this->U[0], 0, sizeof(real_T) << 5U);
  std::memset(&b_this->X[0], 0, 96U * sizeof(real_T));
  b_this->E[0] = 0.0;
  b_this->E[1] = 0.0;
  b_this->E[2] = 0.0;
  b_this->E[3] = 0.0;
  b_this->Ref[0] = 0.0;
  b_this->Ref[1] = 0.0;
  b_this->Ref[2] = 0.0;
  b_this->Con = 0.0;

  /* %% Cost function  */
  std::memset(&longitudinal_mpc_B.GammaP_p[0], 0, 9216U * sizeof(real_T));
  for (i = 0; i < 93; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    std::memset(&longitudinal_mpc_B.GammaP_p[i * 96], 0, 93U * sizeof(real_T));
  }

  for (i = 0; i < 3; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    coffset = (i + 93) * 96;
    longitudinal_mpc_B.GammaP_p[coffset + 93] = b_this->S[3 * i];
    longitudinal_mpc_B.GammaP_p[coffset + 94] = b_this->S[3 * i + 1];
    longitudinal_mpc_B.GammaP_p[coffset + 95] = b_this->S[3 * i + 2];
  }

  std::memcpy(&b_this->Omega[0], &longitudinal_mpc_B.GammaP_p[0], 9216U * sizeof
              (real_T));
  kidx = -1;
  for (coffset = 0; coffset < 32; coffset++) {
    for (b_i1 = 0; b_i1 <= 30; b_i1 += 2) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      i = (coffset << 5) + b_i1;
      _mm_storeu_pd(&longitudinal_mpc_B.H_n[(kidx + b_i1) + 1], _mm_mul_pd
                    (_mm_set_pd(static_cast<real_T>(A[i + 1]), static_cast<
        real_T>(A[i])), _mm_set1_pd(3000.0)));
    }

    kidx += 32;
  }

  std::memcpy(&b_this->Psi[0], &longitudinal_mpc_B.H_n[0], sizeof(real_T) << 10U);

  /* %% Set constraint matrices             */
  /*  Standard constraints */
  /*  State matrix for constraint stages */
  /*  Standard lower and upper bounds on control */
  /*  Standard lower and upper bounds on outputs */
  /*  Affine constraints */
  /*  s + v t_con \leq E{s_pv - s} - d_0 - d_c: time varying parameter */
  /*  -(mv + u) \leq min_braking */
  /*  Integer constraints */
  /*  Fix terminal stages to remove bounded control constraints */
  /*  Control matrix for constraint stages */
  /*  Standard lower and upper bounds on control */
  /*  Standard lower and upper bounds on outputs */
  /*  Affine constraints */
  /*  s + v t_con \leq E{s_pv-s} - d_0 - d_c: time varying parameter */
  /*  -(v + u) \leq min_braking */
  /*  Integer constraints */
  /*  Standard Constraints */
  kidx = -1;
  for (i = 0; i < 3; i++) {
    for (b_i1 = 0; b_i1 < 32; b_i1++) {
      for (b_i2 = 0; b_i2 <= 8; b_i2 += 2) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        _mm_storeu_pd(&longitudinal_mpc_B.K_p[(kidx + b_i2) + 1], _mm_mul_pd
                      (_mm_loadu_pd(&B[10 * i + b_i2]), _mm_set1_pd(static_cast<
          real_T>(tmp_4[b_i1]))));
      }

      kidx += 10;
    }
  }

  for (i = 0; i < 3; i++) {
    std::memcpy(&b_this->D[i * 328], &longitudinal_mpc_B.K_p[i * 320], 320U *
                sizeof(real_T));
    std::memset(&b_this->D[i * 328 + 320], 0, sizeof(real_T) << 3U);
  }

  kidx = -1;
  for (coffset = 0; coffset < 31; coffset++) {
    for (i = 0; i < 3; i++) {
      for (b_i1 = 0; b_i1 < 32; b_i1++) {
        for (b_i2 = 0; b_i2 <= 8; b_i2 += 2) {
          /* Start for MATLABSystem: '<S1>/MPC System' */
          _mm_storeu_pd(&longitudinal_mpc_B.K[(kidx + b_i2) + 1], _mm_mul_pd
                        (_mm_set1_pd(static_cast<real_T>(A_0[(coffset << 5) +
            b_i1])), _mm_loadu_pd(&B[10 * i + b_i2])));
        }

        kidx += 10;
      }
    }
  }

  std::memset(&b_this->M[0], 0, 31488U * sizeof(real_T));
  for (i = 0; i < 93; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    std::memcpy(&b_this->M[i * 328], &longitudinal_mpc_B.K[i * 320], 320U *
                sizeof(real_T));
  }

  for (i = 0; i < 3; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    std::memcpy(&b_this->M[i * 328 + 30824], &tmp_5[i << 3], sizeof(real_T) <<
                3U);
  }

  kidx = -1;
  for (coffset = 0; coffset < 32; coffset++) {
    for (b_i1 = 0; b_i1 < 32; b_i1++) {
      for (b_i2 = 0; b_i2 < 10; b_i2++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        longitudinal_mpc_B.K_m[(kidx + b_i2) + 1] = static_cast<int8_T>(A
          [(coffset << 5) + b_i1] * tmp_6[b_i2]);
      }

      kidx += 10;
    }
  }

  for (i = 0; i < 32; i++) {
    for (b_i1 = 0; b_i1 < 320; b_i1++) {
      b_this->Sigma[b_i1 + 328 * i] = longitudinal_mpc_B.K_m[320 * i + b_i1];
    }

    std::memset(&b_this->Sigma[i * 328 + 320], 0, sizeof(real_T) << 3U);
  }

  /*  Chance constraints */
  longitudinal_m_PCCMPC_getChance(p, p_v);

  /* Start for MATLABSystem: '<S1>/MPC System' */
  longitudinal_mpc_interp1(p, dchance);
  for (b_i2 = 0; b_i2 <= 30; b_i2 += 2) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    tmp_2[0] = std::fmax(dchance[b_i2], 0.0);
    tmp_2[1] = std::fmax(dchance[b_i2 + 1], 0.0);
    tmp_3 = _mm_loadu_pd(&tmp_2[0]);
    _mm_storeu_pd(&b_this->s_chance[b_i2], _mm_add_pd(tmp_3, _mm_set1_pd(3.0)));
  }

  for (b_i2 = 32; b_i2 < 33; b_i2++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_this->s_chance[b_i2] = std::fmax(dchance[b_i2], 0.0) + 3.0;
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Chance constraint should not go below 0, add standstill gap d0 here */
  longitudinal_mpc_interp1(p_v, b_this->v_chance);

  /*  Softened constraints */
  /*  Standard lower and upper bounds on control */
  /*  Standard lower and upper bounds on outputs */
  /*  Affine constraints */
  /*  Integer constraints */
  for (i = 0; i < 32; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_this->upsn[i] = tmp_7[i];
  }

  kidx = -1;
  for (i = 0; i < 4; i++) {
    for (i1 = 0; i1 < 32; i1++) {
      for (b_i2 = 0; b_i2 < 10; b_i2++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        K[(kidx + b_i2) + 1] = B_2[10 * i + b_i2];
      }

      kidx += 10;
    }
  }

  for (i = 0; i < 4; i++) {
    for (b_i1 = 0; b_i1 < 320; b_i1++) {
      b_this->Upsilon[b_i1 + 328 * i] = K[320 * i + b_i1];
    }

    std::memcpy(&b_this->Upsilon[i * 328 + 320], &b_this->upsn[i << 3], sizeof
                (real_T) << 3U);
  }

  std::memset(&b_this->UpsilonI[0], 0, sizeof(real_T) << 4U);
  b_this->UpsilonI[0] = 1.0;
  b_this->Upsilonb[0] = 0.0;
  b_this->UpsilonI[5] = 1.0;
  b_this->Upsilonb[1] = 0.0;
  b_this->UpsilonI[10] = 1.0;
  b_this->Upsilonb[2] = 0.0;
  b_this->UpsilonI[15] = 1.0;
  b_this->Upsilonb[3] = 0.0;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  if nsoft */
  /*  Integer constraints */
  /*  Standard lower and upper bounds on control */
  /*  Standard lower and upper bounds on outputs */
  /*  Affine constraints */
  /*  Integer constraints */
  /*  Standard lower and upper bounds on control */
  /*  Standard lower and upper bounds on outputs */
  /*  Affine constraints */
  /*  Integer constraints */
  std::memset(&b_this->betabn[0], 0, sizeof(real_T) << 3U);
  std::memset(&b_this->beta_rhs[0], 0, 320U * sizeof(real_T));
  std::memcpy(&b_this->beta_rhs[320], &b_this->betabn[0], sizeof(real_T) << 3U);

  /*  if nint */
  /* %% Build MPC matrices */
  /*  Control model */
  std::memset(&PhiP[0], 0, 288U * sizeof(real_T));
  PhiP[0] = 1.0;
  PhiP[97] = 1.0;
  PhiP[194] = 1.0;
  for (b_i2 = 0; b_i2 < 31; b_i2++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    coffset = (b_i2 + 1) * 3;
    i = static_cast<int32_T>(((static_cast<real_T>(b_i2) + 2.0) - 2.0) * 3.0 +
      1.0);
    if (i > coffset) {
      i1 = 0;
      kidx = 0;
    } else {
      i1 = i - 1;
      kidx = coffset;
    }

    coffset = (b_i2 + 2) * 3;
    i = static_cast<int32_T>(((static_cast<real_T>(b_i2) + 2.0) - 1.0) * 3.0 +
      1.0);
    if (i > coffset) {
      b_j1 = 0;
      coffset = 0;
    } else {
      b_j1 = i - 1;
    }

    loop_ub = kidx - i1;
    for (i = 0; i < 3; i++) {
      for (b_i1 = 0; b_i1 < loop_ub; b_i1++) {
        PhiP_data_0[b_i1 + loop_ub * i] = PhiP[(i1 + b_i1) + 96 * i];
      }
    }

    for (i = 0; i < 3; i++) {
      bkj = tmp_8[i + 3];
      b_i1 = static_cast<int32_T>(tmp_8[i]);
      B_0 = tmp_8[i + 6];
      for (i1 = 0; i1 < 3; i1++) {
        tmp[i + 3 * i1] = (PhiP_data_0[3 * i1 + 1] * bkj + PhiP_data_0[3 * i1] *
                           static_cast<real_T>(b_i1)) + PhiP_data_0[3 * i1 + 2] *
          B_0;
      }
    }

    coffset_1 = static_cast<int8_T>(coffset - b_j1);
    coffset_0[0] = coffset_1;
    loop_ub = coffset_1;
    for (i = 0; i < 3; i++) {
      for (b_i1 = 0; b_i1 < loop_ub; b_i1++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        PhiP[(b_j1 + b_i1) + 96 * i] = tmp[coffset_0[0] * i + b_i1];
      }
    }
  }

  std::memset(&longitudinal_mpc_B.GammaP_p[0], 0, 9216U * sizeof(real_T));
  for (b_i2 = 0; b_i2 < 96; b_i2++) {
    longitudinal_mpc_B.GammaP_p[b_i2 + 96 * b_i2] = 1.0;
  }

  for (b_j1 = 0; b_j1 < 31; b_j1++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_i2 = b_j1 * 3 + 1;
    coffset = (b_j1 + 1) * 3;
    i = static_cast<int32_T>(((static_cast<real_T>(b_j1) + 1.0) - 1.0) * 3.0 +
      1.0);
    if (i > coffset) {
      i1 = 0;
      kidx = 0;
    } else {
      i1 = i - 1;
      kidx = coffset;
    }

    loop_ub = (32 - b_j1) * 3;
    for (i = 0; i < 3; i++) {
      for (b_i1 = 0; b_i1 < loop_ub; b_i1++) {
        PhiP_data[b_i1 + loop_ub * i] = PhiP[96 * i + b_i1];
      }
    }

    coffset_0[0] = static_cast<int8_T>(97 - b_i2);
    coffset_0[1] = static_cast<int8_T>(kidx - i1);
    loop_ub = coffset_0[1];
    for (i = 0; i < loop_ub; i++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      coffset = 97 - b_i2;
      for (b_i1 = 0; b_i1 < coffset; b_i1++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        longitudinal_mpc_B.GammaP_p[((b_i2 + b_i1) + 96 * (i1 + i)) - 1] =
          PhiP_data[coffset_0[0] * i + b_i1];
      }
    }
  }

  for (i = 0; i < 3; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    bkj = tmp_8[3 * i + 1];
    B_0 = tmp_8[3 * i];
    B_1 = tmp_8[3 * i + 2];
    for (b_i1 = 0; b_i1 <= 94; b_i1 += 2) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      tmp_3 = _mm_loadu_pd(&PhiP[b_i1 + 96]);
      tmp_0 = _mm_loadu_pd(&PhiP[b_i1]);
      tmp_1 = _mm_loadu_pd(&PhiP[b_i1 + 192]);
      _mm_storeu_pd(&b_this->Phi[b_i1 + 96 * i], _mm_add_pd(_mm_add_pd
        (_mm_mul_pd(_mm_set1_pd(bkj), tmp_3), _mm_mul_pd(_mm_set1_pd(B_0), tmp_0)),
        _mm_mul_pd(_mm_set1_pd(B_1), tmp_1)));
    }
  }

  kidx = -1;
  for (coffset = 0; coffset < 32; coffset++) {
    for (b_i1 = 0; b_i1 < 32; b_i1++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      coffset_1 = A[(coffset << 5) + b_i1];
      longitudinal_mpc_B.b_f[kidx + 1] = static_cast<real_T>(coffset_1) *
        0.05084950377937273;
      longitudinal_mpc_B.b_f[kidx + 2] = static_cast<real_T>(coffset_1) *
        0.26963816807500823;
      longitudinal_mpc_B.b_f[kidx + 3] = static_cast<real_T>(coffset_1) *
        0.83767938881815185;
      kidx += 3;
    }
  }

  for (i = 0; i < 32; i++) {
    for (b_i1 = 0; b_i1 < 96; b_i1++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        bkj += longitudinal_mpc_B.GammaP_p[96 * i1 + b_i1] *
          longitudinal_mpc_B.b_f[96 * i + i1];
      }

      /* Start for MATLABSystem: '<S1>/MPC System' */
      b_this->Gamma[b_i1 + 96 * i] = bkj;
    }
  }

  kidx = -1;
  for (coffset = 0; coffset < 32; coffset++) {
    for (i = 0; i < 3; i++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      bkj = B_3[3 * i];
      B_0 = B_3[3 * i + 1];
      B_1 = B_3[3 * i + 2];
      for (b_i1 = 0; b_i1 < 32; b_i1++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        coffset_1 = A[(coffset << 5) + b_i1];
        longitudinal_mpc_B.GammaP_p[kidx + 1] = static_cast<real_T>(coffset_1) *
          bkj;
        longitudinal_mpc_B.GammaP_p[kidx + 2] = static_cast<real_T>(coffset_1) *
          B_0;
        longitudinal_mpc_B.GammaP_p[kidx + 3] = static_cast<real_T>(coffset_1) *
          B_1;
        kidx += 3;
      }
    }
  }

  for (i = 0; i < 9216; i++) {
    b_this->GammaC[i] = longitudinal_mpc_B.GammaP_p[i];
    longitudinal_mpc_B.GammaP_p[i] = 0.0;
  }

  for (b_i2 = 0; b_i2 < 96; b_i2++) {
    longitudinal_mpc_B.GammaP_p[b_i2 + 96 * b_i2] = 1.0;
  }

  for (b_j1 = 0; b_j1 < 31; b_j1++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_i2 = b_j1 * 3 + 1;
    coffset = (b_j1 + 1) * 3;
    i = static_cast<int32_T>(((static_cast<real_T>(b_j1) + 1.0) - 1.0) * 3.0 +
      1.0);
    if (i > coffset) {
      i1 = 0;
      kidx = 0;
    } else {
      i1 = i - 1;
      kidx = coffset;
    }

    loop_ub = (32 - b_j1) * 3;
    for (i = 0; i < 3; i++) {
      for (b_i1 = 0; b_i1 < loop_ub; b_i1++) {
        PhiP_data[b_i1 + loop_ub * i] = PhiP[96 * i + b_i1];
      }
    }

    coffset_0[0] = static_cast<int8_T>(97 - b_i2);
    coffset_0[1] = static_cast<int8_T>(kidx - i1);
    loop_ub = coffset_0[1];
    for (i = 0; i < loop_ub; i++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      coffset = 97 - b_i2;
      for (b_i1 = 0; b_i1 < coffset; b_i1++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        longitudinal_mpc_B.GammaP_p[((b_i2 + b_i1) + 96 * (i1 + i)) - 1] =
          PhiP_data[coffset_0[0] * i + b_i1];
      }
    }
  }

  for (i = 0; i < 32; i++) {
    for (b_i1 = 0; b_i1 < 96; b_i1++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        bkj += longitudinal_mpc_B.GammaP_p[96 * i1 + b_i1] * 0.0;
      }

      /* Start for MATLABSystem: '<S1>/MPC System' */
      b_this->GammaW[b_i1 + 96 * i] = bkj;
    }
  }

  /*  Cost function */
  /*  J = 1/2 U' H U + 2 U' (F x(0) - Ty Yr - Tu Ur) */
  for (i1 = 0; i1 < 96; i1++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    coffset = (i1 << 5) - 1;
    std::memset(&longitudinal_mpc_B.b_f[coffset + 1], 0, sizeof(real_T) << 5U);
    for (b_i2 = 0; b_i2 < 96; b_i2++) {
      bkj = b_this->GammaC[b_i2 * 96 + i1];
      for (b_i1 = 0; b_i1 < 32; b_i1++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        i = (coffset + b_i1) + 1;
        longitudinal_mpc_B.b_f[i] += b_this->Gamma[b_i1 * 96 + b_i2] * bkj;
      }
    }
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  for (i = 0; i < 32; i++) {
    for (b_i1 = 0; b_i1 < 96; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += longitudinal_mpc_B.b_f[(i1 << 5) + i] * b_this->Omega[96 * b_i1 +
          i1];
      }

      longitudinal_mpc_B.b_g[i + (b_i1 << 5)] = bkj;
    }

    for (b_i1 = 0; b_i1 < 96; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += longitudinal_mpc_B.b_g[(i1 << 5) + i] * b_this->GammaC[96 * b_i1
          + i1];
      }

      longitudinal_mpc_B.b_g1[i + (b_i1 << 5)] = bkj;
    }

    for (b_i1 = 0; b_i1 < 32; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += longitudinal_mpc_B.b_g1[(i1 << 5) + i] * b_this->Gamma[96 * b_i1
          + i1];
      }

      coffset = (b_i1 << 5) + i;
      longitudinal_mpc_B.b_this[coffset] = b_this->Psi[coffset] + bkj;
    }
  }

  for (i = 0; i <= 1022; i += 2) {
    tmp_3 = _mm_loadu_pd(&longitudinal_mpc_B.b_this[i]);

    /* Start for MATLABSystem: '<S1>/MPC System' */
    _mm_storeu_pd(&longitudinal_mpc_B.H_n[i], _mm_mul_pd(_mm_set1_pd(2.0), tmp_3));
  }

  for (i = 0; i < 32; i++) {
    for (b_i1 = 0; b_i1 <= 30; b_i1 += 2) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      i1 = (i << 5) + b_i1;
      tmp_3 = _mm_mul_pd(_mm_add_pd(_mm_loadu_pd(&longitudinal_mpc_B.H_n[i1]),
        _mm_set_pd(longitudinal_mpc_B.H_n[((b_i1 + 1) << 5) + i],
                   longitudinal_mpc_B.H_n[(b_i1 << 5) + i])), _mm_set1_pd(0.5));
      _mm_storeu_pd(&b_this->G[i1], tmp_3);
    }
  }

  for (i1 = 0; i1 < 96; i1++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    coffset = (i1 << 5) - 1;
    std::memset(&longitudinal_mpc_B.b_f[coffset + 1], 0, sizeof(real_T) << 5U);
    for (b_i2 = 0; b_i2 < 96; b_i2++) {
      bkj = b_this->GammaC[b_i2 * 96 + i1];
      for (b_i1 = 0; b_i1 < 32; b_i1++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        i = (coffset + b_i1) + 1;
        longitudinal_mpc_B.b_f[i] += b_this->Gamma[b_i1 * 96 + b_i2] * bkj;
      }
    }
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  for (i = 0; i < 32; i++) {
    for (b_i1 = 0; b_i1 < 96; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += longitudinal_mpc_B.b_f[(i1 << 5) + i] * b_this->Omega[96 * b_i1 +
          i1];
      }

      longitudinal_mpc_B.b_g[i + (b_i1 << 5)] = bkj;
    }

    for (b_i1 = 0; b_i1 < 96; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += longitudinal_mpc_B.b_g[(i1 << 5) + i] * b_this->GammaC[96 * b_i1
          + i1];
      }

      longitudinal_mpc_B.b_g1[i + (b_i1 << 5)] = bkj;
    }

    for (b_i1 = 0; b_i1 < 3; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += longitudinal_mpc_B.b_g1[(i1 << 5) + i] * b_this->Phi[96 * b_i1 +
          i1];
      }

      b[i + (b_i1 << 5)] = bkj;
    }
  }

  for (i1 = 0; i1 < 96; i1++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_this->F[i1] = 2.0 * b[i1];
    coffset = (i1 << 5) - 1;
    std::memset(&longitudinal_mpc_B.b_f[coffset + 1], 0, sizeof(real_T) << 5U);
    for (b_i2 = 0; b_i2 < 96; b_i2++) {
      bkj = b_this->GammaC[b_i2 * 96 + i1];
      for (b_i1 = 0; b_i1 < 32; b_i1++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        i = (coffset + b_i1) + 1;
        longitudinal_mpc_B.b_f[i] += b_this->Gamma[b_i1 * 96 + b_i2] * bkj;
      }
    }
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  for (i = 0; i < 32; i++) {
    for (b_i1 = 0; b_i1 < 96; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += longitudinal_mpc_B.b_f[(i1 << 5) + i] * b_this->Omega[96 * b_i1 +
          i1];
      }

      longitudinal_mpc_B.b_g[i + (b_i1 << 5)] = bkj;
    }

    for (b_i1 = 0; b_i1 < 96; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += longitudinal_mpc_B.b_g[(i1 << 5) + i] * b_this->GammaC[96 * b_i1
          + i1];
      }

      longitudinal_mpc_B.b_g1[i + (b_i1 << 5)] = bkj;
    }

    for (b_i1 = 0; b_i1 < 32; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += longitudinal_mpc_B.b_g1[(i1 << 5) + i] * b_this->GammaW[96 * b_i1
          + i1];
      }

      longitudinal_mpc_B.H_n[i + (b_i1 << 5)] = bkj;
    }
  }

  for (i = 0; i <= 1022; i += 2) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    tmp_3 = _mm_loadu_pd(&longitudinal_mpc_B.H_n[i]);
    _mm_storeu_pd(&b_this->Fw[i], _mm_mul_pd(_mm_set1_pd(2.0), tmp_3));
  }

  for (i1 = 0; i1 < 96; i1++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    coffset = (i1 << 5) - 1;
    std::memset(&longitudinal_mpc_B.b_f[coffset + 1], 0, sizeof(real_T) << 5U);
    for (b_i2 = 0; b_i2 < 96; b_i2++) {
      bkj = b_this->GammaC[b_i2 * 96 + i1];
      for (b_i1 = 0; b_i1 < 32; b_i1++) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        i = (coffset + b_i1) + 1;
        longitudinal_mpc_B.b_f[i] += b_this->Gamma[b_i1 * 96 + b_i2] * bkj;
      }
    }
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  for (i = 0; i < 96; i++) {
    for (b_i1 = 0; b_i1 < 32; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += longitudinal_mpc_B.b_f[(i1 << 5) + b_i1] * b_this->Omega[96 * i +
          i1];
      }

      longitudinal_mpc_B.b_g[b_i1 + (i << 5)] = bkj;
    }
  }

  for (i = 0; i <= 3070; i += 2) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    tmp_3 = _mm_loadu_pd(&longitudinal_mpc_B.b_g[i]);
    _mm_storeu_pd(&b_this->Ty[i], _mm_mul_pd(_mm_set1_pd(2.0), tmp_3));
  }

  for (i = 0; i <= 1022; i += 2) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    tmp_3 = _mm_loadu_pd(&b_this->Psi[i]);
    _mm_storeu_pd(&b_this->Tu[i], _mm_mul_pd(_mm_set1_pd(2.0), tmp_3));
  }

  /*  Constraints */
  for (i = 0; i < 328; i++) {
    for (b_i1 = 0; b_i1 < 32; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += b_this->M[328 * i1 + i] * b_this->Gamma[96 * b_i1 + i1];
      }

      i1 = 328 * b_i1 + i;
      b_this->a[i1] = b_this->Sigma[i1] + bkj;
    }

    for (b_i1 = 0; b_i1 < 3; b_i1++) {
      bkj = 0.0;
      for (i1 = 0; i1 < 96; i1++) {
        bkj += b_this->M[328 * i1 + i] * b_this->Phi[96 * b_i1 + i1];
      }

      i1 = 328 * b_i1 + i;
      b_this->W[i1] = b_this->D[i1] + bkj;
    }
  }
}

solver_longitudinal_mpc_T *longitudinal_mpc::longitudinal_mpc_solver_solver
  (solver_longitudinal_mpc_T *b_this)
{
  solver_longitudinal_mpc_T *c_this;
  real_T K[320];
  real_T A[32];
  real_T A4[16];
  real_T B[10];
  real_T Pold[9];
  real_T b[9];
  real_T K_0[8];
  real_T d6;
  real_T eta1;
  real_T s;
  int32_T kidx;
  boolean_T p;
  static const real_T tmp[9]{ 1.0, 1.4, 0.0, 1.4, 2.46, 0.0, 0.0, 0.0, 3000.0 };

  static const real_T y[16]{ 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.5,
    -1.8181818181818181, 0.0, 0.0, 0.0, 1.8181818181818181, 0.0 };

  __m128d tmp_3;
  __m128d tmp_5;
  real_T tmp_0[16];
  real_T b_tmp[9];
  real_T b_tmp_1[9];
  real_T b_tmp_0[3];
  real_T y_tmp[3];
  real_T y_tmp_0[3];
  real_T tmp_4[2];
  real_T tmp_6[2];
  real_T tmp_7[2];
  real_T tmp_8[2];
  real_T tmp_9[2];
  real_T tmp_a[2];
  real_T tmp_b[2];
  real_T b_tmp_2;
  real_T tmp_1;
  real_T tmp_2;
  int32_T i;
  int32_T j;
  static const real_T y_0[16]{ 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.5,
    -1.8181818181818181, 0.0, 0.0, 0.0, 1.8181818181818181, 0.0 };

  static const real_T b_A6[16]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    2.7320538214602821, -9.93474116894648, 36.126331523441742, 0.0,
    -2.7320538214602821, 9.93474116894648, -36.126331523441742, 0.0 };

  static const real_T b_A4[16]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.82644628099173545, -3.0052592036063106, 10.928215285841128, 0.0,
    -0.82644628099173545, 3.0052592036063106, -10.928215285841128, 0.0 };

  static const real_T tmp_c[16]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    29.856472333422925, -108.56899030335609, 394.79632837584023, 0.0,
    -29.856472333422925, 108.56899030335609, -394.79632837584023, 0.0 };

  static const real_T tmp_d[9]{ 1.0, 1.4, 0.0, 1.4, 2.46, 0.0, 0.0, 0.0, 3000.0
  };

  static const real_T tmp_e[9]{ 1.0, 0.5, 0.074150496220627277, 0.0, 1.0,
    0.23036183192499177, 0.0, 0.0, 0.16232061118184818 };

  static const real_T Bd[3]{ 0.05084950377937273, 0.26963816807500823,
    0.83767938881815185 };

  static const real_T Ad[9]{ 1.0, 0.0, 0.0, 0.5, 1.0, 0.0, 0.074150496220627277,
    0.23036183192499177, 0.16232061118184818 };

  static const boolean_T tmp_f[10]{ false, false, false, false, false, false,
    false, false, true, false };

  static const boolean_T tmp_g[8]{ false, false, false, false, false, false,
    true, false };

  real_T tmp_h[16];
  int32_T exitg1;
  boolean_T exitg2;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T guard3;
  boolean_T guard4;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Constructor */
  /* %% Set control model */
  /*  First order lag on acceleration \dot{a} = tauinv (u - a) */
  /*  Continuous time */
  /*  s v a */
  c_this = b_this;
  b_this->isInitialized = 0;

  /*  Discretize */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* %% Tyler Ard                %%% */
  /* %% Argonne National Lab     %%% */
  /* %% Vehicle Mobility Systems %%% */
  /* %% tard(at)anl.gov          %%% */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Get matrix dimensions */
  /*  Make square matrix S = [A, B; 0, 0] */
  /*  Perform zero order hold */
  p = true;
  j = 1;
  exitg2 = false;
  while ((!exitg2) && (j - 1 < 4)) {
    kidx = 1;
    do {
      exitg1 = 0;
      if (kidx - 1 < 4) {
        if ((kidx != j) && (!(y_0[(((j - 1) << 2) + kidx) - 1] == 0.0))) {
          p = false;
          exitg1 = 1;
        } else {
          kidx++;
        }
      } else {
        j++;
        exitg1 = 2;
      }
    } while (exitg1 == 0);

    if (exitg1 == 1) {
      exitg2 = true;
    }
  }

  if (!p) {
    p = true;
    j = 0;
    exitg2 = false;
    while ((!exitg2) && (j < 4)) {
      kidx = 0;
      do {
        exitg1 = 0;
        if (kidx <= j) {
          if (!(y_0[(j << 2) + kidx] == y_0[(kidx << 2) + j])) {
            p = false;
            exitg1 = 1;
          } else {
            kidx++;
          }
        } else {
          j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (!p) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      d6 = rt_powd_snf(longitudinal_mpc_norm(b_A6), 0.16666666666666666);
      eta1 = std::fmax(rt_powd_snf(longitudinal_mpc_norm(b_A4), 0.25), d6);
      guard1 = false;
      guard2 = false;
      guard3 = false;
      guard4 = false;
      if (eta1 <= 0.01495585217958292) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        for (j = 0; j <= 14; j += 2) {
          tmp_4[0] = std::abs(y_0[j]);
          tmp_4[1] = std::abs(y_0[j + 1]);
          tmp_5 = _mm_loadu_pd(&tmp_4[0]);
          _mm_storeu_pd(&tmp_0[j], _mm_mul_pd(_mm_set1_pd(0.19285012468241128),
            tmp_5));
        }

        longitudinal_mpc_mpower(tmp_0, 7.0, tmp_h);
        if (std::fmax(std::ceil(longitudinal_mpc_log2(longitudinal_mpc_norm
               (tmp_h) / longitudinal_mpc_norm(y_0) * 2.0 /
               2.2204460492503131E-16) / 6.0), 0.0) == 0.0) {
        } else {
          guard4 = true;
        }
      } else {
        guard4 = true;
      }

      if (guard4) {
        if (eta1 <= 0.253939833006323) {
          /* Start for MATLABSystem: '<S1>/MPC System' */
          for (j = 0; j <= 14; j += 2) {
            tmp_6[0] = std::abs(y_0[j]);
            tmp_6[1] = std::abs(y_0[j + 1]);
            tmp_5 = _mm_loadu_pd(&tmp_6[0]);
            _mm_storeu_pd(&tmp_0[j], _mm_mul_pd(_mm_set1_pd(0.12321872304378752),
              tmp_5));
          }

          longitudinal_mpc_mpower(tmp_0, 11.0, tmp_h);
          if (std::fmax(std::ceil(longitudinal_mpc_log2(longitudinal_mpc_norm
                 (tmp_h) / longitudinal_mpc_norm(y_0) * 2.0 /
                 2.2204460492503131E-16) / 10.0), 0.0) == 0.0) {
          } else {
            guard3 = true;
          }
        } else {
          guard3 = true;
        }
      }

      if (guard3) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        longitudinal_mpc_mpower(b_A4, 2.0, tmp_0);
        eta1 = rt_powd_snf(longitudinal_mpc_norm(tmp_0), 0.125);
        d6 = std::fmax(d6, eta1);
        if (d6 <= 0.95041789961629319) {
          /* Start for MATLABSystem: '<S1>/MPC System' */
          for (j = 0; j <= 14; j += 2) {
            tmp_7[0] = std::abs(y_0[j]);
            tmp_7[1] = std::abs(y_0[j + 1]);
            tmp_5 = _mm_loadu_pd(&tmp_7[0]);
            _mm_storeu_pd(&tmp_0[j], _mm_mul_pd(_mm_set1_pd(0.090475336558796943),
              tmp_5));
          }

          longitudinal_mpc_mpower(tmp_0, 15.0, tmp_h);
          if (std::fmax(std::ceil(longitudinal_mpc_log2(longitudinal_mpc_norm
                 (tmp_h) / longitudinal_mpc_norm(y_0) * 2.0 /
                 2.2204460492503131E-16) / 14.0), 0.0) == 0.0) {
          } else {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
      }

      if (guard2) {
        if (d6 <= 2.097847961257068) {
          /* Start for MATLABSystem: '<S1>/MPC System' */
          for (j = 0; j <= 14; j += 2) {
            tmp_8[0] = std::abs(y_0[j]);
            tmp_8[1] = std::abs(y_0[j + 1]);
            tmp_5 = _mm_loadu_pd(&tmp_8[0]);
            _mm_storeu_pd(&tmp_0[j], _mm_mul_pd(_mm_set1_pd(0.071467735648795785),
              tmp_5));
          }

          longitudinal_mpc_mpower(tmp_0, 19.0, tmp_h);
          if (std::fmax(std::ceil(longitudinal_mpc_log2(longitudinal_mpc_norm
                 (tmp_h) / longitudinal_mpc_norm(y_0) * 2.0 /
                 2.2204460492503131E-16) / 18.0), 0.0) == 0.0) {
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
      }

      if (guard1) {
        /* Start for MATLABSystem: '<S1>/MPC System' */
        d6 = std::fmax(std::ceil(longitudinal_mpc_log2(std::fmin(d6, std::fmax
          (eta1, rt_powd_snf(longitudinal_mpc_norm(tmp_c), 0.1))) /
          5.3719203511481517)), 0.0);
        eta1 = rt_powd_snf(2.0, d6);
        for (j = 0; j <= 14; j += 2) {
          tmp_5 = _mm_div_pd(_mm_loadu_pd(&y[j]), _mm_set1_pd(eta1));
          _mm_storeu_pd(&A4[j], tmp_5);
          _mm_storeu_pd(&tmp_a[0], tmp_5);
          tmp_9[0] = std::abs(tmp_a[0]);
          tmp_9[1] = std::abs(tmp_a[1]);
          tmp_5 = _mm_loadu_pd(&tmp_9[0]);
          _mm_storeu_pd(&tmp_0[j], _mm_mul_pd(_mm_set1_pd(0.05031554467093536),
            tmp_5));
        }

        longitudinal_mpc_mpower(tmp_0, 27.0, tmp_h);
        d6 += std::fmax(std::ceil(longitudinal_mpc_log2(longitudinal_mpc_norm
          (tmp_h) / longitudinal_mpc_norm(A4) * 2.0 / 2.2204460492503131E-16) /
          26.0), 0.0);
        if (std::isinf(d6)) {
          d6 = longitudinal_mpc_norm(y_0) / 5.3719203511481517;
          if ((!std::isinf(d6)) && (!std::isnan(d6))) {
            std::frexp(d6, &i);
          }
        }
      }
    }
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Extract */
  /* %% Set objective weights */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* %% Tyler Ard                %%% */
  /* %% Argonne National Lab     %%% */
  /* %% Vehicle Mobility Systems %%% */
  /* %% tard(at)anl.gov          %%% */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  SOLVEDLQR Computes infinite-horizon lqr solution and gain matrix */
  /*  */
  /*  Args: */
  /*    Ad (matrix): discrete state update matrix */
  /*    Bd (matrix): discrete control update matrix */
  /*    Q (matrix): state weighting matrix */
  /*    R (matrix): control weighting matrix */
  /*    N (matrix): coupling weighting matrix between state and control */
  /*    e.g. cost 2 xk' N uk */
  /*        The default is zero. */
  /*  */
  /*  Returns: */
  /*    P (matrix): discrete time algebraic ricatti equation solution matrix */
  /*    F (matrix): optimal input gain matrix s.t. u_k = -F_k x_k */
  /*  */
  /*  See also: https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator#Infinite-horizon.2C_discrete-time_LQR */
  /*  Check inputs */
  /*  Setup */
  /*  Time horizon */
  std::memcpy(&b[0], &tmp_d[0], 9U * sizeof(real_T));

  /*  Terminal condition P_Hp == Q_Hp */
  /*  Update by iterating the solution backwards in time from stages Hp, Hp-1, ... */
  d6 = 1.0;

  /*  Maximum allowed ricatti update equations */
  for (i = 0; i < 1000; i++) {
    longitudinal_mpc_B.e[i] = (rtInf);
  }

  while ((longitudinal_mpc_B.e[static_cast<int32_T>(d6) - 1] > 1.0E-6) && (d6 <
          1000.0)) {
    /*  Update P and F */
    std::memcpy(&Pold[0], &b[0], 9U * sizeof(real_T));

    /* %% Internal functions */
    /*  ITERATEF Return Fk given Pk+1 and discrete lqr matrices */
    /*  inv(A)*b */
    /*  ITERATEP Returns Pk-1 given Pk and discrete lqr matrices */
    eta1 = 0.0;
    for (i = 0; i < 3; i++) {
      s = 0.0;
      b_tmp_2 = tmp_e[i + 3];
      tmp_1 = tmp_e[i];
      tmp_2 = tmp_e[i + 6];
      for (j = 0; j < 3; j++) {
        s += b[3 * i + j] * Bd[j];
        b_tmp[i + 3 * j] = (b[3 * j + 1] * b_tmp_2 + b[3 * j] * tmp_1) + b[3 * j
          + 2] * tmp_2;
      }

      y_tmp[i] = s;
      eta1 += s * Bd[i];
    }

    for (i = 0; i < 3; i++) {
      b_tmp_2 = 0.0;
      tmp_1 = 0.0;
      for (j = 0; j < 3; j++) {
        kidx = 3 * j + i;
        _mm_storeu_pd(&tmp_b[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd(Ad[3 * i + j],
          b_tmp[kidx]), _mm_set_pd(y_tmp[j], Bd[j])), _mm_set_pd(tmp_1, b_tmp_2)));
        b_tmp_2 = tmp_b[0];
        tmp_1 = tmp_b[1];
        b_tmp_1[kidx] = (Ad[3 * j + 1] * b_tmp[i + 3] + Ad[3 * j] * b_tmp[i]) +
          Ad[3 * j + 2] * b_tmp[i + 6];
      }

      b_tmp_0[i] = b_tmp_2 / (eta1 + 3000.0);
      y_tmp_0[i] = tmp_1;
    }

    eta1 = b_tmp_0[0];
    s = b_tmp_0[1];
    b_tmp_2 = b_tmp_0[2];
    for (i = 0; i < 3; i++) {
      _mm_storeu_pd(&b_tmp[3 * i], _mm_mul_pd(_mm_set_pd(s, eta1), _mm_set1_pd
        (y_tmp_0[i])));
      b_tmp[3 * i + 2] = b_tmp_2 * y_tmp_0[i];
    }

    /*  Iterate */
    d6++;
    for (i = 0; i <= 6; i += 2) {
      tmp_5 = _mm_loadu_pd(&b_tmp_1[i]);
      tmp_3 = _mm_loadu_pd(&b_tmp[i]);
      tmp_5 = _mm_add_pd(_mm_sub_pd(tmp_5, tmp_3), _mm_loadu_pd(&tmp[i]));
      _mm_storeu_pd(&b[i], tmp_5);
      tmp_3 = _mm_loadu_pd(&Pold[i]);
      _mm_storeu_pd(&Pold[i], _mm_sub_pd(tmp_5, tmp_3));
    }

    for (i = 8; i < 9; i++) {
      eta1 = (b_tmp_1[i] - b_tmp[i]) + tmp_d[i];
      b[i] = eta1;
      Pold[i] = eta1 - Pold[i];
    }

    eta1 = 0.0;
    kidx = 0;
    exitg2 = false;
    while ((!exitg2) && (kidx < 3)) {
      s = (std::abs(Pold[kidx + 3]) + std::abs(Pold[kidx])) + std::abs(Pold[kidx
        + 6]);
      if (std::isnan(s)) {
        eta1 = (rtNaN);
        exitg2 = true;
      } else {
        if (s > eta1) {
          eta1 = s;
        }

        kidx++;
      }
    }

    longitudinal_mpc_B.e[static_cast<int32_T>(d6) - 1] = eta1;
  }

  /*  Check quality */
  for (i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_this->S[i] = b[i];
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Terminal output weighting matrix */
  /*  Slack variables quadratic and inf norm */
  /* %% Set dimensions */
  /*  % Inputs cib */
  /*  % Slacks c */
  /*  % Betas b */
  /* %% Set default MPC problem */
  /*  One time calculations and  */
  longitudinal_mpc_PCCMPC_initMPC(b_this);

  /*  Check input arguments */
  b_this->pos_max = (rtInf);
  b_this->vel_max = 25.0;

  /* %% Calculations that vary on input signals */
  /*  Control model */
  /*  Cost function */
  /*  Standard constraints - rhs */
  b_this->bi[0] = 3.0;
  b_this->bi[1] = 3.0;
  b_this->bi[2] = 0.0;
  b_this->bi[3] = 0.0;
  b_this->bi[4] = (rtInf);
  b_this->bi[5] = b_this->pos_max - 3.0;
  b_this->bi[6] = b_this->vel_max;
  b_this->bi[7] = (rtInf);
  b_this->bi[8] = 0.0;
  b_this->bi[9] = 0.25;

  /*  RHS bound for Mi xi + Ei ui \leq bi */
  /*  Standard lower and upper bounds on control */
  /*  -ua */
  /*  ua */
  /*  Standard lower and upper bounds on outputs */
  /*  -s */
  /*  -v */
  /*  -a */
  /*  s */
  /*  v */
  /*  a */
  /*  Affine constraints */
  /*  s + v t_con \leq E{s_pv} - d_0 - d_c: time varying parameter */
  /*  -(mv + u) \leq min_braking */
  /*  Integer constraints */
  for (i = 0; i < 8; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_this->bn[i] = b_this->bi[i + 2];
  }

  for (i = 0; i < 10; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    B[i] = b_this->bi[i];
  }

  kidx = -1;
  for (j = 0; j < 32; j++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    std::memcpy(&K[kidx + 1], &B[0], 10U * sizeof(real_T));
    kidx += 10;
  }

  for (i = 0; i < 320; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_this->b[i] = K[i];
  }

  for (i = 0; i < 8; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_this->b[i + 320] = b_this->bn[i];
  }

  /*  Chance constraints */
  kidx = -1;
  for (j = 0; j < 32; j++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    A[j] = b_this->s_chance[j];
    for (i = 0; i <= 8; i += 2) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      _mm_storeu_pd(&K[(kidx + i) + 1], _mm_mul_pd(_mm_set1_pd(A[j]), _mm_set_pd
        (static_cast<real_T>(tmp_f[i + 1]), static_cast<real_T>(tmp_f[i]))));
    }

    kidx += 10;
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  d6 = b_this->s_chance[32];
  kidx = -1;
  for (i = 0; i <= 6; i += 2) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    K_0[i] = d6 * static_cast<real_T>(tmp_g[i]);
    K_0[i + 1] = static_cast<real_T>(tmp_g[i + 1]) * d6;
  }

  for (i = 0; i < 320; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_this->s_chance_vary[i] = K[i];
  }

  for (i = 0; i < 8; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_this->s_chance_vary[i + 320] = K_0[i];
  }

  for (j = 0; j < 32; j++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    A[j] = b_this->v_chance[j];
    for (i = 0; i <= 8; i += 2) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      _mm_storeu_pd(&K[(kidx + i) + 1], _mm_mul_pd(_mm_set1_pd(A[j]), _mm_set_pd
        (static_cast<real_T>(tmp_f[i + 1]), static_cast<real_T>(tmp_f[i]))));
    }

    kidx += 10;
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  d6 = b_this->v_chance[32];
  for (i = 0; i <= 6; i += 2) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    K_0[i] = d6 * static_cast<real_T>(tmp_g[i]);
    K_0[i + 1] = static_cast<real_T>(tmp_g[i + 1]) * d6;
  }

  for (i = 0; i < 320; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_this->v_chance_vary[i] = K[i];
  }

  for (i = 0; i < 8; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_this->v_chance_vary[i + 320] = K_0[i];
  }

  /* %% Error check the setup */
  /*  Cost function */
  /*  Standard linear constraints */
  /*  Softened constraints */
  /*  Chance constraints */
  /*  Binary constraints */
  /*  Decision variables */
  /* %% Report */
  /*  Return */
  return c_this;
}

void longitudinal_mpc::longitudinal__solver_initSolver(solver_longitudinal_mpc_T
  *b_this)
{
  /*         %% Internal solver functions */
  /*  Properties */
  /*  Settings */
  /*  GETSETTINGS structure constructor */
  /*  Message importance for printing to terminal */
  /*  Standard settings */
  /*  At most will need 2^n branches in a binary search */
  /*  Integral problem settings */
  /*  Maximum number of nodes to expand in search tree - correlate this with coder.varsize 'leaves' dimension in solve() method */
  /*  Initialize solver */
  std::memset(&b_this->Xk[0], 0, 36U * sizeof(real_T));
  std::memset(&b_this->Xopt[0], 0, 36U * sizeof(real_T));
  b_this->J = (rtInf);
  b_this->flag = flags::UNSOLVED;
}

/* Function for MATLAB Function: '<S1>/getReference' */
void longitudinal_mpc::longitudinal_mpc_interp1_h(const real_T varargin_1[201],
  const real_T varargin_2[201], const real_T varargin_3[81], real_T Vq[81])
{
  real_T x[201];
  real_T y[201];
  int32_T i;
  std::memcpy(&y[0], &varargin_2[0], 201U * sizeof(real_T));
  std::memcpy(&x[0], &varargin_1[0], 201U * sizeof(real_T));
  for (i = 0; i < 81; i++) {
    Vq[i] = (rtNaN);
  }

  i = 0;
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (i < 201) {
      if (std::isnan(varargin_1[i])) {
        exitg1 = 1;
      } else {
        i++;
      }
    } else {
      real_T xtmp;
      if (varargin_1[1] < varargin_1[0]) {
        for (i = 0; i < 100; i++) {
          xtmp = x[i];
          x[i] = x[200 - i];
          x[200 - i] = xtmp;
          xtmp = y[i];
          y[i] = y[200 - i];
          y[200 - i] = xtmp;
        }
      }

      for (i = 0; i < 81; i++) {
        Vq[i] = (rtNaN);
        xtmp = varargin_3[i];
        if (std::isnan(xtmp)) {
          Vq[i] = (rtNaN);
        } else if ((!(xtmp > x[200])) && (!(xtmp < x[0]))) {
          int32_T high_i;
          int32_T low_i;
          int32_T low_ip1;
          low_i = 1;
          low_ip1 = 2;
          high_i = 201;
          while (high_i > low_ip1) {
            int32_T mid_i;
            mid_i = (low_i + high_i) >> 1;
            if (varargin_3[i] >= x[mid_i - 1]) {
              low_i = mid_i;
              low_ip1 = mid_i + 1;
            } else {
              high_i = mid_i;
            }
          }

          xtmp = x[low_i - 1];
          xtmp = (varargin_3[i] - xtmp) / (x[low_i] - xtmp);
          if (xtmp == 0.0) {
            Vq[i] = y[low_i - 1];
          } else if (xtmp == 1.0) {
            Vq[i] = y[low_i];
          } else {
            real_T tmp;
            tmp = y[low_i - 1];
            if (tmp == y[low_i]) {
              Vq[i] = tmp;
            } else {
              Vq[i] = (1.0 - xtmp) * tmp + xtmp * y[low_i];
            }
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

boolean_T longitudinal_mpc::longitudinal_mpc_vectorAny(const boolean_T x_data[],
  const int32_T x_size[1])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k <= x_size[0] - 1)) {
    if (x_data[b_k]) {
      y = true;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  return y;
}

void longitudinal_mpc::longitudinal_mpc_pchip(const real_T x[81], const real_T
  y[81], real_T v_breaks[81], real_T v_coefs[320])
{
  __m128d tmp_3;
  real_T slopes[81];
  real_T del[80];
  real_T h[80];
  real_T u;
  real_T w1;
  real_T w2;
  for (int32_T b_k{0}; b_k <= 78; b_k += 2) {
    tmp_3 = _mm_sub_pd(_mm_loadu_pd(&x[b_k + 1]), _mm_loadu_pd(&x[b_k]));
    _mm_storeu_pd(&h[b_k], tmp_3);

    /* Start for MATLABSystem: '<S1>/MPC System' */
    _mm_storeu_pd(&del[b_k], _mm_div_pd(_mm_sub_pd(_mm_loadu_pd(&y[b_k + 1]),
      _mm_loadu_pd(&y[b_k])), tmp_3));
  }

  for (int32_T b_k{0}; b_k < 79; b_k++) {
    real_T del_0;
    real_T h_0;
    w2 = h[b_k + 1];
    h_0 = h[b_k];
    w1 = w2 * 2.0 + h_0;
    w2 += h_0 * 2.0;
    slopes[b_k + 1] = 0.0;

    /* Start for MATLABSystem: '<S1>/MPC System' */
    h_0 = del[b_k];
    del_0 = del[b_k + 1];
    u = h_0 * del_0;
    if (std::isnan(u)) {
      u = (rtNaN);
    } else if (u < 0.0) {
      u = -1.0;
    } else {
      u = (u > 0.0);
    }

    if (u > 0.0) {
      slopes[b_k + 1] = (w1 + w2) / (w1 / h_0 + w2 / del_0);
    }
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  w1 = ((2.0 * h[0] + h[1]) * del[0] - h[0] * del[1]) / (h[0] + h[1]);
  if (std::isnan(del[0])) {
    w2 = (rtNaN);
  } else if (del[0] < 0.0) {
    w2 = -1.0;
  } else {
    w2 = (del[0] > 0.0);
  }

  if (std::isnan(w1)) {
    u = (rtNaN);
  } else if (w1 < 0.0) {
    u = -1.0;
  } else {
    u = (w1 > 0.0);
  }

  if (u != w2) {
    w1 = 0.0;
  } else {
    if (std::isnan(del[1])) {
      u = (rtNaN);
    } else if (del[1] < 0.0) {
      u = -1.0;
    } else {
      u = (del[1] > 0.0);
    }

    if ((w2 != u) && (std::abs(w1) > std::abs(3.0 * del[0]))) {
      w1 = 3.0 * del[0];
    }
  }

  slopes[0] = w1;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  w1 = ((2.0 * h[79] + h[78]) * del[79] - del[78] * h[79]) / (h[78] + h[79]);
  if (std::isnan(del[79])) {
    w2 = (rtNaN);
  } else if (del[79] < 0.0) {
    w2 = -1.0;
  } else {
    w2 = (del[79] > 0.0);
  }

  if (std::isnan(w1)) {
    u = (rtNaN);
  } else if (w1 < 0.0) {
    u = -1.0;
  } else {
    u = (w1 > 0.0);
  }

  if (u != w2) {
    w1 = 0.0;
  } else {
    if (std::isnan(del[78])) {
      u = (rtNaN);
    } else if (del[78] < 0.0) {
      u = -1.0;
    } else {
      u = (del[78] > 0.0);
    }

    if ((w2 != u) && (std::abs(w1) > std::abs(3.0 * del[79]))) {
      w1 = 3.0 * del[79];
    }
  }

  slopes[80] = w1;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  std::memcpy(&v_breaks[0], &x[0], 81U * sizeof(real_T));
  for (int32_T b_k{0}; b_k <= 78; b_k += 2) {
    __m128d tmp;
    __m128d tmp_0;
    __m128d tmp_1;
    __m128d tmp_2;

    /* Start for MATLABSystem: '<S1>/MPC System' */
    tmp_3 = _mm_loadu_pd(&del[b_k]);
    tmp = _mm_loadu_pd(&slopes[b_k]);
    tmp_0 = _mm_loadu_pd(&h[b_k]);
    tmp_1 = _mm_div_pd(_mm_sub_pd(tmp_3, tmp), tmp_0);

    /* Start for MATLABSystem: '<S1>/MPC System' */
    tmp_2 = _mm_loadu_pd(&slopes[b_k + 1]);
    tmp_3 = _mm_div_pd(_mm_sub_pd(tmp_2, tmp_3), tmp_0);

    /* Start for MATLABSystem: '<S1>/MPC System' */
    _mm_storeu_pd(&v_coefs[b_k], _mm_div_pd(_mm_sub_pd(tmp_3, tmp_1), tmp_0));
    _mm_storeu_pd(&v_coefs[b_k + 80], _mm_sub_pd(_mm_mul_pd(_mm_set1_pd(2.0),
      tmp_1), tmp_3));
    _mm_storeu_pd(&v_coefs[b_k + 160], tmp);
    _mm_storeu_pd(&v_coefs[b_k + 240], _mm_loadu_pd(&y[b_k]));
  }
}

real_T longitudinal_mpc::longitudinal_mpc_ppval(const real_T pp_breaks[81],
  const real_T pp_coefs[320], real_T x)
{
  real_T v;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (std::isnan(x)) {
    v = (rtNaN);
  } else {
    int32_T high_i;
    int32_T low_i;
    int32_T low_ip1;
    low_i = 0;
    low_ip1 = 1;
    high_i = 81;
    while (high_i > low_ip1 + 1) {
      int32_T mid_i;
      mid_i = ((low_i + high_i) + 1) >> 1;
      if (x >= pp_breaks[mid_i - 1]) {
        low_i = mid_i - 1;
        low_ip1 = mid_i;
      } else {
        high_i = mid_i;
      }
    }

    real_T xloc;
    xloc = x - pp_breaks[low_i];
    v = ((xloc * pp_coefs[low_i] + pp_coefs[low_i + 80]) * xloc + pp_coefs[low_i
         + 160]) * xloc + pp_coefs[low_i + 240];
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
  return v;
}

void longitudinal_mpc::longitudinal_mpc_interp1_b(const real_T varargin_1[81],
  const real_T varargin_2[243], const real_T varargin_3[32], real_T Vq[96])
{
  real_T slopes[243];
  real_T yp[243];
  real_T del[240];
  real_T x[81];
  real_T h[80];
  real_T tmp_1[2];
  int32_T c1;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  std::memcpy(&slopes[0], &varargin_2[0], 243U * sizeof(real_T));
  std::memcpy(&x[0], &varargin_1[0], 81U * sizeof(real_T));
  std::memset(&Vq[0], 0, 96U * sizeof(real_T));
  c1 = 0;
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (c1 < 81) {
      if (std::isnan(varargin_1[c1])) {
        exitg1 = 1;
      } else {
        c1++;
      }
    } else {
      __m128d tmp;
      real_T d2;
      real_T divdifij;
      real_T w2;
      real_T xtmp;
      real_T xtmp_tmp;
      real_T xtmp_tmp_0;
      real_T yit_idx_1;
      int32_T c2;
      int32_T i;
      int32_T joffset;
      int32_T low_i;
      if (varargin_1[1] < varargin_1[0]) {
        for (joffset = 0; joffset < 40; joffset++) {
          xtmp = x[joffset];
          x[joffset] = x[80 - joffset];
          x[80 - joffset] = xtmp;
        }

        for (low_i = 0; low_i < 3; low_i++) {
          i = low_i * 81 - 1;
          for (c1 = 0; c1 < 40; c1++) {
            joffset = (c1 + i) + 1;
            xtmp = slopes[joffset];
            c2 = (i - c1) + 81;
            slopes[joffset] = slopes[c2];
            slopes[c2] = xtmp;
          }
        }
      }

      std::memset(&Vq[0], 0, 96U * sizeof(real_T));
      for (i = 0; i < 81; i++) {
        yp[3 * i] = slopes[i];
        yp[3 * i + 1] = slopes[i + 81];
        yp[3 * i + 2] = slopes[i + 162];
      }

      for (c1 = 0; c1 <= 78; c1 += 2) {
        __m128d tmp_0;
        tmp = _mm_loadu_pd(&x[c1 + 1]);
        tmp_0 = _mm_loadu_pd(&x[c1]);
        _mm_storeu_pd(&h[c1], _mm_sub_pd(tmp, tmp_0));
      }

      for (joffset = 0; joffset <= 78; joffset += 2) {
        i = (joffset + 1) * 3;
        yit_idx_1 = yp[i];
        c1 = (joffset + 2) * 3;
        tmp = _mm_div_pd(_mm_sub_pd(_mm_set_pd(yp[c1], yit_idx_1), _mm_set_pd
          (yit_idx_1, yp[joffset * 3])), _mm_loadu_pd(&h[joffset]));
        _mm_storeu_pd(&tmp_1[0], tmp);
        del[joffset * 3] = tmp_1[0];
        del[i] = tmp_1[1];
        yit_idx_1 = yp[i + 1];
        low_i = joffset * 3 + 1;
        tmp = _mm_div_pd(_mm_sub_pd(_mm_set_pd(yp[c1 + 1], yit_idx_1),
          _mm_set_pd(yit_idx_1, yp[low_i])), _mm_loadu_pd(&h[joffset]));
        _mm_storeu_pd(&tmp_1[0], tmp);
        del[low_i] = tmp_1[0];
        del[i + 1] = tmp_1[1];
        yit_idx_1 = yp[i + 2];
        low_i = joffset * 3 + 2;
        tmp = _mm_div_pd(_mm_sub_pd(_mm_set_pd(yp[c1 + 2], yit_idx_1),
          _mm_set_pd(yit_idx_1, yp[low_i])), _mm_loadu_pd(&h[joffset]));
        _mm_storeu_pd(&tmp_1[0], tmp);
        del[low_i] = tmp_1[0];
        del[i + 2] = tmp_1[1];
      }

      for (i = 0; i < 79; i++) {
        w2 = h[i + 1];
        yit_idx_1 = h[i];
        xtmp = w2 * 2.0 + yit_idx_1;
        w2 += 2.0 * yit_idx_1;
        c1 = i * 3 - 1;
        c2 = (i + 1) * 3 - 1;
        divdifij = del[c1 + 1];
        d2 = del[c2 + 1];
        slopes[c2 + 1] = 0.0;
        yit_idx_1 = divdifij * d2;
        if (std::isnan(yit_idx_1)) {
          yit_idx_1 = (rtNaN);
        } else if (yit_idx_1 < 0.0) {
          yit_idx_1 = -1.0;
        } else {
          yit_idx_1 = (yit_idx_1 > 0.0);
        }

        if (yit_idx_1 > 0.0) {
          slopes[c2 + 1] = (xtmp + w2) / (xtmp / divdifij + w2 / d2);
        }

        divdifij = del[c1 + 2];
        d2 = del[c2 + 2];
        slopes[c2 + 2] = 0.0;
        yit_idx_1 = divdifij * d2;
        if (std::isnan(yit_idx_1)) {
          yit_idx_1 = (rtNaN);
        } else if (yit_idx_1 < 0.0) {
          yit_idx_1 = -1.0;
        } else {
          yit_idx_1 = (yit_idx_1 > 0.0);
        }

        if (yit_idx_1 > 0.0) {
          slopes[c2 + 2] = (xtmp + w2) / (xtmp / divdifij + w2 / d2);
        }

        divdifij = del[c1 + 3];
        d2 = del[c2 + 3];
        slopes[c2 + 3] = 0.0;
        yit_idx_1 = divdifij * d2;
        if (std::isnan(yit_idx_1)) {
          yit_idx_1 = (rtNaN);
        } else if (yit_idx_1 < 0.0) {
          yit_idx_1 = -1.0;
        } else {
          yit_idx_1 = (yit_idx_1 > 0.0);
        }

        if (yit_idx_1 > 0.0) {
          slopes[c2 + 3] = (xtmp + w2) / (xtmp / divdifij + w2 / d2);
        }
      }

      divdifij = 2.0 * h[0] + h[1];
      d2 = h[0] + h[1];
      xtmp = (divdifij * del[0] - h[0] * del[3]) / d2;
      if (std::isnan(del[0])) {
        w2 = (rtNaN);
      } else if (del[0] < 0.0) {
        w2 = -1.0;
      } else {
        w2 = (del[0] > 0.0);
      }

      if (std::isnan(xtmp)) {
        yit_idx_1 = (rtNaN);
      } else if (xtmp < 0.0) {
        yit_idx_1 = -1.0;
      } else {
        yit_idx_1 = (xtmp > 0.0);
      }

      if (yit_idx_1 != w2) {
        xtmp = 0.0;
      } else {
        if (std::isnan(del[3])) {
          yit_idx_1 = (rtNaN);
        } else if (del[3] < 0.0) {
          yit_idx_1 = -1.0;
        } else {
          yit_idx_1 = (del[3] > 0.0);
        }

        if ((w2 != yit_idx_1) && (std::abs(xtmp) > std::abs(3.0 * del[0]))) {
          xtmp = 3.0 * del[0];
        }
      }

      slopes[0] = xtmp;
      xtmp_tmp = 2.0 * h[79] + h[78];
      xtmp_tmp_0 = h[78] + h[79];
      xtmp = (xtmp_tmp * del[237] - h[79] * del[234]) / xtmp_tmp_0;
      if (std::isnan(del[237])) {
        w2 = (rtNaN);
      } else if (del[237] < 0.0) {
        w2 = -1.0;
      } else {
        w2 = (del[237] > 0.0);
      }

      if (std::isnan(xtmp)) {
        yit_idx_1 = (rtNaN);
      } else if (xtmp < 0.0) {
        yit_idx_1 = -1.0;
      } else {
        yit_idx_1 = (xtmp > 0.0);
      }

      if (yit_idx_1 != w2) {
        xtmp = 0.0;
      } else {
        if (std::isnan(del[234])) {
          yit_idx_1 = (rtNaN);
        } else if (del[234] < 0.0) {
          yit_idx_1 = -1.0;
        } else {
          yit_idx_1 = (del[234] > 0.0);
        }

        if ((w2 != yit_idx_1) && (std::abs(xtmp) > std::abs(3.0 * del[237]))) {
          xtmp = 3.0 * del[237];
        }
      }

      slopes[240] = xtmp;
      xtmp = (divdifij * del[1] - h[0] * del[4]) / d2;
      if (std::isnan(del[1])) {
        w2 = (rtNaN);
      } else if (del[1] < 0.0) {
        w2 = -1.0;
      } else {
        w2 = (del[1] > 0.0);
      }

      if (std::isnan(xtmp)) {
        yit_idx_1 = (rtNaN);
      } else if (xtmp < 0.0) {
        yit_idx_1 = -1.0;
      } else {
        yit_idx_1 = (xtmp > 0.0);
      }

      if (yit_idx_1 != w2) {
        xtmp = 0.0;
      } else {
        if (std::isnan(del[4])) {
          yit_idx_1 = (rtNaN);
        } else if (del[4] < 0.0) {
          yit_idx_1 = -1.0;
        } else {
          yit_idx_1 = (del[4] > 0.0);
        }

        if ((w2 != yit_idx_1) && (std::abs(xtmp) > std::abs(3.0 * del[1]))) {
          xtmp = 3.0 * del[1];
        }
      }

      slopes[1] = xtmp;
      xtmp = (xtmp_tmp * del[238] - h[79] * del[235]) / xtmp_tmp_0;
      if (std::isnan(del[238])) {
        w2 = (rtNaN);
      } else if (del[238] < 0.0) {
        w2 = -1.0;
      } else {
        w2 = (del[238] > 0.0);
      }

      if (std::isnan(xtmp)) {
        yit_idx_1 = (rtNaN);
      } else if (xtmp < 0.0) {
        yit_idx_1 = -1.0;
      } else {
        yit_idx_1 = (xtmp > 0.0);
      }

      if (yit_idx_1 != w2) {
        xtmp = 0.0;
      } else {
        if (std::isnan(del[235])) {
          yit_idx_1 = (rtNaN);
        } else if (del[235] < 0.0) {
          yit_idx_1 = -1.0;
        } else {
          yit_idx_1 = (del[235] > 0.0);
        }

        if ((w2 != yit_idx_1) && (std::abs(xtmp) > std::abs(3.0 * del[238]))) {
          xtmp = 3.0 * del[238];
        }
      }

      slopes[241] = xtmp;
      xtmp = (divdifij * del[2] - h[0] * del[5]) / d2;
      if (std::isnan(del[2])) {
        w2 = (rtNaN);
      } else if (del[2] < 0.0) {
        w2 = -1.0;
      } else {
        w2 = (del[2] > 0.0);
      }

      if (std::isnan(xtmp)) {
        yit_idx_1 = (rtNaN);
      } else if (xtmp < 0.0) {
        yit_idx_1 = -1.0;
      } else {
        yit_idx_1 = (xtmp > 0.0);
      }

      if (yit_idx_1 != w2) {
        xtmp = 0.0;
      } else {
        if (std::isnan(del[5])) {
          yit_idx_1 = (rtNaN);
        } else if (del[5] < 0.0) {
          yit_idx_1 = -1.0;
        } else {
          yit_idx_1 = (del[5] > 0.0);
        }

        if ((w2 != yit_idx_1) && (std::abs(xtmp) > std::abs(3.0 * del[2]))) {
          xtmp = 3.0 * del[2];
        }
      }

      slopes[2] = xtmp;
      xtmp = (xtmp_tmp * del[239] - h[79] * del[236]) / xtmp_tmp_0;
      if (std::isnan(del[239])) {
        w2 = (rtNaN);
      } else if (del[239] < 0.0) {
        w2 = -1.0;
      } else {
        w2 = (del[239] > 0.0);
      }

      if (std::isnan(xtmp)) {
        yit_idx_1 = (rtNaN);
      } else if (xtmp < 0.0) {
        yit_idx_1 = -1.0;
      } else {
        yit_idx_1 = (xtmp > 0.0);
      }

      if (yit_idx_1 != w2) {
        xtmp = 0.0;
      } else {
        if (std::isnan(del[236])) {
          yit_idx_1 = (rtNaN);
        } else if (del[236] < 0.0) {
          yit_idx_1 = -1.0;
        } else {
          yit_idx_1 = (del[236] > 0.0);
        }

        if ((w2 != yit_idx_1) && (std::abs(xtmp) > std::abs(3.0 * del[239]))) {
          xtmp = 3.0 * del[239];
        }
      }

      slopes[242] = xtmp;
      for (i = 0; i < 80; i++) {
        joffset = i * 3 - 1;
        divdifij = del[joffset + 1];
        xtmp = slopes[joffset + 1];
        w2 = (divdifij - xtmp) / h[i];
        divdifij = (slopes[joffset + 4] - divdifij) / h[i];
        longitudinal_mpc_B.pp_coefs[joffset + 1] = (divdifij - w2) / h[i];
        longitudinal_mpc_B.pp_coefs[joffset + 241] = 2.0 * w2 - divdifij;
        longitudinal_mpc_B.pp_coefs[joffset + 481] = xtmp;
        longitudinal_mpc_B.pp_coefs[joffset + 721] = yp[joffset + 1];
        divdifij = del[joffset + 2];
        xtmp = slopes[joffset + 2];
        w2 = (divdifij - xtmp) / h[i];
        divdifij = (slopes[joffset + 5] - divdifij) / h[i];
        longitudinal_mpc_B.pp_coefs[joffset + 2] = (divdifij - w2) / h[i];
        longitudinal_mpc_B.pp_coefs[joffset + 242] = 2.0 * w2 - divdifij;
        longitudinal_mpc_B.pp_coefs[joffset + 482] = xtmp;
        longitudinal_mpc_B.pp_coefs[joffset + 722] = yp[joffset + 2];
        divdifij = del[joffset + 3];
        xtmp = slopes[joffset + 3];
        w2 = (divdifij - xtmp) / h[i];
        divdifij = (slopes[joffset + 6] - divdifij) / h[i];
        longitudinal_mpc_B.pp_coefs[joffset + 3] = (divdifij - w2) / h[i];
        longitudinal_mpc_B.pp_coefs[joffset + 243] = 2.0 * w2 - divdifij;
        longitudinal_mpc_B.pp_coefs[joffset + 483] = xtmp;
        longitudinal_mpc_B.pp_coefs[joffset + 723] = yp[joffset + 3];
      }

      for (c1 = 0; c1 < 32; c1++) {
        xtmp = varargin_3[c1];
        if (std::isnan(xtmp)) {
          Vq[c1] = (rtNaN);
          Vq[c1 + 32] = (rtNaN);
          Vq[c1 + 64] = (rtNaN);
        } else {
          low_i = 0;
          i = 1;
          c2 = 81;
          while (c2 > i + 1) {
            joffset = ((low_i + c2) + 1) >> 1;
            if (xtmp >= x[joffset - 1]) {
              low_i = joffset - 1;
              i = joffset;
            } else {
              c2 = joffset;
            }
          }

          i = low_i * 3;
          xtmp = varargin_3[c1] - x[low_i];
          w2 = longitudinal_mpc_B.pp_coefs[i];
          yit_idx_1 = longitudinal_mpc_B.pp_coefs[i + 1];
          divdifij = longitudinal_mpc_B.pp_coefs[i + 2];
          for (joffset = 0; joffset < 3; joffset++) {
            low_i = ((joffset + 1) * 240 + i) - 1;
            tmp = _mm_add_pd(_mm_mul_pd(_mm_set1_pd(xtmp), _mm_set_pd(yit_idx_1,
              w2)), _mm_loadu_pd(&longitudinal_mpc_B.pp_coefs[low_i + 1]));
            _mm_storeu_pd(&tmp_1[0], tmp);
            w2 = tmp_1[0];
            yit_idx_1 = tmp_1[1];
            divdifij = xtmp * divdifij + longitudinal_mpc_B.pp_coefs[low_i + 3];
          }

          Vq[c1] = w2;
          Vq[c1 + 32] = yit_idx_1;
          Vq[c1 + 64] = divdifij;
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

void longitudinal_mpc::longitudinal_m_factoryConstruct
  (skczSSN0IseekIuhVW4znFG_longi_T *obj)
{
  obj->ldm = 37;
  obj->ndims = 0;
  obj->info = 0;
  obj->scaleFactor = 1.0;
  obj->ConvexCheck = true;
  obj->regTol_ = 0.0;
}

real_T longitudinal_mpc::longitudinal_mpc_xnrm2(int32_T n, const real_T x[12321],
  int32_T ix0)
{
  real_T y;
  y = 0.0;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (int32_T k{ix0}; k < kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = std::sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = std::sqrt(b * b + 1.0) * a;
  } else if (std::isnan(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

void longitudinal_mpc::longitudinal_mpc_xzlarfg(int32_T n, real_T alpha1, real_T
  x[12321], int32_T ix0, real_T *b_alpha1, real_T *tau)
{
  __m128d tmp;
  real_T beta1;
  int32_T k;
  int32_T knt;
  int32_T scalarLB;
  int32_T vectorUB;
  int32_T vectorUB_tmp;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  *b_alpha1 = alpha1;
  *tau = 0.0;
  if (n > 0) {
    beta1 = longitudinal_mpc_xnrm2(n - 1, x, ix0);
    if (beta1 != 0.0) {
      beta1 = rt_hypotd_snf(alpha1, beta1);
      if (alpha1 >= 0.0) {
        beta1 = -beta1;
      }

      if (std::abs(beta1) < 1.0020841800044864E-292) {
        knt = -1;
        do {
          knt++;
          scalarLB = ix0 + n;
          vectorUB = ((((scalarLB - ix0) - 1) / 2) << 1) + ix0;
          vectorUB_tmp = vectorUB - 2;
          for (k = ix0; k <= vectorUB_tmp; k += 2) {
            tmp = _mm_loadu_pd(&x[k - 1]);
            _mm_storeu_pd(&x[k - 1], _mm_mul_pd(tmp, _mm_set1_pd
              (9.9792015476736E+291)));
          }

          for (k = vectorUB; k <= scalarLB - 2; k++) {
            x[k - 1] *= 9.9792015476736E+291;
          }

          beta1 *= 9.9792015476736E+291;
          *b_alpha1 *= 9.9792015476736E+291;
        } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt + 1 < 20));

        beta1 = rt_hypotd_snf(*b_alpha1, longitudinal_mpc_xnrm2(n - 1, x, ix0));
        if (*b_alpha1 >= 0.0) {
          beta1 = -beta1;
        }

        *tau = (beta1 - *b_alpha1) / beta1;
        *b_alpha1 = 1.0 / (*b_alpha1 - beta1);
        for (k = ix0; k <= vectorUB_tmp; k += 2) {
          tmp = _mm_loadu_pd(&x[k - 1]);
          _mm_storeu_pd(&x[k - 1], _mm_mul_pd(tmp, _mm_set1_pd(*b_alpha1)));
        }

        for (k = vectorUB; k <= scalarLB - 2; k++) {
          x[k - 1] *= *b_alpha1;
        }

        for (k = 0; k <= knt; k++) {
          beta1 *= 1.0020841800044864E-292;
        }

        *b_alpha1 = beta1;
      } else {
        *tau = (beta1 - alpha1) / beta1;
        *b_alpha1 = 1.0 / (alpha1 - beta1);
        knt = ix0 + n;
        scalarLB = ((((knt - ix0) - 1) / 2) << 1) + ix0;
        vectorUB = scalarLB - 2;
        for (k = ix0; k <= vectorUB; k += 2) {
          tmp = _mm_loadu_pd(&x[k - 1]);
          _mm_storeu_pd(&x[k - 1], _mm_mul_pd(tmp, _mm_set1_pd(*b_alpha1)));
        }

        for (k = scalarLB; k <= knt - 2; k++) {
          x[k - 1] *= *b_alpha1;
        }

        *b_alpha1 = beta1;
      }
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_xzlarf(int32_T m, int32_T n, int32_T iv0,
  real_T tau, real_T C[12321], int32_T ic0, real_T work[333])
{
  int32_T coltop;
  int32_T ia;
  int32_T lastc;
  int32_T lastv;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (tau != 0.0) {
    boolean_T exitg2;
    lastv = m;
    lastc = (iv0 + m) - 2;
    while ((lastv > 0) && (C[lastc] == 0.0)) {
      lastv--;
      lastc--;
    }

    lastc = n - 1;
    exitg2 = false;
    while ((!exitg2) && (lastc + 1 > 0)) {
      int32_T exitg1;
      coltop = lastc * 37 + ic0;
      ia = coltop;
      do {
        exitg1 = 0;
        if (ia <= (coltop + lastv) - 1) {
          if (C[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = -1;
  }

  if (lastv > 0) {
    real_T c;
    int32_T jy;
    if (lastc + 1 != 0) {
      if (lastc >= 0) {
        std::memset(&work[0], 0, static_cast<uint32_T>(lastc + 1) * sizeof
                    (real_T));
      }

      coltop = 37 * lastc + ic0;
      for (int32_T iac{ic0}; iac <= coltop; iac += 37) {
        c = 0.0;
        jy = iac + lastv;
        for (ia = iac; ia < jy; ia++) {
          c += C[((iv0 + ia) - iac) - 1] * C[ia - 1];
        }

        ia = div_nde_s32_floor(iac - ic0, 37);
        work[ia] += c;
      }
    }

    if (!(-tau == 0.0)) {
      coltop = ic0;
      for (ia = 0; ia <= lastc; ia++) {
        c = work[ia];
        if (c != 0.0) {
          c *= -tau;
          jy = (lastv + coltop) - 1;
          for (int32_T iac{coltop}; iac <= jy; iac++) {
            C[iac - 1] += C[((iv0 + iac) - coltop) - 1] * c;
          }
        }

        coltop += 37;
      }
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_qrf(real_T A[12321], int32_T m, int32_T
  n, int32_T nfxd, real_T tau[37])
{
  real_T work[333];
  real_T b_atmp;
  int32_T c;
  int32_T i;
  int32_T ii;
  int32_T mmi;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  std::memset(&work[0], 0, 333U * sizeof(real_T));
  c = static_cast<uint16_T>(nfxd) - 1;
  for (i = 0; i <= c; i++) {
    ii = i * 37 + i;
    mmi = m - i;
    if (i + 1 < m) {
      longitudinal_mpc_xzlarfg(mmi, A[ii], A, ii + 2, &b_atmp, &tau[i]);
      A[ii] = b_atmp;
    } else {
      tau[i] = 0.0;
    }

    if (i + 1 < n) {
      b_atmp = A[ii];
      A[ii] = 1.0;
      longitudinal_mpc_xzlarf(mmi, (n - i) - 1, ii + 1, tau[i], A, ii + 38, work);
      A[ii] = b_atmp;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_factorQRE(const
  sCCE9T0P8IQkk6PxUdCnbBE_longi_T *obj, int32_T mrows, int32_T ncols,
  sCCE9T0P8IQkk6PxUdCnbBE_longi_T *b_obj)
{
  real_T vn1[333];
  real_T vn2[333];
  real_T work[333];
  real_T s;
  real_T t;
  real_T temp;
  real_T vn1_0;
  int32_T b_obj_tmp;
  int32_T i;
  int32_T ii;
  int32_T ip1;
  int32_T itemp;
  int32_T iy;
  int32_T mmi;
  int32_T nfxd;
  int32_T nmi;
  int32_T pvt;
  int32_T tmp;
  static const int32_T offsets[4]{ 0, 1, 2, 3 };

  int32_T temp_tmp;
  *b_obj = *obj;
  if (mrows * ncols == 0) {
    b_obj->mrows = mrows;
    b_obj->ncols = ncols;
    b_obj->minRowCol = 0;
  } else {
    b_obj->usedPivoting = true;
    b_obj->mrows = mrows;
    b_obj->ncols = ncols;

    /* Start for MATLABSystem: '<S1>/MPC System' */
    if (mrows <= ncols) {
      b_obj_tmp = mrows;
    } else {
      b_obj_tmp = ncols;
    }

    b_obj->minRowCol = b_obj_tmp;
    std::memcpy(&b_obj->jpvt[0], &obj->jpvt[0], 333U * sizeof(int32_T));
    std::memcpy(&b_obj->QR[0], &obj->QR[0], 12321U * sizeof(real_T));
    std::memset(&b_obj->tau[0], 0, 37U * sizeof(real_T));
    if (b_obj_tmp < 1) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      b_obj_tmp = (ncols / 4) << 2;
      nfxd = b_obj_tmp - 4;
      for (i = 0; i <= nfxd; i += 4) {
        _mm_storeu_si128((__m128i *)&b_obj->jpvt[i], _mm_add_epi32(_mm_add_epi32
          (_mm_set1_epi32(i), _mm_loadu_si128((const __m128i *)&offsets[0])),
          _mm_set1_epi32(1)));
      }

      /* Start for MATLABSystem: '<S1>/MPC System' */
      for (i = b_obj_tmp; i < ncols; i++) {
        b_obj->jpvt[i] = i + 1;
      }
    } else {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      nfxd = -1;
      for (ip1 = 0; ip1 < ncols; ip1++) {
        if (b_obj->jpvt[ip1] != 0) {
          nfxd++;
          if (ip1 + 1 != nfxd + 1) {
            i = ip1 * 37;
            iy = nfxd * 37;
            for (itemp = 0; itemp < mrows; itemp++) {
              temp_tmp = i + itemp;
              temp = b_obj->QR[temp_tmp];
              tmp = iy + itemp;
              b_obj->QR[temp_tmp] = b_obj->QR[tmp];
              b_obj->QR[tmp] = temp;
            }

            b_obj->jpvt[ip1] = b_obj->jpvt[nfxd];
            b_obj->jpvt[nfxd] = ip1 + 1;
          } else {
            b_obj->jpvt[ip1] = ip1 + 1;
          }
        } else {
          b_obj->jpvt[ip1] = ip1 + 1;
        }
      }

      if (nfxd + 1 <= b_obj_tmp) {
        nfxd++;
      } else {
        nfxd = b_obj_tmp;
      }

      std::memset(&b_obj->tau[0], 0, 37U * sizeof(real_T));
      longitudinal_mpc_qrf(b_obj->QR, mrows, ncols, nfxd, b_obj->tau);
      if (nfxd < b_obj_tmp) {
        std::memset(&work[0], 0, 333U * sizeof(real_T));
        std::memset(&vn1[0], 0, 333U * sizeof(real_T));
        std::memset(&vn2[0], 0, 333U * sizeof(real_T));

        /* Start for MATLABSystem: '<S1>/MPC System' */
        for (ip1 = nfxd + 1; ip1 <= ncols; ip1++) {
          vn1_0 = longitudinal_mpc_xnrm2(mrows - nfxd, b_obj->QR, ((ip1 - 1) *
            37 + nfxd) + 1);
          vn1[ip1 - 1] = vn1_0;
          vn2[ip1 - 1] = vn1_0;
        }

        ip1 = nfxd;
        for (nfxd = ip1 + 1; nfxd <= b_obj_tmp; nfxd++) {
          /* Start for MATLABSystem: '<S1>/MPC System' */
          iy = (nfxd - 1) * 37;
          ii = (iy + nfxd) - 1;
          nmi = (ncols - nfxd) + 1;
          mmi = mrows - nfxd;
          if (nmi < 1) {
            pvt = -1;
          } else {
            pvt = 0;
            if (nmi > 1) {
              temp = std::abs(vn1[nfxd - 1]);
              for (itemp = 2; itemp <= nmi; itemp++) {
                s = std::abs(vn1[(nfxd + itemp) - 2]);
                if (s > temp) {
                  pvt = itemp - 1;
                  temp = s;
                }
              }
            }
          }

          pvt = (nfxd + pvt) - 1;
          if (pvt + 1 != nfxd) {
            i = pvt * 37;
            for (itemp = 0; itemp < mrows; itemp++) {
              temp_tmp = i + itemp;
              temp = b_obj->QR[temp_tmp];
              tmp = iy + itemp;
              b_obj->QR[temp_tmp] = b_obj->QR[tmp];
              b_obj->QR[tmp] = temp;
            }

            itemp = b_obj->jpvt[pvt];
            b_obj->jpvt[pvt] = b_obj->jpvt[nfxd - 1];
            b_obj->jpvt[nfxd - 1] = itemp;
            vn1[pvt] = vn1[nfxd - 1];
            vn2[pvt] = vn2[nfxd - 1];
          }

          if (nfxd < mrows) {
            longitudinal_mpc_xzlarfg(mmi + 1, b_obj->QR[ii], b_obj->QR, ii + 2,
              &temp, &b_obj->tau[nfxd - 1]);
            b_obj->QR[ii] = temp;
          } else {
            b_obj->tau[nfxd - 1] = 0.0;
          }

          if (nfxd < ncols) {
            temp = b_obj->QR[ii];
            b_obj->QR[ii] = 1.0;
            longitudinal_mpc_xzlarf(mmi + 1, nmi - 1, ii + 1, b_obj->tau[nfxd -
              1], b_obj->QR, ii + 38, work);
            b_obj->QR[ii] = temp;
          }

          for (i = nfxd + 1; i <= ncols; i++) {
            itemp = (i - 1) * 37 + nfxd;
            vn1_0 = vn1[i - 1];
            if (vn1_0 != 0.0) {
              temp = std::abs(b_obj->QR[itemp - 1]) / vn1_0;
              temp = 1.0 - temp * temp;
              if (temp < 0.0) {
                temp = 0.0;
              }

              s = vn1_0 / vn2[i - 1];
              s = s * s * temp;
              if (s <= 1.4901161193847656E-8) {
                if (nfxd < mrows) {
                  iy = itemp;
                  temp = 0.0;
                  if (mmi >= 1) {
                    if (mmi == 1) {
                      temp = std::abs(b_obj->QR[itemp]);
                    } else {
                      s = 3.3121686421112381E-170;
                      ii = itemp + mmi;
                      for (itemp = iy + 1; itemp <= ii; itemp++) {
                        vn1_0 = std::abs(b_obj->QR[itemp - 1]);
                        if (vn1_0 > s) {
                          t = s / vn1_0;
                          temp = temp * t * t + 1.0;
                          s = vn1_0;
                        } else {
                          t = vn1_0 / s;
                          temp += t * t;
                        }
                      }

                      temp = s * std::sqrt(temp);
                    }
                  }

                  vn1[i - 1] = temp;
                  vn2[i - 1] = temp;
                } else {
                  vn1[i - 1] = 0.0;
                  vn2[i - 1] = 0.0;
                }
              } else {
                vn1[i - 1] = vn1_0 * std::sqrt(temp);
              }
            }
          }
        }
      }
    }
  }
}

void longitudinal_mpc::longitudinal_mpc_countsort(int32_T x[333], int32_T xLen,
  int32_T workspace[333], int32_T xMin, int32_T xMax)
{
  /* Start for MATLABSystem: '<S1>/MPC System' */
  if ((xLen > 1) && (xMax > xMin)) {
    int32_T c;
    int32_T idxEnd;
    int32_T idxFill;
    int32_T idxStart;
    int32_T maxOffset;
    maxOffset = xMax - xMin;
    if (maxOffset >= 0) {
      std::memset(&workspace[0], 0, static_cast<uint32_T>(maxOffset + 1) *
                  sizeof(int32_T));
    }

    for (idxStart = 0; idxStart < xLen; idxStart++) {
      idxFill = x[idxStart] - xMin;
      workspace[idxFill]++;
    }

    for (idxStart = 2; idxStart <= maxOffset + 1; idxStart++) {
      workspace[idxStart - 1] += workspace[idxStart - 2];
    }

    idxStart = 1;
    idxEnd = workspace[0];
    c = maxOffset - 1;
    for (maxOffset = 0; maxOffset <= c; maxOffset++) {
      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x[idxFill - 1] = maxOffset + xMin;
      }

      idxStart = workspace[maxOffset] + 1;
      idxEnd = workspace[maxOffset + 1];
    }

    for (maxOffset = idxStart; maxOffset <= idxEnd; maxOffset++) {
      x[maxOffset - 1] = xMax;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_removeConstr
  (s1rlrl8wYvAiB01zuzfeYY_longit_T *obj, int32_T idx_global)
{
  int32_T TYPE_tmp;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  TYPE_tmp = obj->Wid[idx_global - 1] - 1;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE_tmp] + obj->Wlocalidx[idx_global -
                       1]) - 2] = false;
  if (idx_global < obj->nActiveConstr) {
    int32_T c;
    obj->Wid[idx_global - 1] = obj->Wid[obj->nActiveConstr - 1];
    obj->Wlocalidx[idx_global - 1] = obj->Wlocalidx[obj->nActiveConstr - 1];
    c = static_cast<uint8_T>(obj->nVar) - 1;
    for (int32_T b_idx{0}; b_idx <= c; b_idx++) {
      obj->ATwset[b_idx + 37 * (idx_global - 1)] = obj->ATwset
        [(obj->nActiveConstr - 1) * 37 + b_idx];
    }

    obj->bwset[idx_global - 1] = obj->bwset[obj->nActiveConstr - 1];
  }

  obj->nActiveConstr--;
  obj->nWConstr[TYPE_tmp]--;

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudin_RemoveDependentIneq_
  (s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset, sCCE9T0P8IQkk6PxUdCnbBE_longi_T *
   qrmanager, sK6ng1KsrjtpGD3SgQmbb8_longit_T *memspace, real_T tolfactor)
{
  real_T tol;
  int32_T idxPosQR_tmp;
  int32_T nActiveConstr;
  int32_T nDepIneq;
  int32_T nFixedConstr;
  int32_T nVar;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  nFixedConstr = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
  nVar = workingset->nVar;
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    tol = tolfactor * static_cast<real_T>(workingset->nVar) *
      2.2204460492503131E-16;
    for (nDepIneq = 0; nDepIneq <= nFixedConstr - 2; nDepIneq++) {
      qrmanager->jpvt[nDepIneq] = 1;
    }

    if (nFixedConstr <= workingset->nActiveConstr) {
      std::memset(&qrmanager->jpvt[nFixedConstr + -1], 0, static_cast<uint32_T>
                  ((workingset->nActiveConstr - nFixedConstr) + 1) * sizeof
                  (int32_T));
    }

    nActiveConstr = workingset->nActiveConstr - 1;
    for (nDepIneq = 0; nDepIneq <= nActiveConstr; nDepIneq++) {
      idxPosQR_tmp = 37 * nDepIneq - 1;
      std::memcpy(&qrmanager->QR[idxPosQR_tmp + 1], &workingset->
                  ATwset[idxPosQR_tmp + 1], static_cast<uint32_T>((((
        static_cast<uint8_T>(nVar) - 1) + idxPosQR_tmp) - idxPosQR_tmp) + 1) *
                  sizeof(real_T));
    }

    longitudinal_mpc_B.qrmanager = *qrmanager;
    longitudinal_mpc_factorQRE(&longitudinal_mpc_B.qrmanager, workingset->nVar,
      workingset->nActiveConstr, qrmanager);
    nDepIneq = -1;
    for (nActiveConstr = workingset->nActiveConstr; nActiveConstr > nVar;
         nActiveConstr--) {
      nDepIneq++;
      memspace->workspace_int[nDepIneq] = qrmanager->jpvt[nActiveConstr - 1];
    }

    if (nActiveConstr <= workingset->nVar) {
      nVar = ((nActiveConstr - 1) * 37 + nActiveConstr) - 1;
      while ((nActiveConstr > nFixedConstr - 1) && (std::abs(qrmanager->QR[nVar])
              < tol)) {
        nDepIneq++;
        memspace->workspace_int[nDepIneq] = qrmanager->jpvt[nActiveConstr - 1];
        nActiveConstr--;
        nVar -= 38;
      }
    }

    longitudinal_mpc_countsort(memspace->workspace_int, nDepIneq + 1,
      memspace->workspace_sort, nFixedConstr, workingset->nActiveConstr);
    for (nFixedConstr = nDepIneq + 1; nFixedConstr >= 1; nFixedConstr--) {
      longitudinal_mpc_removeConstr(workingset, memspace->
        workspace_int[nFixedConstr - 1]);
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_factorQR(sCCE9T0P8IQkk6PxUdCnbBE_longi_T
  *obj, const real_T A[12321], int32_T mrows, int32_T ncols)
{
  int32_T b_idx;
  int32_T i;
  static const int32_T offsets[4]{ 0, 1, 2, 3 };

  int32_T vectorUB;
  boolean_T guard1;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  vectorUB = mrows * ncols;
  guard1 = false;
  if (vectorUB > 0) {
    for (b_idx = 0; b_idx < ncols; b_idx++) {
      vectorUB = 37 * b_idx - 1;
      std::memcpy(&obj->QR[vectorUB + 1], &A[vectorUB + 1], static_cast<uint32_T>
                  ((((static_cast<uint8_T>(mrows) - 1) + vectorUB) - vectorUB) +
                   1) * sizeof(real_T));
    }

    guard1 = true;
  } else if (vectorUB == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    i = (ncols / 4) << 2;
    vectorUB = i - 4;
    for (b_idx = 0; b_idx <= vectorUB; b_idx += 4) {
      _mm_storeu_si128((__m128i *)&obj->jpvt[b_idx], _mm_add_epi32(_mm_add_epi32
        (_mm_set1_epi32(b_idx), _mm_loadu_si128((const __m128i *)&offsets[0])),
        _mm_set1_epi32(1)));
    }

    for (b_idx = i; b_idx < ncols; b_idx++) {
      obj->jpvt[b_idx] = b_idx + 1;
    }

    if (mrows <= ncols) {
      b_idx = mrows;
    } else {
      b_idx = ncols;
    }

    obj->minRowCol = b_idx;
    std::memset(&obj->tau[0], 0, 37U * sizeof(real_T));
    if (b_idx >= 1) {
      std::memset(&obj->tau[0], 0, 37U * sizeof(real_T));
      longitudinal_mpc_qrf(obj->QR, mrows, ncols, b_idx, obj->tau);
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_computeQ_
  (sCCE9T0P8IQkk6PxUdCnbBE_longi_T *obj, int32_T nrows)
{
  real_T work[37];
  int32_T b_idx;
  int32_T i;
  int32_T iQR0;
  int32_T lastv;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  lastv = obj->minRowCol - 1;
  for (b_idx = 0; b_idx <= lastv; b_idx++) {
    iQR0 = 37 * b_idx + b_idx;
    i = (obj->mrows - b_idx) - 2;
    if (i >= 0) {
      std::memcpy(&obj->Q[iQR0 + 1], &obj->QR[iQR0 + 1], static_cast<uint32_T>
                  (((i + iQR0) - iQR0) + 1) * sizeof(real_T));
    }
  }

  b_idx = obj->mrows;
  lastv = obj->minRowCol;
  if (nrows >= 1) {
    int32_T itau;
    for (itau = lastv; itau < nrows; itau++) {
      iQR0 = itau * 37;
      std::memset(&obj->Q[iQR0], 0, static_cast<uint32_T>((b_idx + iQR0) - iQR0)
                  * sizeof(real_T));
      obj->Q[iQR0 + itau] = 1.0;
    }

    itau = obj->minRowCol - 1;
    std::memset(&work[0], 0, 37U * sizeof(real_T));
    for (i = obj->minRowCol; i >= 1; i--) {
      int32_T i_0;
      int32_T iaii;
      int32_T jA;
      iaii = (i - 1) * 37 + i;
      if (i < nrows) {
        int32_T ia;
        int32_T lastc;
        obj->Q[iaii - 1] = 1.0;
        lastv = b_idx - i;
        iQR0 = iaii + 37;
        if (obj->tau[itau] != 0.0) {
          boolean_T exitg2;
          i_0 = (iaii + lastv) - 1;
          while ((lastv + 1 > 0) && (obj->Q[i_0] == 0.0)) {
            lastv--;
            i_0--;
          }

          lastc = (nrows - i) - 1;
          exitg2 = false;
          while ((!exitg2) && (lastc + 1 > 0)) {
            int32_T exitg1;
            i_0 = (lastc * 37 + iaii) + 37;
            ia = i_0;
            do {
              exitg1 = 0;
              if (ia <= i_0 + lastv) {
                if (obj->Q[ia - 1] != 0.0) {
                  exitg1 = 1;
                } else {
                  ia++;
                }
              } else {
                lastc--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = -1;
          lastc = -1;
        }

        if (lastv + 1 > 0) {
          real_T c;
          if (lastc + 1 != 0) {
            if (lastc >= 0) {
              std::memset(&work[0], 0, static_cast<uint32_T>(lastc + 1) * sizeof
                          (real_T));
            }

            i_0 = (37 * lastc + iaii) + 37;
            for (int32_T iac{iQR0}; iac <= i_0; iac += 37) {
              c = 0.0;
              jA = iac + lastv;
              for (ia = iac; ia <= jA; ia++) {
                c += obj->Q[((iaii + ia) - iac) - 1] * obj->Q[ia - 1];
              }

              ia = div_nde_s32_floor((iac - iaii) - 37, 37);
              work[ia] += c;
            }
          }

          if (!(-obj->tau[itau] == 0.0)) {
            jA = iaii;
            for (ia = 0; ia <= lastc; ia++) {
              c = work[ia];
              if (c != 0.0) {
                c *= -obj->tau[itau];
                i_0 = jA + 37;
                iQR0 = (lastv + jA) + 37;
                for (int32_T iac{i_0}; iac <= iQR0; iac++) {
                  obj->Q[iac - 1] += obj->Q[((iaii + iac) - jA) - 38] * c;
                }
              }

              jA += 37;
            }
          }
        }
      }

      if (i < b_idx) {
        i_0 = (iaii + b_idx) - i;
        for (lastv = iaii + 1; lastv <= i_0; lastv++) {
          obj->Q[lastv - 1] *= -obj->tau[itau];
        }
      }

      obj->Q[iaii - 1] = 1.0 - obj->tau[itau];
      jA = i - 2;
      for (i_0 = 0; i_0 <= jA; i_0++) {
        obj->Q[(iaii - i_0) - 2] = 0.0;
      }

      itau--;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_xgemv(int32_T m, const real_T A[12284],
  const real_T x[12321], real_T y[333])
{
  /* Start for MATLABSystem: '<S1>/MPC System' */
  for (int32_T b_iy{0}; b_iy <= 330; b_iy += 2) {
    __m128d tmp;
    tmp = _mm_loadu_pd(&y[b_iy]);
    _mm_storeu_pd(&y[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
  }

  for (int32_T b_iy{0}; b_iy <= 12247; b_iy += 37) {
    real_T c;
    int32_T e;
    int32_T ia;
    c = 0.0;
    e = b_iy + m;
    for (ia = b_iy + 1; ia <= e; ia++) {
      c += x[(ia - b_iy) - 1] * A[ia - 1];
    }

    ia = div_nde_s32_floor(b_iy, 37);
    y[ia] += c;
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitud_maxConstraintViolation(const
  s1rlrl8wYvAiB01zuzfeYY_longit_T *obj, const real_T x[12321], real_T *v,
  s1rlrl8wYvAiB01zuzfeYY_longit_T *b_obj)
{
  real_T b_obj_0;
  int32_T b_k;
  int32_T k;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (obj->probType == 2) {
    *b_obj = *obj;
    *v = 0.0;
    std::memcpy(&b_obj->maxConstrWorkspace[0], &b_obj->bineq[0], 332U * sizeof
                (real_T));
    longitudinal_mpc_xgemv(36, b_obj->Aineq, x, b_obj->maxConstrWorkspace);
    for (b_k = 0; b_k < 332; b_k++) {
      b_obj_0 = b_obj->maxConstrWorkspace[b_k] - x[b_k + 36];
      b_obj->maxConstrWorkspace[b_k] = b_obj_0;
      *v = std::fmax(*v, b_obj_0);
    }
  } else {
    *b_obj = *obj;
    *v = 0.0;
    std::memcpy(&b_obj->maxConstrWorkspace[0], &b_obj->bineq[0], 332U * sizeof
                (real_T));
    longitudinal_mpc_xgemv(b_obj->nVar, b_obj->Aineq, x,
      b_obj->maxConstrWorkspace);
    for (b_k = 0; b_k < 332; b_k++) {
      *v = std::fmax(*v, b_obj->maxConstrWorkspace[b_k]);
    }
  }

  if (obj->sizes[3] > 0) {
    k = static_cast<uint16_T>(obj->sizes[3]) - 1;
    for (b_k = 0; b_k <= k; b_k++) {
      *v = std::fmax(*v, -x[b_obj->indexLB[b_k] - 1]);
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_xgemv_b(int32_T m, const real_T A[12284],
  const real_T x[12321], real_T y[333])
{
  /* Start for MATLABSystem: '<S1>/MPC System' */
  for (int32_T b_iy{0}; b_iy <= 330; b_iy += 2) {
    __m128d tmp;
    tmp = _mm_loadu_pd(&y[b_iy]);
    _mm_storeu_pd(&y[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
  }

  for (int32_T b_iy{0}; b_iy <= 12247; b_iy += 37) {
    real_T c;
    int32_T e;
    int32_T ia;
    c = 0.0;
    e = b_iy + m;
    for (ia = b_iy + 1; ia <= e; ia++) {
      c += x[(ia - b_iy) + 332] * A[ia - 1];
    }

    ia = div_nde_s32_floor(b_iy, 37);
    y[ia] += c;
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longit_maxConstraintViolation_b(const
  s1rlrl8wYvAiB01zuzfeYY_longit_T *obj, const real_T x[12321], real_T *v,
  s1rlrl8wYvAiB01zuzfeYY_longit_T *b_obj)
{
  real_T b_obj_0;
  int32_T b_k;
  int32_T k;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (obj->probType == 2) {
    *b_obj = *obj;
    *v = 0.0;
    std::memcpy(&b_obj->maxConstrWorkspace[0], &b_obj->bineq[0], 332U * sizeof
                (real_T));
    longitudinal_mpc_xgemv_b(36, b_obj->Aineq, x, b_obj->maxConstrWorkspace);
    for (b_k = 0; b_k < 332; b_k++) {
      b_obj_0 = b_obj->maxConstrWorkspace[b_k] - x[b_k + 369];
      b_obj->maxConstrWorkspace[b_k] = b_obj_0;
      *v = std::fmax(*v, b_obj_0);
    }
  } else {
    *b_obj = *obj;
    *v = 0.0;
    std::memcpy(&b_obj->maxConstrWorkspace[0], &b_obj->bineq[0], 332U * sizeof
                (real_T));
    longitudinal_mpc_xgemv_b(b_obj->nVar, b_obj->Aineq, x,
      b_obj->maxConstrWorkspace);
    for (b_k = 0; b_k < 332; b_k++) {
      *v = std::fmax(*v, b_obj->maxConstrWorkspace[b_k]);
    }
  }

  if (obj->sizes[3] > 0) {
    k = static_cast<uint16_T>(obj->sizes[3]) - 1;
    for (b_k = 0; b_k <= k; b_k++) {
      *v = std::fmax(*v, -x[b_obj->indexLB[b_k] + 332]);
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

boolean_T longitudinal_mpc::longitu_feasibleX0ForWorkingSet(real_T workspace
  [12321], real_T xCurrent[37], s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset,
  sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T c;
  real_T constrViolation_basicX;
  int32_T b_idx;
  int32_T br;
  int32_T d;
  int32_T i;
  int32_T iAcol;
  int32_T ix;
  int32_T iy;
  int32_T mWConstr;
  int32_T vectorUB;
  boolean_T nonDegenerateWset;
  static const int32_T offsets[4]{ 0, 1, 2, 3 };

  int32_T exitg1;
  int32_T mWConstr_tmp_tmp;
  int32_T nVar_tmp_tmp;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  mWConstr_tmp_tmp = workingset->nActiveConstr - 1;
  nVar_tmp_tmp = workingset->nVar;
  nonDegenerateWset = true;
  if (workingset->nActiveConstr != 0) {
    for (b_idx = 0; b_idx <= mWConstr_tmp_tmp; b_idx++) {
      workspace[b_idx] = workingset->bwset[b_idx];
      workspace[b_idx + 333] = workingset->bwset[b_idx];
    }

    if (workingset->nActiveConstr != 0) {
      i = (workingset->nActiveConstr - 1) * 37 + 1;
      for (mWConstr = 1; mWConstr <= i; mWConstr += 37) {
        c = 0.0;
        d = mWConstr + nVar_tmp_tmp;
        for (b_idx = mWConstr; b_idx < d; b_idx++) {
          c += workingset->ATwset[b_idx - 1] * xCurrent[b_idx - mWConstr];
        }

        iAcol = div_nde_s32_floor(mWConstr - 1, 37);
        workspace[iAcol] -= c;
      }
    }

    if (workingset->nActiveConstr >= workingset->nVar) {
      i = static_cast<uint8_T>(workingset->nVar) - 1;
      for (b_idx = 0; b_idx <= i; b_idx++) {
        iy = 37 * b_idx - 1;
        for (d = 0; d <= mWConstr_tmp_tmp; d++) {
          qrmanager->QR[(d + iy) + 1] = workingset->ATwset[37 * d + b_idx];
        }
      }

      qrmanager->usedPivoting = false;
      qrmanager->mrows = workingset->nActiveConstr;
      qrmanager->ncols = workingset->nVar;
      d = (static_cast<uint8_T>(workingset->nVar) / 4) << 2;
      vectorUB = d - 4;
      for (b_idx = 0; b_idx <= vectorUB; b_idx += 4) {
        _mm_storeu_si128((__m128i *)&qrmanager->jpvt[b_idx], _mm_add_epi32
                         (_mm_add_epi32(_mm_set1_epi32(b_idx), _mm_loadu_si128((
          const __m128i *)&offsets[0])), _mm_set1_epi32(1)));
      }

      for (b_idx = d; b_idx <= i; b_idx++) {
        qrmanager->jpvt[b_idx] = b_idx + 1;
      }

      if (workingset->nActiveConstr <= workingset->nVar) {
        b_idx = workingset->nActiveConstr;
      } else {
        b_idx = workingset->nVar;
      }

      qrmanager->minRowCol = b_idx;
      std::memcpy(&longitudinal_mpc_B.B[0], &qrmanager->QR[0], 12321U * sizeof
                  (real_T));
      std::memset(&qrmanager->tau[0], 0, 37U * sizeof(real_T));
      if (b_idx >= 1) {
        std::memcpy(&longitudinal_mpc_B.B[0], &qrmanager->QR[0], 12321U * sizeof
                    (real_T));
        std::memset(&qrmanager->tau[0], 0, 37U * sizeof(real_T));
        longitudinal_mpc_qrf(longitudinal_mpc_B.B, workingset->nActiveConstr,
                             workingset->nVar, b_idx, qrmanager->tau);
      }

      std::memcpy(&qrmanager->QR[0], &longitudinal_mpc_B.B[0], 12321U * sizeof
                  (real_T));
      longitudinal_mpc_computeQ_(qrmanager, qrmanager->mrows);
      std::memcpy(&longitudinal_mpc_B.B[0], &workspace[0], 12321U * sizeof
                  (real_T));
      for (b_idx = 0; b_idx <= 333; b_idx += 333) {
        iy = b_idx + nVar_tmp_tmp;
        for (d = b_idx + 1; d <= iy; d++) {
          workspace[d - 1] = 0.0;
        }
      }

      br = -1;
      for (i = 0; i <= 333; i += 333) {
        iAcol = -1;
        iy = i + nVar_tmp_tmp;
        for (mWConstr = i + 1; mWConstr <= iy; mWConstr++) {
          c = 0.0;
          for (b_idx = 0; b_idx <= mWConstr_tmp_tmp; b_idx++) {
            c += qrmanager->Q[(b_idx + iAcol) + 1] * longitudinal_mpc_B.B[(b_idx
              + br) + 1];
          }

          workspace[mWConstr - 1] += c;
          iAcol += 37;
        }

        br += 333;
      }

      for (i = 0; i < 2; i++) {
        ix = 333 * i - 1;
        for (b_idx = nVar_tmp_tmp; b_idx >= 1; b_idx--) {
          mWConstr = (b_idx - 1) * 37 - 1;
          iAcol = b_idx + ix;
          c = workspace[iAcol];
          if (c != 0.0) {
            workspace[iAcol] = c / qrmanager->QR[b_idx + mWConstr];
            iy = static_cast<uint8_T>(b_idx - 1) - 1;
            for (d = 0; d <= iy; d++) {
              mWConstr_tmp_tmp = (d + ix) + 1;
              workspace[mWConstr_tmp_tmp] -= qrmanager->QR[(d + mWConstr) + 1] *
                workspace[iAcol];
            }
          }
        }
      }
    } else {
      longitudinal_mpc_factorQR(qrmanager, workingset->ATwset, workingset->nVar,
        workingset->nActiveConstr);
      longitudinal_mpc_computeQ_(qrmanager, qrmanager->minRowCol);
      for (i = 0; i < 2; i++) {
        ix = 333 * i - 1;
        for (d = 0; d <= mWConstr_tmp_tmp; d++) {
          iAcol = 37 * d;
          b_idx = (d + ix) + 1;
          c = workspace[b_idx];
          iy = static_cast<uint8_T>(d) - 1;
          for (mWConstr = 0; mWConstr <= iy; mWConstr++) {
            c -= workspace[(mWConstr + ix) + 1] * qrmanager->QR[mWConstr + iAcol];
          }

          workspace[b_idx] = c / qrmanager->QR[d + iAcol];
        }
      }

      std::memcpy(&longitudinal_mpc_B.B[0], &workspace[0], 12321U * sizeof
                  (real_T));
      for (b_idx = 0; b_idx <= 333; b_idx += 333) {
        iy = b_idx + nVar_tmp_tmp;
        for (d = b_idx + 1; d <= iy; d++) {
          workspace[d - 1] = 0.0;
        }
      }

      br = 1;
      for (i = 0; i <= 333; i += 333) {
        iAcol = 0;
        iy = br + mWConstr_tmp_tmp;
        for (ix = br; ix <= iy; ix++) {
          b_idx = i + nVar_tmp_tmp;
          d = ((((b_idx - i) / 2) << 1) + i) + 1;
          vectorUB = d - 2;
          for (mWConstr = i + 1; mWConstr <= vectorUB; mWConstr += 2) {
            tmp = _mm_loadu_pd(&qrmanager->Q[((iAcol + mWConstr) - i) - 1]);
            tmp_0 = _mm_loadu_pd(&workspace[mWConstr - 1]);
            _mm_storeu_pd(&workspace[mWConstr - 1], _mm_add_pd(_mm_mul_pd(tmp,
              _mm_set1_pd(longitudinal_mpc_B.B[ix - 1])), tmp_0));
          }

          for (mWConstr = d; mWConstr <= b_idx; mWConstr++) {
            workspace[mWConstr - 1] += qrmanager->Q[((iAcol + mWConstr) - i) - 1]
              * longitudinal_mpc_B.B[ix - 1];
          }

          iAcol += 37;
        }

        br += 333;
      }
    }

    mWConstr = 1;
    do {
      exitg1 = 0;
      if (mWConstr - 1 <= static_cast<uint8_T>(nVar_tmp_tmp) - 1) {
        c = workspace[mWConstr - 1];
        if (std::isinf(c) || std::isnan(c)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[mWConstr + 332];
          if (std::isinf(c) || std::isnan(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            mWConstr++;
          }
        }
      } else {
        d = (nVar_tmp_tmp / 2) << 1;
        vectorUB = d - 2;
        for (b_idx = 0; b_idx <= vectorUB; b_idx += 2) {
          tmp = _mm_loadu_pd(&workspace[b_idx]);
          tmp_0 = _mm_loadu_pd(&xCurrent[b_idx]);
          _mm_storeu_pd(&workspace[b_idx], _mm_add_pd(tmp, tmp_0));
        }

        for (b_idx = d; b_idx < nVar_tmp_tmp; b_idx++) {
          workspace[b_idx] += xCurrent[b_idx];
        }

        longitud_maxConstraintViolation(workingset, workspace, &c,
          &longitudinal_mpc_B.c_workingset);
        longit_maxConstraintViolation_b(&longitudinal_mpc_B.c_workingset,
          workspace, &constrViolation_basicX, workingset);
        if ((c <= 2.2204460492503131E-16) || (c < constrViolation_basicX)) {
          std::memcpy(&xCurrent[0], &workspace[0], static_cast<uint32_T>((
            static_cast<uint8_T>(nVar_tmp_tmp) - 1) + 1) * sizeof(real_T));
        } else {
          std::memcpy(&xCurrent[0], &workspace[333], static_cast<uint32_T>((
            static_cast<uint8_T>(nVar_tmp_tmp) - 1) + 1) * sizeof(real_T));
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
  return nonDegenerateWset;
}

void longitudinal_mpc::longitudi_PresolveWorkingSet_b4(const
  sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution, const real_T
  memspace_workspace_float[12321], const int32_T memspace_workspace_int[333],
  const int32_T memspace_workspace_sort[333], s1rlrl8wYvAiB01zuzfeYY_longit_T
  *workingset, sJvHSAPlL1SbbU0gnSE72ZG_longi_T *b_solution,
  sK6ng1KsrjtpGD3SgQmbb8_longit_T *b_memspace, sCCE9T0P8IQkk6PxUdCnbBE_longi_T
  *qrmanager)
{
  boolean_T okWorkingSet;
  *b_solution = *solution;
  b_solution->state = 82;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  qrmanager->ldq = 37;
  std::memset(&qrmanager->QR[0], 0, 12321U * sizeof(real_T));
  std::memset(&qrmanager->Q[0], 0, 1369U * sizeof(real_T));
  std::memset(&qrmanager->jpvt[0], 0, 333U * sizeof(int32_T));
  qrmanager->mrows = 0;
  qrmanager->ncols = 0;
  std::memset(&qrmanager->tau[0], 0, 37U * sizeof(real_T));
  qrmanager->minRowCol = 0;
  qrmanager->usedPivoting = false;
  std::memcpy(&b_memspace->workspace_float[0], &memspace_workspace_float[0],
              12321U * sizeof(real_T));
  std::memcpy(&b_memspace->workspace_int[0], &memspace_workspace_int[0], 333U *
              sizeof(int32_T));
  std::memcpy(&b_memspace->workspace_sort[0], &memspace_workspace_sort[0], 333U *
              sizeof(int32_T));
  longitudin_RemoveDependentIneq_(workingset, qrmanager, b_memspace, 100.0);
  std::memcpy(&b_solution->xstar[0], &solution->xstar[0], 37U * sizeof(real_T));
  okWorkingSet = longitu_feasibleX0ForWorkingSet(b_memspace->workspace_float,
    b_solution->xstar, workingset, qrmanager);
  if (!okWorkingSet) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    longitudin_RemoveDependentIneq_(workingset, qrmanager, b_memspace, 1000.0);
    okWorkingSet = longitu_feasibleX0ForWorkingSet(b_memspace->workspace_float,
      b_solution->xstar, workingset, qrmanager);
    if (!okWorkingSet) {
      b_solution->state = -7;
    }
  }
}

void longitudinal_mpc::longitudinal_mpc_xgemv_b4(int32_T m, const real_T A[12284],
  const real_T x[37], real_T y[333])
{
  /* Start for MATLABSystem: '<S1>/MPC System' */
  for (int32_T b_iy{0}; b_iy <= 330; b_iy += 2) {
    __m128d tmp;
    tmp = _mm_loadu_pd(&y[b_iy]);
    _mm_storeu_pd(&y[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
  }

  for (int32_T b_iy{0}; b_iy <= 12247; b_iy += 37) {
    real_T c;
    int32_T b;
    int32_T ia;
    c = 0.0;
    b = b_iy + m;
    for (ia = b_iy + 1; ia <= b; ia++) {
      c += x[(ia - b_iy) - 1] * A[ia - 1];
    }

    ia = div_nde_s32_floor(b_iy, 37);
    y[ia] += c;
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longi_maxConstraintViolation_b4(const
  s1rlrl8wYvAiB01zuzfeYY_longit_T *obj, const real_T x[37], real_T *v,
  s1rlrl8wYvAiB01zuzfeYY_longit_T *b_obj)
{
  real_T b_obj_0;
  int32_T b_k;
  int32_T k;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (obj->probType == 2) {
    *b_obj = *obj;
    *v = 0.0;
    std::memcpy(&b_obj->maxConstrWorkspace[0], &b_obj->bineq[0], 332U * sizeof
                (real_T));
    longitudinal_mpc_xgemv_b4(36, b_obj->Aineq, x, b_obj->maxConstrWorkspace);
    for (b_k = 0; b_k < 332; b_k++) {
      b_obj_0 = b_obj->maxConstrWorkspace[b_k] - x[36];
      b_obj->maxConstrWorkspace[b_k] = b_obj_0;
      *v = std::fmax(*v, b_obj_0);
    }
  } else {
    *b_obj = *obj;
    *v = 0.0;
    std::memcpy(&b_obj->maxConstrWorkspace[0], &b_obj->bineq[0], 332U * sizeof
                (real_T));
    longitudinal_mpc_xgemv_b4(b_obj->nVar, b_obj->Aineq, x,
      b_obj->maxConstrWorkspace);
    for (b_k = 0; b_k < 332; b_k++) {
      *v = std::fmax(*v, b_obj->maxConstrWorkspace[b_k]);
    }
  }

  if (obj->sizes[3] > 0) {
    k = static_cast<uint16_T>(obj->sizes[3]) - 1;
    for (b_k = 0; b_k <= k; b_k++) {
      *v = std::fmax(*v, -x[b_obj->indexLB[b_k] - 1]);
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitu_modifyOverheadPhaseOne_(const
  s1rlrl8wYvAiB01zuzfeYY_longit_T *obj, s1rlrl8wYvAiB01zuzfeYY_longit_T *b_obj)
{
  int32_T b;
  int32_T idxStartIneq;
  *b_obj = *obj;
  for (idxStartIneq = 0; idxStartIneq < 332; idxStartIneq++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_obj->Aineq[37 * idxStartIneq + 36] = -1.0;
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  b_obj->indexLB[obj->sizes[3] - 1] = 37;
  b_obj->lb[36] = 0.0;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  idxStartIneq = obj->isActiveIdx[2];
  b = obj->nActiveConstr;
  for (int32_T c_idx{idxStartIneq}; c_idx <= b; c_idx++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    b_obj->ATwset[37 * (c_idx - 1) + 36] = -1.0;
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (obj->nWConstr[4] <= 0) {
    b_obj->isActiveConstr[obj->isActiveIdx[4] - 1] = false;
  }

  b_obj->isActiveConstr[obj->isActiveIdx[4] - 2] = false;
}

void longitudinal_mpc::longitudinal_mpc_setProblemType
  (s1rlrl8wYvAiB01zuzfeYY_longit_T *obj, int32_T PROBLEM_TYPE)
{
  int32_T b_idx_col;
  int32_T colOffsetAineq;
  int32_T f_tmp;
  int32_T idx_col;
  int32_T idx_lb;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  switch (PROBLEM_TYPE) {
   case 3:
    obj->nVar = 36;
    obj->mConstr = 332;
    for (b_idx_col = 0; b_idx_col < 5; b_idx_col++) {
      obj->sizes[b_idx_col] = obj->sizesNormal[b_idx_col];
    }

    for (b_idx_col = 0; b_idx_col < 6; b_idx_col++) {
      obj->isActiveIdx[b_idx_col] = obj->isActiveIdxNormal[b_idx_col];
    }
    break;

   case 1:
    obj->nVar = 37;
    obj->mConstr = 333;
    for (b_idx_col = 0; b_idx_col < 5; b_idx_col++) {
      obj->sizes[b_idx_col] = obj->sizesPhaseOne[b_idx_col];
    }

    longitudinal_mpc_B.obj = *obj;
    longitu_modifyOverheadPhaseOne_(&longitudinal_mpc_B.obj, obj);
    for (b_idx_col = 0; b_idx_col < 6; b_idx_col++) {
      obj->isActiveIdx[b_idx_col] = obj->isActiveIdxPhaseOne[b_idx_col];
    }
    break;

   case 2:
    obj->nVar = 36;
    obj->mConstr = 332;
    for (b_idx_col = 0; b_idx_col < 5; b_idx_col++) {
      obj->sizes[b_idx_col] = obj->sizesRegularized[b_idx_col];
    }

    if (obj->probType != 4) {
      for (b_idx_col = 0; b_idx_col < 332; b_idx_col++) {
        colOffsetAineq = 37 * b_idx_col - 1;
        for (idx_lb = 37; idx_lb <= b_idx_col + 36; idx_lb++) {
          obj->Aineq[idx_lb + colOffsetAineq] = 0.0;
        }

        obj->Aineq[(b_idx_col + colOffsetAineq) + 37] = -1.0;
      }

      idx_lb = 36;
      for (b_idx_col = 0; b_idx_col < 332; b_idx_col++) {
        idx_lb++;
        obj->indexLB[b_idx_col] = idx_lb;
      }

      idx_lb = obj->isActiveIdx[4];
      idx_col = obj->isActiveIdxRegularized[4];
      if (idx_lb <= idx_col - 1) {
        std::memset(&obj->isActiveConstr[idx_lb + -1], 0, static_cast<uint32_T>
                    (idx_col - idx_lb) * sizeof(boolean_T));
      }

      obj->lb[36] = 0.0;
      idx_lb = obj->isActiveIdx[2];
      idx_col = obj->nActiveConstr;
      for (b_idx_col = idx_lb; b_idx_col <= idx_col; b_idx_col++) {
        colOffsetAineq = (b_idx_col - 1) * 37 - 1;
        if (obj->Wid[b_idx_col - 1] == 3) {
          f_tmp = obj->Wlocalidx[b_idx_col - 1];
          if (f_tmp + 35 >= 37) {
            std::memset(&obj->ATwset[colOffsetAineq + 37], 0,
                        static_cast<uint32_T>(((f_tmp + colOffsetAineq) -
              colOffsetAineq) - 1) * sizeof(real_T));
          }

          obj->ATwset[(f_tmp + colOffsetAineq) + 36] = -1.0;
          if (f_tmp + 37 <= 36) {
            std::memset(&obj->ATwset[(f_tmp + colOffsetAineq) + 37], 0,
                        static_cast<uint32_T>((colOffsetAineq - f_tmp) -
              colOffsetAineq) * sizeof(real_T));
          }
        }
      }
    }

    for (b_idx_col = 0; b_idx_col < 6; b_idx_col++) {
      obj->isActiveIdx[b_idx_col] = obj->isActiveIdxRegularized[b_idx_col];
    }
    break;

   default:
    obj->nVar = 37;
    obj->mConstr = 333;
    for (b_idx_col = 0; b_idx_col < 5; b_idx_col++) {
      obj->sizes[b_idx_col] = obj->sizesRegPhaseOne[b_idx_col];
    }

    longitudinal_mpc_B.obj = *obj;
    longitu_modifyOverheadPhaseOne_(&longitudinal_mpc_B.obj, obj);
    for (b_idx_col = 0; b_idx_col < 6; b_idx_col++) {
      obj->isActiveIdx[b_idx_col] = obj->isActiveIdxRegPhaseOne[b_idx_col];
    }
    break;
  }

  obj->probType = PROBLEM_TYPE;

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_linearForm_(boolean_T obj_hasLinear,
  int32_T obj_nvar, real_T workspace[12321], const real_T H[1296], const real_T
  f[36], const real_T x[37])
{
  real_T fMultiplier;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  fMultiplier = 0.0;
  if (obj_hasLinear) {
    if (static_cast<uint8_T>(obj_nvar) - 1 >= 0) {
      std::memcpy(&workspace[0], &f[0], static_cast<uint32_T>
                  ((static_cast<uint8_T>(obj_nvar) - 1) + 1) * sizeof(real_T));
    }

    fMultiplier = 1.0;
  }

  if (obj_nvar != 0) {
    int32_T d;
    int32_T ix;
    if (fMultiplier != 1.0) {
      std::memset(&workspace[0], 0, static_cast<uint32_T>((static_cast<uint8_T>
        (obj_nvar) - 1) + 1) * sizeof(real_T));
    }

    ix = 0;
    d = (obj_nvar - 1) * obj_nvar;
    for (int32_T b_i{1}; obj_nvar < 0 ? b_i >= d + 1 : b_i <= d + 1; b_i +=
         obj_nvar) {
      int32_T e;
      fMultiplier = 0.5 * x[ix];
      e = b_i + obj_nvar;
      for (int32_T c{b_i}; c < e; c++) {
        int32_T tmp;
        tmp = c - b_i;
        workspace[tmp] += H[c - 1] * fMultiplier;
      }

      ix++;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

real_T longitudinal_mpc::longitudinal_mpc_computeFval(const
  scJprC0tZnwNUG3KoXsnHVD_longi_T *obj, real_T workspace[12321], const real_T H
  [1296], const real_T f[36], const real_T x[37])
{
  real_T val;
  int32_T b;
  int32_T idx;
  int32_T scalarLB;
  int32_T vectorUB;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  switch (obj->objtype) {
   case 5:
    val = x[obj->nvar - 1] * obj->gammaScalar;
    break;

   case 3:
    longitudinal_mpc_linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
    val = 0.0;
    if (obj->nvar >= 1) {
      b = static_cast<uint8_T>(obj->nvar) - 1;
      for (idx = 0; idx <= b; idx++) {
        val += x[idx] * workspace[idx];
      }
    }
    break;

   default:
    longitudinal_mpc_linearForm_(obj->hasLinear, obj->nvar, workspace, H, f, x);
    b = obj->nvar;
    scalarLB = ((((36 - obj->nvar) / 2) << 1) + obj->nvar) + 1;
    vectorUB = scalarLB - 2;
    for (idx = b + 1; idx <= vectorUB; idx += 2) {
      _mm_storeu_pd(&workspace[idx - 1], _mm_mul_pd(_mm_loadu_pd(&x[idx - 1]),
        _mm_set1_pd(0.0)));
    }

    for (idx = scalarLB; idx < 37; idx++) {
      workspace[idx - 1] = x[idx - 1] * 0.0;
    }

    val = 0.0;
    for (idx = 0; idx < 36; idx++) {
      val += x[idx] * workspace[idx];
    }
    break;
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
  return val;
}

void longitudinal_mpc::longitudinal_mpc_xgemv_b4t(int32_T m, int32_T n, const
  real_T A[1296], int32_T lda, const real_T x[37], real_T y[36])
{
  /* Start for MATLABSystem: '<S1>/MPC System' */
  if ((m != 0) && (n != 0)) {
    int32_T d;
    int32_T ix;
    std::memset(&y[0], 0, static_cast<uint32_T>((static_cast<uint8_T>(m) - 1) +
      1) * sizeof(real_T));
    ix = 0;
    d = (n - 1) * lda;
    for (int32_T b_iy{1}; lda < 0 ? b_iy >= d + 1 : b_iy <= d + 1; b_iy += lda)
    {
      int32_T e;
      e = b_iy + m;
      for (int32_T b{b_iy}; b < e; b++) {
        int32_T tmp;
        tmp = b - b_iy;
        y[tmp] += A[b - 1] * x[ix];
      }

      ix++;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudina_computeGrad_StoreHx
  (scJprC0tZnwNUG3KoXsnHVD_longi_T *obj, const real_T H[1296], const real_T f[36],
   const real_T x[37])
{
  __m128d tmp;
  int32_T idx;
  int32_T ixlast;
  int32_T scalarLB;
  int32_T vectorUB;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  switch (obj->objtype) {
   case 5:
    if (obj->nvar - 2 >= 0) {
      std::memset(&obj->grad[0], 0, static_cast<uint32_T>((obj->nvar - 2) + 1) *
                  sizeof(real_T));
    }

    obj->grad[obj->nvar - 1] = obj->gammaScalar;
    break;

   case 3:
    longitudinal_mpc_xgemv_b4t(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
    if (static_cast<uint8_T>(obj->nvar) - 1 >= 0) {
      std::memcpy(&obj->grad[0], &obj->Hx[0], static_cast<uint32_T>((
        static_cast<uint8_T>(obj->nvar) - 1) + 1) * sizeof(real_T));
    }

    if (obj->hasLinear && (obj->nvar >= 1)) {
      ixlast = obj->nvar;
      scalarLB = (obj->nvar / 2) << 1;
      vectorUB = scalarLB - 2;
      for (idx = 0; idx <= vectorUB; idx += 2) {
        tmp = _mm_loadu_pd(&obj->grad[idx]);
        _mm_storeu_pd(&obj->grad[idx], _mm_add_pd(tmp, _mm_loadu_pd(&f[idx])));
      }

      for (idx = scalarLB; idx < ixlast; idx++) {
        obj->grad[idx] += f[idx];
      }
    }
    break;

   default:
    longitudinal_mpc_xgemv_b4t(obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
    ixlast = obj->nvar;
    scalarLB = ((((36 - obj->nvar) / 2) << 1) + obj->nvar) + 1;
    vectorUB = scalarLB - 2;
    for (idx = ixlast + 1; idx <= vectorUB; idx += 2) {
      _mm_storeu_pd(&obj->Hx[idx - 1], _mm_mul_pd(_mm_loadu_pd(&x[idx - 1]),
        _mm_set1_pd(0.0)));
    }

    for (idx = scalarLB; idx < 37; idx++) {
      obj->Hx[idx - 1] = x[idx - 1] * 0.0;
    }

    std::memcpy(&obj->grad[0], &obj->Hx[0], 36U * sizeof(real_T));
    if (obj->hasLinear && (obj->nvar >= 1)) {
      ixlast = obj->nvar;
      scalarLB = (obj->nvar / 2) << 1;
      vectorUB = scalarLB - 2;
      for (idx = 0; idx <= vectorUB; idx += 2) {
        tmp = _mm_loadu_pd(&obj->grad[idx]);
        _mm_storeu_pd(&obj->grad[idx], _mm_add_pd(tmp, _mm_loadu_pd(&f[idx])));
      }

      for (idx = scalarLB; idx < ixlast; idx++) {
        obj->grad[idx] += f[idx];
      }
    }
    break;
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

real_T longitudinal_mpc::longitudina_computeFval_ReuseHx(const
  scJprC0tZnwNUG3KoXsnHVD_longi_T *obj, real_T workspace[12321], const real_T f
  [36], const real_T x[37])
{
  real_T val;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  switch (obj->objtype) {
   case 5:
    val = x[obj->nvar - 1] * obj->gammaScalar;
    break;

   case 3:
    {
      if (obj->hasLinear) {
        int32_T b_k;
        int32_T e;
        int32_T vectorUB;
        e = static_cast<uint8_T>(obj->nvar) - 1;
        b_k = (static_cast<uint8_T>(obj->nvar) / 2) << 1;
        vectorUB = b_k - 2;
        for (int32_T k{0}; k <= vectorUB; k += 2) {
          __m128d tmp;
          tmp = _mm_loadu_pd(&obj->Hx[k]);
          _mm_storeu_pd(&workspace[k], _mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.5),
            tmp), _mm_loadu_pd(&f[k])));
        }

        for (int32_T k{b_k}; k <= e; k++) {
          workspace[k] = 0.5 * obj->Hx[k] + f[k];
        }

        val = 0.0;
        if (obj->nvar >= 1) {
          for (b_k = 0; b_k <= e; b_k++) {
            val += x[b_k] * workspace[b_k];
          }
        }
      } else {
        val = 0.0;
        if (obj->nvar >= 1) {
          int32_T e;
          e = static_cast<uint8_T>(obj->nvar) - 1;
          for (int32_T b_k{0}; b_k <= e; b_k++) {
            val += x[b_k] * obj->Hx[b_k];
          }
        }

        val *= 0.5;
      }
    }
    break;

   default:
    {
      if (obj->hasLinear) {
        int32_T k;
        if (static_cast<uint8_T>(obj->nvar) - 1 >= 0) {
          std::memcpy(&workspace[0], &f[0], static_cast<uint32_T>((static_cast<
            uint8_T>(obj->nvar) - 1) + 1) * sizeof(real_T));
        }

        k = 35 - obj->nvar;
        for (int32_T b_k{0}; b_k <= k; b_k++) {
          workspace[obj->nvar + b_k] = 0.0;
        }

        val = 0.0;
        for (k = 0; k < 36; k++) {
          real_T workspace_0;
          workspace_0 = 0.5 * obj->Hx[k] + workspace[k];
          workspace[k] = workspace_0;
          val += x[k] * workspace_0;
        }
      } else {
        int32_T k;
        val = 0.0;
        for (int32_T b_k{0}; b_k < 36; b_k++) {
          val += x[b_k] * obj->Hx[b_k];
        }

        val *= 0.5;
        k = obj->nvar;
        for (int32_T b_k{k + 1}; b_k < 37; b_k++) {
          val += x[b_k - 1] * 0.0;
        }
      }
    }
    break;
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
  return val;
}

void longitudinal_mpc::longitudinal_mpc_xrotg(real_T a, real_T b, real_T *b_a,
  real_T *b_b, real_T *c, real_T *s)
{
  real_T absa;
  real_T absb;
  real_T roe;
  real_T scale;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  roe = b;
  absa = std::abs(a);
  absb = std::abs(b);
  if (absa > absb) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    roe = a;
  }

  scale = absa + absb;
  if (scale == 0.0) {
    *s = 0.0;
    *c = 1.0;
    *b_a = 0.0;
    *b_b = 0.0;
  } else {
    real_T ads;
    real_T bds;
    ads = absa / scale;
    bds = absb / scale;

    /* Start for MATLABSystem: '<S1>/MPC System' */
    *b_a = std::sqrt(ads * ads + bds * bds) * scale;
    if (roe < 0.0) {
      *b_a = -*b_a;
    }

    /* Start for MATLABSystem: '<S1>/MPC System' */
    *c = a / *b_a;
    *s = b / *b_a;
    if (absa > absb) {
      *b_b = *s;
    } else if (*c != 0.0) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      *b_b = 1.0 / *c;
    } else {
      *b_b = 1.0;
    }
  }
}

void longitudinal_mpc::longitudinal_m_deleteColMoveEnd
  (sCCE9T0P8IQkk6PxUdCnbBE_longi_T *obj, int32_T idx)
{
  real_T b_s;
  real_T c_c;
  real_T temp;
  real_T temp_tmp_0;
  int32_T QRk0;
  int32_T c_k;
  int32_T endIdx;
  int32_T i;
  int32_T idxRotGCol;
  int32_T n;
  int32_T temp_tmp;
  int32_T tmp;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt[i - 1] != idx)) {
      i++;
    }

    idx = i;
  }

  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    obj->jpvt[idx - 1] = obj->jpvt[obj->ncols - 1];
    idxRotGCol = obj->minRowCol - 1;
    for (endIdx = 0; endIdx <= idxRotGCol; endIdx++) {
      obj->QR[endIdx + 37 * (idx - 1)] = obj->QR[(obj->ncols - 1) * 37 + endIdx];
    }

    obj->ncols--;
    if (obj->mrows <= obj->ncols) {
      obj->minRowCol = obj->mrows;
    } else {
      obj->minRowCol = obj->ncols;
    }

    if (idx < obj->mrows) {
      if (obj->mrows - 1 <= obj->ncols) {
        endIdx = obj->mrows - 1;
      } else {
        endIdx = obj->ncols;
      }

      i = endIdx;
      idxRotGCol = (idx - 1) * 37;
      while (i >= idx) {
        tmp = i + idxRotGCol;
        longitudinal_mpc_xrotg(obj->QR[tmp - 1], obj->QR[tmp], &obj->QR[tmp - 1],
          &temp, &c_c, &b_s);
        obj->QR[tmp] = temp;
        tmp = (i - 1) * 37;
        obj->QR[i + tmp] = 0.0;
        QRk0 = (37 * idx + i) - 1;
        n = obj->ncols - idx;
        if (n >= 1) {
          for (c_k = 0; c_k < n; c_k++) {
            temp_tmp = c_k * 37 + QRk0;
            temp_tmp_0 = obj->QR[temp_tmp + 1];
            temp = obj->QR[temp_tmp] * c_c + temp_tmp_0 * b_s;
            obj->QR[temp_tmp + 1] = temp_tmp_0 * c_c - obj->QR[temp_tmp] * b_s;
            obj->QR[temp_tmp] = temp;
          }
        }

        n = obj->mrows;
        if (obj->mrows >= 1) {
          for (c_k = 0; c_k < n; c_k++) {
            temp_tmp = tmp + c_k;
            temp_tmp_0 = obj->Q[temp_tmp + 37];
            temp = temp_tmp_0 * b_s + obj->Q[temp_tmp] * c_c;
            obj->Q[temp_tmp + 37] = temp_tmp_0 * c_c - obj->Q[temp_tmp] * b_s;
            obj->Q[temp_tmp] = temp;
          }
        }

        i--;
      }

      for (c_k = idx + 1; c_k <= endIdx; c_k++) {
        idxRotGCol = (c_k - 1) * 37;
        tmp = c_k + idxRotGCol;
        longitudinal_mpc_xrotg(obj->QR[tmp - 1], obj->QR[tmp], &obj->QR[tmp - 1],
          &temp, &c_c, &b_s);
        obj->QR[tmp] = temp;
        QRk0 = c_k * 38 - 1;
        n = obj->ncols - c_k;
        if (n >= 1) {
          for (i = 0; i < n; i++) {
            temp_tmp = i * 37 + QRk0;
            temp = obj->QR[temp_tmp + 1] * b_s + obj->QR[temp_tmp] * c_c;
            obj->QR[temp_tmp + 1] = obj->QR[temp_tmp + 1] * c_c - obj->
              QR[temp_tmp] * b_s;
            obj->QR[temp_tmp] = temp;
          }
        }

        n = obj->mrows;
        if (obj->mrows >= 1) {
          for (i = 0; i < n; i++) {
            temp_tmp = idxRotGCol + i;
            temp = obj->Q[temp_tmp + 37] * b_s + obj->Q[temp_tmp] * c_c;
            obj->Q[temp_tmp + 37] = obj->Q[temp_tmp + 37] * c_c - obj->
              Q[temp_tmp] * b_s;
            obj->Q[temp_tmp] = temp;
          }
        }
      }
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_fullColLDL2_
  (skczSSN0IseekIuhVW4znFG_longi_T *obj, int32_T NColsRemain, real_T REG_PRIMAL)
{
  int32_T lastDiag;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  for (lastDiag = 0; lastDiag < NColsRemain; lastDiag++) {
    real_T neg_D;
    real_T temp;
    int32_T LD_diagOffset;
    int32_T b_k;
    int32_T ijA;
    int32_T subMatrixDim;
    LD_diagOffset = 38 * lastDiag;
    temp = obj->FMat[LD_diagOffset];
    if (std::abs(temp) <= obj->regTol_) {
      temp += REG_PRIMAL;
      obj->FMat[LD_diagOffset] = temp;
    }

    neg_D = -1.0 / temp;
    subMatrixDim = (NColsRemain - lastDiag) - 1;
    for (b_k = 0; b_k < subMatrixDim; b_k++) {
      obj->workspace_[b_k] = obj->FMat[(LD_diagOffset + b_k) + 1];
    }

    if (!(neg_D == 0.0)) {
      int32_T jA;
      jA = LD_diagOffset;
      for (int32_T k{0}; k < subMatrixDim; k++) {
        temp = obj->workspace_[k];
        if (temp != 0.0) {
          int32_T c;
          temp *= neg_D;
          b_k = jA + 39;
          c = (subMatrixDim + jA) + 38;
          for (ijA = b_k; ijA <= c; ijA++) {
            obj->FMat[ijA - 1] += obj->workspace_[(ijA - jA) - 39] * temp;
          }
        }

        jA += 37;
      }
    }

    neg_D = 1.0 / obj->FMat[LD_diagOffset];
    b_k = (LD_diagOffset + subMatrixDim) + 1;
    ijA = (((((b_k - LD_diagOffset) - 1) / 2) << 1) + LD_diagOffset) + 2;
    subMatrixDim = ijA - 2;
    for (int32_T k{LD_diagOffset + 2}; k <= subMatrixDim; k += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&obj->FMat[k - 1]);
      _mm_storeu_pd(&obj->FMat[k - 1], _mm_mul_pd(tmp, _mm_set1_pd(neg_D)));
    }

    for (int32_T k{ijA}; k <= b_k; k++) {
      obj->FMat[k - 1] *= neg_D;
    }
  }

  lastDiag = (NColsRemain - 1) * 38;
  if (std::abs(obj->FMat[lastDiag]) <= obj->regTol_) {
    obj->FMat[lastDiag] += REG_PRIMAL;
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_xgemv_b4tv(int32_T m, int32_T n, const
  real_T A[1369], int32_T ia0, const real_T x[12321], real_T y[37])
{
  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (m != 0) {
    int32_T b;
    int32_T ix;
    if (m - 1 >= 0) {
      std::memset(&y[0], 0, static_cast<uint32_T>(m) * sizeof(real_T));
    }

    ix = 0;
    b = (n - 1) * 37 + ia0;
    for (int32_T b_iy{ia0}; b_iy <= b; b_iy += 37) {
      int32_T d;
      d = b_iy + m;
      for (int32_T ia{b_iy}; ia < d; ia++) {
        int32_T tmp;
        tmp = ia - b_iy;
        y[tmp] += A[ia - 1] * x[ix];
      }

      ix++;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_compute_deltax(const real_T H[1296],
  sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution, sK6ng1KsrjtpGD3SgQmbb8_longit_T
  *memspace, const sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager,
  skczSSN0IseekIuhVW4znFG_longi_T *cholmanager, const
  scJprC0tZnwNUG3KoXsnHVD_longi_T *objective)
{
  __m128d tmp;
  real_T s;
  real_T smax;
  real_T temp;
  int32_T A_maxDiag_idx;
  int32_T ar;
  int32_T br;
  int32_T e;
  int32_T exitg1;
  int32_T g;
  int32_T ia;
  int32_T ix;
  int32_T lastColC;
  int32_T mNull_tmp;
  int32_T nVar;
  int32_T nVars;
  int32_T nullStartIdx;
  int32_T nullStartIdx_tmp;
  int32_T scalarLB;
  int32_T vectorUB;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  nVar = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    if (qrmanager->mrows - 1 >= 0) {
      std::memset(&solution->searchDir[0], 0, static_cast<uint32_T>
                  ((qrmanager->mrows - 1) + 1) * sizeof(real_T));
    }
  } else {
    scalarLB = (qrmanager->mrows / 2) << 1;
    vectorUB = scalarLB - 2;
    for (nullStartIdx = 0; nullStartIdx <= vectorUB; nullStartIdx += 2) {
      tmp = _mm_loadu_pd(&objective->grad[nullStartIdx]);
      _mm_storeu_pd(&solution->searchDir[nullStartIdx], _mm_mul_pd(tmp,
        _mm_set1_pd(-1.0)));
    }

    for (nullStartIdx = scalarLB; nullStartIdx <= nVar; nullStartIdx++) {
      solution->searchDir[nullStartIdx] = -objective->grad[nullStartIdx];
    }

    if (qrmanager->ncols <= 0) {
      if (objective->objtype == 3) {
        temp = 1.4901161193847656E-6 * static_cast<real_T>(qrmanager->mrows);
        cholmanager->ndims = qrmanager->mrows;
        for (ix = 0; ix <= nVar; ix++) {
          nullStartIdx = (nVar + 1) * ix - 1;
          A_maxDiag_idx = 37 * ix - 1;
          for (ia = 0; ia <= nVar; ia++) {
            cholmanager->FMat[(A_maxDiag_idx + ia) + 1] = H[(ia + nullStartIdx)
              + 1];
          }
        }

        if (qrmanager->mrows < 1) {
          A_maxDiag_idx = -1;
        } else {
          A_maxDiag_idx = 0;
          if (qrmanager->mrows > 1) {
            smax = std::abs(cholmanager->FMat[0]);
            for (ia = 2; ia <= nVar + 1; ia++) {
              s = std::abs(cholmanager->FMat[(ia - 1) * 38]);
              if (s > smax) {
                A_maxDiag_idx = ia - 1;
                smax = s;
              }
            }
          }
        }

        cholmanager->regTol_ = std::fmax(std::abs(cholmanager->FMat[37 *
          A_maxDiag_idx + A_maxDiag_idx]) * 2.2204460492503131E-16, std::abs
          (temp));
        longitudinal_mpc_fullColLDL2_(cholmanager, qrmanager->mrows, temp);
        if (cholmanager->ConvexCheck) {
          nullStartIdx = 1;
          do {
            exitg1 = 0;
            if (nullStartIdx - 1 <= nVar) {
              if (cholmanager->FMat[((nullStartIdx - 1) * 37 + nullStartIdx) - 1]
                  <= 0.0) {
                cholmanager->info = -nullStartIdx;
                exitg1 = 1;
              } else {
                nullStartIdx++;
              }
            } else {
              cholmanager->ConvexCheck = false;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          nullStartIdx = cholmanager->ndims;
          if (cholmanager->ndims != 0) {
            for (nVars = 0; nVars < nullStartIdx; nVars++) {
              lastColC = nVars * 37 + nVars;
              A_maxDiag_idx = (nullStartIdx - nVars) - 2;
              for (ia = 0; ia <= A_maxDiag_idx; ia++) {
                ix = (ia + nVars) + 1;
                solution->searchDir[ix] -= cholmanager->FMat[(ia + lastColC) + 1]
                  * solution->searchDir[nVars];
              }
            }
          }

          A_maxDiag_idx = cholmanager->ndims - 1;
          for (ix = 0; ix <= A_maxDiag_idx; ix++) {
            solution->searchDir[ix] /= cholmanager->FMat[37 * ix + ix];
          }

          if (cholmanager->ndims != 0) {
            for (nVar = nullStartIdx; nVar >= 1; nVar--) {
              br = (nVar - 1) * 37 - 1;
              temp = solution->searchDir[nVar - 1];
              for (ia = nullStartIdx; ia >= nVar + 1; ia--) {
                temp -= cholmanager->FMat[br + ia] * solution->searchDir[ia - 1];
              }

              solution->searchDir[nVar - 1] = temp;
            }
          }
        }
      }
    } else {
      nullStartIdx_tmp = 37 * qrmanager->ncols;
      nullStartIdx = nullStartIdx_tmp + 1;
      if (objective->objtype == 5) {
        for (ia = 0; ia < mNull_tmp; ia++) {
          memspace->workspace_float[ia] = -qrmanager->Q[(qrmanager->ncols + ia) *
            37 + nVar];
        }

        longitudinal_mpc_xgemv_b4tv(qrmanager->mrows, mNull_tmp, qrmanager->Q,
          nullStartIdx_tmp + 1, memspace->workspace_float, solution->searchDir);
      } else {
        if (objective->objtype == 3) {
          nVars = qrmanager->mrows;
          if ((qrmanager->mrows != 0) && (mNull_tmp != 0)) {
            lastColC = (mNull_tmp - 1) * 333;
            for (ia = 0; ia <= lastColC; ia += 333) {
              br = ia + nVars;
              for (ix = ia + 1; ix <= br; ix++) {
                memspace->workspace_float[ix - 1] = 0.0;
              }
            }

            br = nullStartIdx_tmp;
            for (A_maxDiag_idx = 0; A_maxDiag_idx <= lastColC; A_maxDiag_idx +=
                 333) {
              ar = 0;
              e = br + nVars;
              for (ia = br + 1; ia <= e; ia++) {
                g = A_maxDiag_idx + nVars;
                scalarLB = ((((g - A_maxDiag_idx) / 2) << 1) + A_maxDiag_idx) +
                  1;
                vectorUB = scalarLB - 2;
                for (ix = A_maxDiag_idx + 1; ix <= vectorUB; ix += 2) {
                  tmp = _mm_loadu_pd(&memspace->workspace_float[ix - 1]);
                  _mm_storeu_pd(&memspace->workspace_float[ix - 1], _mm_add_pd
                                (_mm_mul_pd(_mm_loadu_pd(&H[((ar + ix) -
                    A_maxDiag_idx) - 1]), _mm_set1_pd(qrmanager->Q[ia - 1])),
                                 tmp));
                }

                for (ix = scalarLB; ix <= g; ix++) {
                  memspace->workspace_float[ix - 1] += H[((ar + ix) -
                    A_maxDiag_idx) - 1] * qrmanager->Q[ia - 1];
                }

                ar += nVars;
              }

              br += 37;
            }
          }

          if (mNull_tmp != 0) {
            lastColC = (mNull_tmp - 1) * 37;
            for (ia = 0; ia <= lastColC; ia += 37) {
              br = ia + mNull_tmp;
              for (ix = ia + 1; ix <= br; ix++) {
                cholmanager->FMat[ix - 1] = 0.0;
              }
            }

            br = -1;
            for (A_maxDiag_idx = 0; A_maxDiag_idx <= lastColC; A_maxDiag_idx +=
                 37) {
              ar = nullStartIdx_tmp - 1;
              e = A_maxDiag_idx + mNull_tmp;
              for (ix = A_maxDiag_idx + 1; ix <= e; ix++) {
                temp = 0.0;
                for (ia = 0; ia < nVars; ia++) {
                  temp += qrmanager->Q[(ia + ar) + 1] *
                    memspace->workspace_float[(ia + br) + 1];
                }

                cholmanager->FMat[ix - 1] += temp;
                ar += 37;
              }

              br += 333;
            }
          }
        }

        temp = 1.4901161193847656E-6 * static_cast<real_T>(mNull_tmp);
        cholmanager->ndims = mNull_tmp;
        A_maxDiag_idx = 0;
        if (mNull_tmp > 1) {
          smax = std::abs(cholmanager->FMat[0]);
          for (ia = 2; ia <= mNull_tmp; ia++) {
            s = std::abs(cholmanager->FMat[(ia - 1) * 38]);
            if (s > smax) {
              A_maxDiag_idx = ia - 1;
              smax = s;
            }
          }
        }

        cholmanager->regTol_ = std::fmax(std::abs(cholmanager->FMat[37 *
          A_maxDiag_idx + A_maxDiag_idx]) * 2.2204460492503131E-16, temp);
        longitudinal_mpc_fullColLDL2_(cholmanager, mNull_tmp, temp);
        if (cholmanager->ConvexCheck) {
          ix = 1;
          do {
            exitg1 = 0;
            if (ix - 1 <= mNull_tmp - 1) {
              if (cholmanager->FMat[((ix - 1) * 37 + ix) - 1] <= 0.0) {
                cholmanager->info = -ix;
                exitg1 = 1;
              } else {
                ix++;
              }
            } else {
              cholmanager->ConvexCheck = false;
              exitg1 = 1;
            }
          } while (exitg1 == 0);
        }

        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          if (qrmanager->mrows != 0) {
            std::memset(&memspace->workspace_float[0], 0, static_cast<uint32_T>
                        (mNull_tmp) * sizeof(real_T));
            A_maxDiag_idx = ((mNull_tmp - 1) * 37 + nullStartIdx_tmp) + 1;
            for (nVars = nullStartIdx; nVars <= A_maxDiag_idx; nVars += 37) {
              temp = 0.0;
              br = nVars + nVar;
              for (ia = nVars; ia <= br; ia++) {
                temp += qrmanager->Q[ia - 1] * objective->grad[ia - nVars];
              }

              ix = div_nde_s32_floor((nVars - nullStartIdx_tmp) - 1, 37);
              memspace->workspace_float[ix] -= temp;
            }
          }

          nullStartIdx = cholmanager->ndims;
          if (cholmanager->ndims != 0) {
            for (nVars = 0; nVars < nullStartIdx; nVars++) {
              lastColC = nVars * 37 + nVars;
              A_maxDiag_idx = (nullStartIdx - nVars) - 2;
              for (ia = 0; ia <= A_maxDiag_idx; ia++) {
                ix = (ia + nVars) + 1;
                memspace->workspace_float[ix] -= cholmanager->FMat[(ia +
                  lastColC) + 1] * memspace->workspace_float[nVars];
              }
            }
          }

          A_maxDiag_idx = cholmanager->ndims - 1;
          for (ix = 0; ix <= A_maxDiag_idx; ix++) {
            memspace->workspace_float[ix] /= cholmanager->FMat[37 * ix + ix];
          }

          if (cholmanager->ndims != 0) {
            for (nVar = nullStartIdx; nVar >= 1; nVar--) {
              br = (nVar - 1) * 37 - 1;
              temp = memspace->workspace_float[nVar - 1];
              for (ia = nullStartIdx; ia >= nVar + 1; ia--) {
                temp -= cholmanager->FMat[br + ia] * memspace->
                  workspace_float[ia - 1];
              }

              memspace->workspace_float[nVar - 1] = temp;
            }
          }

          longitudinal_mpc_xgemv_b4tv(qrmanager->mrows, mNull_tmp, qrmanager->Q,
            nullStartIdx_tmp + 1, memspace->workspace_float, solution->searchDir);
        }
      }
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

real_T longitudinal_mpc::longitudinal_mpc_xnrm2_b(int32_T n, const real_T x[37])
{
  real_T y;
  y = 0.0;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[0]);
    } else {
      real_T scale;
      int32_T b;
      scale = 3.3121686421112381E-170;
      b = static_cast<uint8_T>(n) - 1;
      for (int32_T b_k{0}; b_k <= b; b_k++) {
        real_T absxk;
        absxk = std::abs(x[b_k]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
  return y;
}

void longitudinal_mpc::longitudinal_mpc_xgemv_b4tv3(int32_T m, const real_T A
  [12284], const real_T x[37], real_T y[12321])
{
  /* Start for MATLABSystem: '<S1>/MPC System' */
  for (int32_T b_iy{0}; b_iy <= 330; b_iy += 2) {
    __m128d tmp;
    tmp = _mm_loadu_pd(&y[b_iy]);
    _mm_storeu_pd(&y[b_iy], _mm_mul_pd(tmp, _mm_set1_pd(-1.0)));
  }

  for (int32_T b_iy{0}; b_iy <= 12247; b_iy += 37) {
    real_T c;
    int32_T b;
    int32_T ia;
    c = 0.0;
    b = b_iy + m;
    for (ia = b_iy + 1; ia <= b; ia++) {
      c += x[(ia - b_iy) - 1] * A[ia - 1];
    }

    ia = div_nde_s32_floor(b_iy, 37);
    y[ia] += c;
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_ratiotest(const real_T solution_xstar[37],
  const real_T solution_searchDir[37], real_T workspace[12321], int32_T
  workingset_nVar, const real_T workingset_Aineq[12284], const real_T
  workingset_bineq[332], const int32_T workingset_indexLB[37], const int32_T
  workingset_sizes[5], const int32_T workingset_isActiveIdx[6], const boolean_T
  workingset_isActiveConstr[333], const int32_T workingset_nWConstr[5], real_T
  toldelta, real_T *alpha, boolean_T *newBlocking, int32_T *constrType, int32_T *
  constrIdx, real_T *b_toldelta)
{
  real_T tmp[2];
  real_T alphaTemp;
  real_T c;
  real_T denomTol;
  real_T p_max;
  real_T phaseOneCorrectionP;
  real_T pk_corrected;
  real_T workspace_0;
  int32_T b_k;
  int32_T ia;
  int32_T k;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  p_max = 0.0;
  denomTol = 2.2204460492503131E-13 * longitudinal_mpc_xnrm2_b(workingset_nVar,
    solution_searchDir);
  if (workingset_nWConstr[2] < 332) {
    std::memcpy(&workspace[0], &workingset_bineq[0], 332U * sizeof(real_T));
    longitudinal_mpc_xgemv_b4tv3(workingset_nVar, workingset_Aineq,
      solution_xstar, workspace);
    std::memset(&workspace[333], 0, 332U * sizeof(real_T));
    for (k = 0; k <= 12247; k += 37) {
      c = 0.0;
      b_k = k + workingset_nVar;
      for (ia = k + 1; ia <= b_k; ia++) {
        c += solution_searchDir[(ia - k) - 1] * workingset_Aineq[ia - 1];
      }

      b_k = div_nde_s32_floor(k, 37) + 333;
      workspace[b_k] += c;
    }

    for (b_k = 0; b_k < 332; b_k++) {
      workspace_0 = workspace[b_k + 333];
      if ((workspace_0 > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[2] + b_k) - 1])) {
        c = workspace[b_k];
        alphaTemp = std::fmin(std::abs(c - toldelta), (1.0E-5 - c) + toldelta) /
          workspace_0;
        if ((alphaTemp <= *alpha) && (std::abs(workspace_0) > p_max)) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = b_k + 1;
          *newBlocking = true;
        }

        alphaTemp = std::fmin(std::abs(c), 1.0E-5 - c) / workspace_0;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = b_k + 1;
          *newBlocking = true;
          p_max = std::abs(workspace_0);
        }
      }
    }
  }

  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_set_pd
      (solution_searchDir[workingset_nVar - 1], solution_xstar[workingset_nVar -
       1]), _mm_set1_pd(0.0)));
    c = tmp[0];
    phaseOneCorrectionP = tmp[1];
    b_k = workingset_sizes[3] - 2;
    for (ia = 0; ia <= b_k; ia++) {
      k = workingset_indexLB[ia];
      pk_corrected = -solution_searchDir[k - 1] - phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[3] + ia) - 1])) {
        workspace_0 = -solution_xstar[k - 1];
        alphaTemp = (workspace_0 - toldelta) - c;
        alphaTemp = std::fmin(std::abs(alphaTemp), 1.0E-5 - alphaTemp) /
          pk_corrected;
        if ((alphaTemp <= *alpha) && (std::abs(pk_corrected) > p_max)) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = ia + 1;
          *newBlocking = true;
        }

        alphaTemp = workspace_0 - c;
        alphaTemp = std::fmin(std::abs(alphaTemp), 1.0E-5 - alphaTemp) /
          pk_corrected;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = ia + 1;
          *newBlocking = true;
          p_max = std::abs(pk_corrected);
        }
      }
    }

    b_k = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    c = solution_searchDir[b_k];
    if ((-c > denomTol) && (!workingset_isActiveConstr[(workingset_isActiveIdx[3]
          + workingset_sizes[3]) - 2])) {
      workspace_0 = -solution_xstar[b_k];
      alphaTemp = workspace_0 - toldelta;
      alphaTemp = std::fmin(std::abs(alphaTemp), 1.0E-5 - alphaTemp) / -c;
      if ((alphaTemp <= *alpha) && (std::abs(c) > p_max)) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }

      alphaTemp = std::fmin(std::abs(workspace_0), 1.0E-5 - workspace_0) / -c;
      if (alphaTemp < *alpha) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
        p_max = std::abs(c);
      }
    }
  }

  *b_toldelta = toldelta + 6.608625846508183E-7;
  if (p_max > 0.0) {
    *alpha = std::fmax(*alpha, 6.608625846508183E-7 / p_max);
  }

  *newBlocking = (((!*newBlocking) || (!(*alpha > 1.0))) && (*newBlocking));
  *alpha = std::fmin(*alpha, 1.0);

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal__feasibleratiotest(const real_T
  solution_xstar[37], const real_T solution_searchDir[37], real_T workspace
  [12321], int32_T workingset_nVar, const real_T workingset_Aineq[12284], const
  real_T workingset_bineq[332], const int32_T workingset_indexLB[37], const
  int32_T workingset_sizes[5], const int32_T workingset_isActiveIdx[6], const
  boolean_T workingset_isActiveConstr[333], const int32_T workingset_nWConstr[5],
  boolean_T isPhaseOne, real_T *alpha, boolean_T *newBlocking, int32_T
  *constrType, int32_T *constrIdx)
{
  real_T tmp[2];
  real_T alphaTemp;
  real_T c;
  real_T denomTol;
  real_T phaseOneCorrectionP;
  real_T ratio;
  int32_T b_k;
  int32_T ia;
  int32_T k;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  denomTol = 2.2204460492503131E-13 * longitudinal_mpc_xnrm2_b(workingset_nVar,
    solution_searchDir);
  if (workingset_nWConstr[2] < 332) {
    std::memcpy(&workspace[0], &workingset_bineq[0], 332U * sizeof(real_T));
    longitudinal_mpc_xgemv_b4tv3(workingset_nVar, workingset_Aineq,
      solution_xstar, workspace);
    std::memset(&workspace[333], 0, 332U * sizeof(real_T));
    for (k = 0; k <= 12247; k += 37) {
      c = 0.0;
      b_k = k + workingset_nVar;
      for (ia = k + 1; ia <= b_k; ia++) {
        c += solution_searchDir[(ia - k) - 1] * workingset_Aineq[ia - 1];
      }

      b_k = div_nde_s32_floor(k, 37) + 333;
      workspace[b_k] += c;
    }

    for (b_k = 0; b_k < 332; b_k++) {
      c = workspace[b_k + 333];
      if ((c > denomTol) && (!workingset_isActiveConstr[(workingset_isActiveIdx
            [2] + b_k) - 1])) {
        alphaTemp = workspace[b_k];
        alphaTemp = std::fmin(std::abs(alphaTemp), 1.0E-5 - alphaTemp) / c;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = b_k + 1;
          *newBlocking = true;
        }
      }
    }
  }

  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    _mm_storeu_pd(&tmp[0], _mm_mul_pd(_mm_set_pd
      (solution_searchDir[workingset_nVar - 1], solution_xstar[workingset_nVar -
       1]), _mm_set1_pd(static_cast<real_T>(isPhaseOne))));
    c = tmp[0];
    phaseOneCorrectionP = tmp[1];
    b_k = workingset_sizes[3] - 2;
    for (ia = 0; ia <= b_k; ia++) {
      k = workingset_indexLB[ia];
      alphaTemp = -solution_searchDir[k - 1] - phaseOneCorrectionP;
      if ((alphaTemp > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[3] + ia) - 1])) {
        ratio = -solution_xstar[k - 1] - c;
        alphaTemp = std::fmin(std::abs(ratio), 1.0E-5 - ratio) / alphaTemp;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = ia + 1;
          *newBlocking = true;
        }
      }
    }

    b_k = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    c = -solution_searchDir[b_k];
    if ((c > denomTol) && (!workingset_isActiveConstr[(workingset_isActiveIdx[3]
          + workingset_sizes[3]) - 2])) {
      denomTol = -solution_xstar[b_k];
      alphaTemp = std::fmin(std::abs(denomTol), 1.0E-5 - denomTol) / c;
      if (alphaTemp < *alpha) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }
    }
  }

  if (!isPhaseOne) {
    *newBlocking = (((!*newBlocking) || (!(*alpha > 1.0))) && (*newBlocking));
    *alpha = std::fmin(*alpha, 1.0);
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longit_checkUnboundedOrIllPosed
  (sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution, const
   scJprC0tZnwNUG3KoXsnHVD_longi_T *objective)
{
  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (objective->objtype == 5) {
    if (longitudinal_mpc_xnrm2_b(objective->nvar, solution->searchDir) > 100.0 *
        static_cast<real_T>(objective->nvar) * 1.4901161193847656E-8) {
      solution->state = 3;
    } else {
      solution->state = 4;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::long_addBoundToActiveSetMatrix_(const
  s1rlrl8wYvAiB01zuzfeYY_longit_T *obj, int32_T TYPE, int32_T idx_local,
  s1rlrl8wYvAiB01zuzfeYY_longit_T *b_obj)
{
  int32_T colOffset;
  int32_T idx_bnd_local;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  *b_obj = *obj;
  b_obj->nWConstr[TYPE - 1] = obj->nWConstr[TYPE - 1] + 1;
  b_obj->isActiveConstr[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  b_obj->nActiveConstr = obj->nActiveConstr + 1;
  b_obj->Wid[obj->nActiveConstr] = TYPE;
  b_obj->Wlocalidx[obj->nActiveConstr] = idx_local;
  colOffset = 37 * obj->nActiveConstr - 1;
  if (TYPE == 5) {
    /* Check node always fails. would cause program termination and was eliminated */
  } else {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    idx_bnd_local = obj->indexLB[idx_local - 1];
    b_obj->bwset[obj->nActiveConstr] = 0.0;
  }

  if (idx_bnd_local - 2 >= 0) {
    std::memset(&b_obj->ATwset[colOffset + 1], 0, static_cast<uint32_T>
                ((((idx_bnd_local - 2) + colOffset) - colOffset) + 1) * sizeof
                (real_T));
  }

  b_obj->ATwset[idx_bnd_local + colOffset] = static_cast<real_T>(TYPE == 5) *
    2.0 - 1.0;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  if (idx_bnd_local + 1 <= obj->nVar) {
    std::memset(&b_obj->ATwset[(idx_bnd_local + colOffset) + 1], 0,
                static_cast<uint32_T>(((obj->nVar + colOffset) - idx_bnd_local)
      - colOffset) * sizeof(real_T));
  }

  switch (obj->probType) {
   case 3:
   case 2:
    break;

   default:
    b_obj->ATwset[obj->nVar + colOffset] = -1.0;
    break;
  }
}

void longitudinal_mpc::lo_checkStoppingAndUpdateFval_b(int32_T activeSetChangeID,
  const real_T f[36], sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution,
  sK6ng1KsrjtpGD3SgQmbb8_longit_T *memspace, const
  scJprC0tZnwNUG3KoXsnHVD_longi_T *objective, s1rlrl8wYvAiB01zuzfeYY_longit_T
  *workingset, sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager, real_T
  options_ObjectiveLimit, real_T runTimeOptions_ConstrRelTolFact, boolean_T
  updateFval, int32_T *b_activeSetChangeID, boolean_T *b_updateFval)
{
  real_T b;
  real_T tempMaxConstr;
  boolean_T nonDegenerateWset;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  *b_updateFval = updateFval;
  *b_activeSetChangeID = activeSetChangeID;
  solution->iterations++;
  if ((solution->iterations >= 2000) && ((solution->state != 1) ||
       (objective->objtype == 5))) {
    solution->state = 0;
  }

  if (solution->iterations - solution->iterations / 50 * 50 == 0) {
    longitudinal_mpc_B.workingset_c = *workingset;
    longi_maxConstraintViolation_b4(&longitudinal_mpc_B.workingset_c,
      solution->xstar, &b, workingset);
    solution->maxConstr = b;
    tempMaxConstr = b;
    if (objective->objtype == 5) {
      tempMaxConstr = b - solution->xstar[objective->nvar - 1];
    }

    if (tempMaxConstr > 1.0E-5 * runTimeOptions_ConstrRelTolFact) {
      if (static_cast<uint8_T>(objective->nvar) - 1 >= 0) {
        std::memcpy(&solution->searchDir[0], &solution->xstar[0],
                    static_cast<uint32_T>((static_cast<uint8_T>(objective->nvar)
          - 1) + 1) * sizeof(real_T));
      }

      nonDegenerateWset = longitu_feasibleX0ForWorkingSet
        (memspace->workspace_float, solution->searchDir, workingset, qrmanager);
      if ((!nonDegenerateWset) && (solution->state != 0)) {
        solution->state = -2;
      }

      *b_activeSetChangeID = 0;
      longitudinal_mpc_B.workingset_c = *workingset;
      longi_maxConstraintViolation_b4(&longitudinal_mpc_B.workingset_c,
        solution->searchDir, &tempMaxConstr, workingset);
      if (tempMaxConstr < b) {
        if (static_cast<uint8_T>(objective->nvar) - 1 >= 0) {
          std::memcpy(&solution->xstar[0], &solution->searchDir[0],
                      static_cast<uint32_T>((static_cast<uint8_T>
            (objective->nvar) - 1) + 1) * sizeof(real_T));
        }

        solution->maxConstr = tempMaxConstr;
      }
    }
  }

  if (updateFval) {
    b = longitudina_computeFval_ReuseHx(objective, memspace->workspace_float, f,
      solution->xstar);
    solution->fstar = b;
    if ((b < options_ObjectiveLimit) && ((solution->state != 0) ||
         (objective->objtype != 5))) {
      solution->state = 2;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_iterate_b(const real_T H[1296], const
  real_T f[36], sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution,
  sK6ng1KsrjtpGD3SgQmbb8_longit_T *memspace, s1rlrl8wYvAiB01zuzfeYY_longit_T
  *workingset, sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager,
  skczSSN0IseekIuhVW4znFG_longi_T *cholmanager, scJprC0tZnwNUG3KoXsnHVD_longi_T *
  objective, real_T options_ObjectiveLimit, real_T options_StepTolerance, real_T
  runTimeOptions_ConstrRelTolFact, real_T runTimeOptions_ProbRelTolFactor,
  boolean_T runTimeOptions_RemainFeasible)
{
  __m128d tmp;
  __m128d tmp_0;
  real_T c;
  real_T c_tmp;
  real_T normDelta;
  real_T s;
  real_T tolDelta;
  int32_T TYPE;
  int32_T activeSetChangeID;
  int32_T exitg1;
  int32_T globalActiveConstrIdx;
  int32_T iAw0;
  int32_T iQR0;
  int32_T idx;
  int32_T ix;
  int32_T iyend;
  int32_T nVar_tmp_tmp;
  int32_T workingIdx;
  boolean_T guard1;
  boolean_T guard11;
  boolean_T guard2;
  boolean_T newBlocking;
  boolean_T subProblemChanged;
  boolean_T updateFval;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  tolDelta = 6.7434957617430445E-7;
  nVar_tmp_tmp = workingset->nVar;
  globalActiveConstrIdx = 0;
  longitudina_computeGrad_StoreHx(objective, H, f, solution->xstar);
  solution->fstar = longitudina_computeFval_ReuseHx(objective,
    memspace->workspace_float, f, solution->xstar);
  if (solution->iterations < 2000) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }

  std::memset(&solution->lambda[0], 0, 333U * sizeof(real_T));
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      guard11 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          workingIdx = (workingset->nActiveConstr - 1) * 37;
          if (qrmanager->mrows <= qrmanager->ncols + 1) {
            qrmanager->minRowCol = qrmanager->mrows;
          } else {
            qrmanager->minRowCol = qrmanager->ncols + 1;
          }

          iQR0 = 37 * qrmanager->ncols;
          if (qrmanager->mrows != 0) {
            iyend = iQR0 + qrmanager->mrows;
            if (iQR0 + 1 <= iyend) {
              std::memset(&qrmanager->QR[iQR0], 0, static_cast<uint32_T>(iyend -
                iQR0) * sizeof(real_T));
            }

            iyend = (qrmanager->mrows - 1) * 37 + 1;
            for (idx = 1; idx <= iyend; idx += 37) {
              c = 0.0;
              ix = idx + qrmanager->mrows;
              for (iAw0 = idx; iAw0 < ix; iAw0++) {
                c += workingset->ATwset[(workingIdx + iAw0) - idx] *
                  qrmanager->Q[iAw0 - 1];
              }

              ix = div_nde_s32_floor(idx - 1, 37) + iQR0;
              qrmanager->QR[ix] += c;
            }
          }

          qrmanager->ncols++;
          qrmanager->jpvt[qrmanager->ncols - 1] = qrmanager->ncols;
          for (idx = qrmanager->mrows - 2; idx + 2 > qrmanager->ncols; idx--) {
            ix = (qrmanager->ncols - 1) * 37 + idx;
            longitudinal_mpc_xrotg(qrmanager->QR[ix], qrmanager->QR[ix + 1],
              &qrmanager->QR[ix], &c, &normDelta, &s);
            qrmanager->QR[ix + 1] = c;
            workingIdx = 37 * idx;
            iAw0 = qrmanager->mrows;
            if (qrmanager->mrows >= 1) {
              for (iyend = 0; iyend < iAw0; iyend++) {
                iQR0 = workingIdx + iyend;
                c_tmp = qrmanager->Q[iQR0 + 37];
                c = c_tmp * s + qrmanager->Q[iQR0] * normDelta;
                qrmanager->Q[iQR0 + 37] = c_tmp * normDelta - qrmanager->Q[iQR0]
                  * s;
                qrmanager->Q[iQR0] = c;
              }
            }
          }
          break;

         case -1:
          longitudinal_m_deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          longitudinal_mpc_factorQR(qrmanager, workingset->ATwset, nVar_tmp_tmp,
            workingset->nActiveConstr);
          longitudinal_mpc_computeQ_(qrmanager, qrmanager->mrows);
          break;
        }

        longitudinal_mpc_compute_deltax(H, solution, memspace, qrmanager,
          cholmanager, objective);
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          normDelta = longitudinal_mpc_xnrm2_b(nVar_tmp_tmp, solution->searchDir);
          guard11 = true;
        }
      } else {
        if (nVar_tmp_tmp - 1 >= 0) {
          std::memset(&solution->searchDir[0], 0, static_cast<uint32_T>
                      (nVar_tmp_tmp) * sizeof(real_T));
        }

        normDelta = 0.0;
        guard11 = true;
      }

      if (guard11) {
        if ((!subProblemChanged) || (normDelta < options_StepTolerance) ||
            (workingset->nActiveConstr >= nVar_tmp_tmp)) {
          workingIdx = qrmanager->ncols;
          if (qrmanager->ncols > 0) {
            guard1 = false;
            if (objective->objtype != 4) {
              normDelta = 100.0 * static_cast<real_T>(qrmanager->mrows) *
                2.2204460492503131E-16;
              updateFval = ((qrmanager->mrows > 0) && (qrmanager->ncols > 0));
              if (updateFval) {
                idx = qrmanager->ncols;
                guard2 = false;
                if (qrmanager->mrows < qrmanager->ncols) {
                  iyend = ((qrmanager->ncols - 1) * 37 + qrmanager->mrows) - 1;
                  while ((idx > qrmanager->mrows) && (std::abs(qrmanager->
                           QR[iyend]) >= normDelta)) {
                    idx--;
                    iyend -= 37;
                  }

                  updateFval = (idx == qrmanager->mrows);
                  if (!updateFval) {
                  } else {
                    guard2 = true;
                  }
                } else {
                  guard2 = true;
                }

                if (guard2) {
                  iyend = ((idx - 1) * 37 + idx) - 1;
                  while ((idx >= 1) && (std::abs(qrmanager->QR[iyend]) >=
                                        normDelta)) {
                    idx--;
                    iyend -= 38;
                  }

                  updateFval = (idx == 0);
                }
              }

              if (!updateFval) {
                solution->state = -7;
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }

            if (guard1) {
              if (qrmanager->mrows != 0) {
                std::memset(&memspace->workspace_float[0], 0, static_cast<
                            uint32_T>((qrmanager->ncols - 1) + 1) * sizeof
                            (real_T));
                iyend = (qrmanager->ncols - 1) * 37;
                for (idx = 1; idx <= iyend + 1; idx += 37) {
                  c = 0.0;
                  ix = idx + qrmanager->mrows;
                  for (iAw0 = idx; iAw0 < ix; iAw0++) {
                    c += qrmanager->Q[iAw0 - 1] * objective->grad[iAw0 - idx];
                  }

                  ix = div_nde_s32_floor(idx - 1, 37);
                  memspace->workspace_float[ix] += c;
                }
              }

              for (idx = workingIdx; idx >= 1; idx--) {
                iQR0 = ((idx - 1) * 37 + idx) - 1;
                memspace->workspace_float[idx - 1] /= qrmanager->QR[iQR0];
                iyend = idx - 2;
                for (iAw0 = 0; iAw0 <= iyend; iAw0++) {
                  ix = (idx - iAw0) - 2;
                  memspace->workspace_float[ix] -= qrmanager->QR[(iQR0 - iAw0) -
                    1] * memspace->workspace_float[idx - 1];
                }
              }

              idx = (qrmanager->ncols / 2) << 1;
              iAw0 = idx - 2;
              for (iyend = 0; iyend <= iAw0; iyend += 2) {
                tmp = _mm_loadu_pd(&memspace->workspace_float[iyend]);
                _mm_storeu_pd(&solution->lambda[iyend], _mm_mul_pd(tmp,
                  _mm_set1_pd(-1.0)));
              }

              for (iyend = idx; iyend < workingIdx; iyend++) {
                solution->lambda[iyend] = -memspace->workspace_float[iyend];
              }
            }
          }

          if ((solution->state != -7) || (workingset->nActiveConstr >
               nVar_tmp_tmp)) {
            workingIdx = 0;
            normDelta = 0.0 * runTimeOptions_ProbRelTolFactor *
              static_cast<real_T>(TYPE != 5);
            iyend = workingset->nWConstr[0] + workingset->nWConstr[1];
            iAw0 = workingset->nActiveConstr;
            for (idx = iyend + 1; idx <= iAw0; idx++) {
              c = solution->lambda[idx - 1];
              if (c < normDelta) {
                normDelta = c;
                workingIdx = idx;
              }
            }

            if (workingIdx == 0) {
              solution->state = 1;
            } else {
              activeSetChangeID = -1;
              globalActiveConstrIdx = workingIdx;
              subProblemChanged = true;
              longitudinal_mpc_removeConstr(workingset, workingIdx);
              if (workingIdx < workingset->nActiveConstr + 1) {
                solution->lambda[workingIdx - 1] = solution->lambda
                  [workingset->nActiveConstr];
              }

              solution->lambda[workingset->nActiveConstr] = 0.0;
            }
          } else {
            workingIdx = workingset->nActiveConstr - 1;
            activeSetChangeID = 0;
            globalActiveConstrIdx = workingset->nActiveConstr;
            subProblemChanged = true;
            longitudinal_mpc_removeConstr(workingset, workingset->nActiveConstr);
            solution->lambda[workingIdx] = 0.0;
          }

          updateFval = false;
        } else {
          updateFval = (TYPE == 5);
          if (updateFval || runTimeOptions_RemainFeasible) {
            longitudinal__feasibleratiotest(solution->xstar, solution->searchDir,
              memspace->workspace_float, workingset->nVar, workingset->Aineq,
              workingset->bineq, workingset->indexLB, workingset->sizes,
              workingset->isActiveIdx, workingset->isActiveConstr,
              workingset->nWConstr, updateFval, &normDelta, &newBlocking, &idx,
              &workingIdx);
          } else {
            longitudinal_mpc_ratiotest(solution->xstar, solution->searchDir,
              memspace->workspace_float, workingset->nVar, workingset->Aineq,
              workingset->bineq, workingset->indexLB, workingset->sizes,
              workingset->isActiveIdx, workingset->isActiveConstr,
              workingset->nWConstr, tolDelta, &normDelta, &newBlocking, &idx,
              &workingIdx, &tolDelta);
          }

          if (newBlocking) {
            switch (idx) {
             case 3:
              workingset->nWConstr[2]++;
              workingset->isActiveConstr[(workingset->isActiveIdx[2] +
                workingIdx) - 2] = true;
              workingset->nActiveConstr++;
              workingset->Wid[workingset->nActiveConstr - 1] = 3;
              workingset->Wlocalidx[workingset->nActiveConstr - 1] = workingIdx;
              activeSetChangeID = (workingIdx - 1) * 37;
              iAw0 = (workingset->nActiveConstr - 1) * 37;
              iyend = workingset->nVar;
              for (idx = 0; idx < iyend; idx++) {
                workingset->ATwset[iAw0 + idx] = workingset->
                  Aineq[activeSetChangeID + idx];
              }

              workingset->bwset[workingset->nActiveConstr - 1] =
                workingset->bineq[workingIdx - 1];
              break;

             case 4:
              longitudinal_mpc_B.workingset = *workingset;
              long_addBoundToActiveSetMatrix_(&longitudinal_mpc_B.workingset, 4,
                workingIdx, workingset);
              break;

             default:
              longitudinal_mpc_B.workingset = *workingset;
              long_addBoundToActiveSetMatrix_(&longitudinal_mpc_B.workingset, 5,
                workingIdx, workingset);
              break;
            }

            activeSetChangeID = 1;
          } else {
            longit_checkUnboundedOrIllPosed(solution, objective);
            subProblemChanged = false;
            if (workingset->nActiveConstr == 0) {
              solution->state = 1;
            }
          }

          if (!(normDelta == 0.0)) {
            idx = (nVar_tmp_tmp / 2) << 1;
            iAw0 = idx - 2;
            for (iyend = 0; iyend <= iAw0; iyend += 2) {
              tmp = _mm_loadu_pd(&solution->searchDir[iyend]);
              tmp_0 = _mm_loadu_pd(&solution->xstar[iyend]);
              _mm_storeu_pd(&solution->xstar[iyend], _mm_add_pd(_mm_mul_pd
                (_mm_set1_pd(normDelta), tmp), tmp_0));
            }

            for (iyend = idx; iyend < nVar_tmp_tmp; iyend++) {
              solution->xstar[iyend] += normDelta * solution->searchDir[iyend];
            }
          }

          longitudina_computeGrad_StoreHx(objective, H, f, solution->xstar);
          updateFval = true;
        }

        lo_checkStoppingAndUpdateFval_b(activeSetChangeID, f, solution, memspace,
          objective, workingset, qrmanager, options_ObjectiveLimit,
          runTimeOptions_ConstrRelTolFact, updateFval, &activeSetChangeID,
          &updateFval);
      }
    } else {
      if (!updateFval) {
        solution->fstar = longitudina_computeFval_ReuseHx(objective,
          memspace->workspace_float, f, solution->xstar);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_phaseone_b4(const real_T H[1296], const
  real_T f[36], sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution,
  sK6ng1KsrjtpGD3SgQmbb8_longit_T *memspace, s1rlrl8wYvAiB01zuzfeYY_longit_T
  *workingset, sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager,
  skczSSN0IseekIuhVW4znFG_longi_T *cholmanager, const
  sL9bDKomAYkxZSVrG9w6En_longit_T *runTimeOptions,
  scJprC0tZnwNUG3KoXsnHVD_longi_T *objective, s7GW9uShiIXbHYZwohNmyqD_longi_T
  *options)
{
  int32_T idxEndIneq;
  int32_T idxStartIneq_tmp;
  int32_T mConstr;
  static const char_T t0_FiniteDifferenceType[7]{ 'f', 'o', 'r', 'w', 'a', 'r',
    'd' };

  static const char_T t0_Algorithm[10]{ 'a', 'c', 't', 'i', 'v', 'e', '-', 's',
    'e', 't' };

  static const char_T t0_SolverName[8]{ 'q', 'u', 'a', 'd', 'p', 'r', 'o', 'g' };

  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  options->NonFiniteSupport = true;
  options->IterDisplaySQP = false;
  options->InitDamping = 0.01;
  for (mConstr = 0; mConstr < 7; mConstr++) {
    options->FiniteDifferenceType[mConstr] = t0_FiniteDifferenceType[mConstr];
  }

  options->SpecifyObjectiveGradient = false;
  options->ScaleProblem = false;
  options->SpecifyConstraintGradient = false;
  options->FiniteDifferenceStepSize = -1.0;
  options->MaxFunctionEvaluations = -1.0;
  options->IterDisplayQP = false;
  options->PricingTolerance = 0.0;
  for (mConstr = 0; mConstr < 10; mConstr++) {
    options->Algorithm[mConstr] = t0_Algorithm[mConstr];
  }

  options->ConstraintTolerance = 1.0E-5;
  options->OptimalityTolerance = 1.0E-6;
  options->MaxIterations = 2000.0;
  options->FunctionTolerance = (rtInf);
  for (mConstr = 0; mConstr < 8; mConstr++) {
    options->SolverName[mConstr] = t0_SolverName[mConstr];
  }

  options->CheckGradients = false;
  options->DiffMaxChange = (rtInf);
  options->DiffMinChange = 0.0;
  options->Diagnostics[0] = 'o';
  options->Display[0] = 'o';
  options->FunValCheck[0] = 'o';
  options->Diagnostics[1] = 'f';
  options->Display[1] = 'f';
  options->FunValCheck[1] = 'f';
  options->Diagnostics[2] = 'f';
  options->Display[2] = 'f';
  options->FunValCheck[2] = 'f';
  options->UseParallel = false;
  options->LinearSolver[0] = 'a';
  options->LinearSolver[1] = 'u';
  options->LinearSolver[2] = 't';
  options->LinearSolver[3] = 'o';
  options->SubproblemAlgorithm[0] = 'c';
  options->SubproblemAlgorithm[1] = 'g';
  solution->xstar[36] = solution->maxConstr + 1.0;
  longitudinal_mpc_setProblemType(workingset, 1);
  idxStartIneq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  idxEndIneq = workingset->nActiveConstr;
  for (mConstr = idxStartIneq_tmp + 1; mConstr <= idxEndIneq; mConstr++) {
    workingset->isActiveConstr[(workingset->isActiveIdx[workingset->Wid[mConstr
      - 1] - 1] + workingset->Wlocalidx[mConstr - 1]) - 2] = false;
  }

  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = idxStartIneq_tmp;
  std::memset(&objective->grad[0], 0, 37U * sizeof(real_T));
  std::memset(&objective->Hx[0], 0, 36U * sizeof(real_T));
  objective->maxVar = 37;
  objective->beta = 0.0;
  objective->rho = 0.0;
  objective->prev_objtype = 3;
  objective->prev_nvar = 36;
  objective->prev_hasLinear = true;
  objective->objtype = 5;
  objective->nvar = 37;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  options->ObjectiveLimit = 1.0E-5 * runTimeOptions->ConstrRelTolFactor;
  solution->fstar = longitudinal_mpc_computeFval(objective,
    memspace->workspace_float, H, f, solution->xstar);
  solution->state = 5;
  longitudinal_mpc_iterate_b(H, f, solution, memspace, workingset, qrmanager,
    cholmanager, objective, options->ObjectiveLimit, 1.4901161193847657E-10,
    runTimeOptions->ConstrRelTolFactor, runTimeOptions->ProbRelTolFactor,
    runTimeOptions->RemainFeasible);
  if (workingset->isActiveConstr[(workingset->isActiveIdx[3] + workingset->
       sizes[3]) - 2]) {
    mConstr = 0;
    exitg1 = false;
    while ((!exitg1) && (mConstr + 1 <= workingset->nActiveConstr)) {
      if ((workingset->Wid[mConstr] == 4) && (workingset->Wlocalidx[mConstr] ==
           workingset->sizes[3])) {
        longitudinal_mpc_removeConstr(workingset, mConstr + 1);
        exitg1 = true;
      } else {
        mConstr++;
      }
    }
  }

  for (mConstr = workingset->nActiveConstr; mConstr > 36; mConstr--) {
    longitudinal_mpc_removeConstr(workingset, mConstr);
  }

  solution->maxConstr = solution->xstar[36];
  longitudinal_mpc_setProblemType(workingset, 3);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
  options->ObjectiveLimit = -1.0E+20;
  options->StepTolerance = 1.0E-8;

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

int32_T longitudinal_mpc::longitudinal_RemoveDependentEq_
  (sK6ng1KsrjtpGD3SgQmbb8_longit_T *memspace, const
   s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset, sCCE9T0P8IQkk6PxUdCnbBE_longi_T *
   qrmanager)
{
  real_T qtb;
  real_T tol;
  int32_T b_idx_col;
  int32_T ix;
  int32_T mTotalWorkingEq_tmp;
  int32_T mWorkingFixed;
  int32_T nDepInd;
  int32_T offsetQR_tmp;
  int32_T totalRank;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  mWorkingFixed = workingset->nWConstr[0] - 1;
  mTotalWorkingEq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp > 0) {
    offsetQR_tmp = static_cast<uint8_T>(workingset->nVar) - 1;
    for (totalRank = 0; totalRank < mTotalWorkingEq_tmp; totalRank++) {
      for (b_idx_col = 0; b_idx_col <= offsetQR_tmp; b_idx_col++) {
        qrmanager->QR[totalRank + 37 * b_idx_col] = workingset->ATwset[37 *
          totalRank + b_idx_col];
      }
    }

    b_idx_col = mTotalWorkingEq_tmp - workingset->nVar;
    if (b_idx_col > 0) {
      nDepInd = b_idx_col;
    }

    std::memset(&qrmanager->jpvt[0], 0, static_cast<uint32_T>
                ((static_cast<uint8_T>(workingset->nVar) - 1) + 1) * sizeof
                (int32_T));
    longitudinal_mpc_B.qrmanager_c = *qrmanager;
    longitudinal_mpc_factorQRE(&longitudinal_mpc_B.qrmanager_c,
      mTotalWorkingEq_tmp, workingset->nVar, qrmanager);
    tol = 100.0 * static_cast<real_T>(workingset->nVar) * 2.2204460492503131E-16;
    if (workingset->nVar <= mTotalWorkingEq_tmp) {
      totalRank = workingset->nVar;
    } else {
      totalRank = mTotalWorkingEq_tmp;
    }

    totalRank += (totalRank - 1) * 37;
    while ((totalRank > 0) && (std::abs(qrmanager->QR[totalRank - 1]) < tol)) {
      totalRank -= 38;
      nDepInd++;
    }

    if (nDepInd > 0) {
      longitudinal_mpc_computeQ_(qrmanager, qrmanager->mrows);
      b_idx_col = 1;
      exitg1 = false;
      while ((!exitg1) && (b_idx_col - 1 <= nDepInd - 1)) {
        qtb = 0.0;
        ix = (mTotalWorkingEq_tmp - b_idx_col) * 37;
        for (totalRank = 0; totalRank < mTotalWorkingEq_tmp; totalRank++) {
          qtb += qrmanager->Q[ix + totalRank] * workingset->bwset[totalRank];
        }

        if (std::abs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          b_idx_col++;
        }
      }
    }

    if (nDepInd > 0) {
      for (b_idx_col = 0; b_idx_col < mTotalWorkingEq_tmp; b_idx_col++) {
        totalRank = 37 * b_idx_col - 1;
        std::memcpy(&qrmanager->QR[totalRank + 1], &workingset->ATwset[totalRank
                    + 1], static_cast<uint32_T>(((offsetQR_tmp + totalRank) -
          totalRank) + 1) * sizeof(real_T));
      }

      for (totalRank = 0; totalRank <= mWorkingFixed; totalRank++) {
        qrmanager->jpvt[totalRank] = 1;
      }

      mWorkingFixed = workingset->nWConstr[0];
      if (mWorkingFixed + 1 <= mTotalWorkingEq_tmp) {
        std::memset(&qrmanager->jpvt[mWorkingFixed], 0, static_cast<uint32_T>
                    (mTotalWorkingEq_tmp - mWorkingFixed) * sizeof(int32_T));
      }

      longitudinal_mpc_B.qrmanager_c = *qrmanager;
      longitudinal_mpc_factorQRE(&longitudinal_mpc_B.qrmanager_c,
        workingset->nVar, mTotalWorkingEq_tmp, qrmanager);
      for (mWorkingFixed = 0; mWorkingFixed < nDepInd; mWorkingFixed++) {
        memspace->workspace_int[mWorkingFixed] = qrmanager->jpvt
          [(mTotalWorkingEq_tmp - nDepInd) + mWorkingFixed];
      }

      longitudinal_mpc_countsort(memspace->workspace_int, nDepInd,
        memspace->workspace_sort, 1, mTotalWorkingEq_tmp);
      if (mTotalWorkingEq_tmp != 0) {
        for (mWorkingFixed = nDepInd; mWorkingFixed >= 1; mWorkingFixed--) {
          b_idx_col = memspace->workspace_int[mWorkingFixed - 1];
          if ((b_idx_col <= mTotalWorkingEq_tmp) &&
              (!((workingset->nActiveConstr == mTotalWorkingEq_tmp) ||
                 (b_idx_col == mTotalWorkingEq_tmp)))) {
            /* Check node always fails. would cause program termination and was eliminated */
          } else {
            /* Check node always fails. would cause program termination and was eliminated */
          }
        }
      }
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
  return nDepInd;
}

void longitudinal_mpc::longitud_PresolveWorkingSet_b4t(const
  sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution, sK6ng1KsrjtpGD3SgQmbb8_longit_T
  *memspace, s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset,
  sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager, sJvHSAPlL1SbbU0gnSE72ZG_longi_T
  *b_solution)
{
  real_T constrViolation;
  int32_T i;
  int32_T idxEndIneq;
  int32_T idxStartIneq_tmp;
  boolean_T guard1;
  boolean_T okWorkingSet;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  *b_solution = *solution;
  b_solution->state = 82;
  i = longitudinal_RemoveDependentEq_(memspace, workingset, qrmanager);
  if ((i != -1) && (workingset->nActiveConstr <= 37)) {
    longitudin_RemoveDependentIneq_(workingset, qrmanager, memspace, 100.0);
    std::memcpy(&b_solution->xstar[0], &solution->xstar[0], 37U * sizeof(real_T));
    okWorkingSet = longitu_feasibleX0ForWorkingSet(memspace->workspace_float,
      b_solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      longitudin_RemoveDependentIneq_(workingset, qrmanager, memspace, 1000.0);
      okWorkingSet = longitu_feasibleX0ForWorkingSet(memspace->workspace_float,
        b_solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        b_solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      if (workingset->nWConstr[0] + workingset->nWConstr[1] == workingset->nVar)
      {
        longitudinal_mpc_B.workingset_k = *workingset;
        longi_maxConstraintViolation_b4(&longitudinal_mpc_B.workingset_k,
          b_solution->xstar, &constrViolation, workingset);
        if (constrViolation > 1.0E-5) {
          b_solution->state = -2;
        }
      }
    }
  } else {
    b_solution->state = -3;
    idxStartIneq_tmp = workingset->nWConstr[0] + workingset->nWConstr[1];
    idxEndIneq = workingset->nActiveConstr;
    for (i = idxStartIneq_tmp + 1; i <= idxEndIneq; i++) {
      workingset->isActiveConstr[(workingset->isActiveIdx[workingset->Wid[i - 1]
        - 1] + workingset->Wlocalidx[i - 1]) - 2] = false;
    }

    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = idxStartIneq_tmp;
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

boolean_T longitudinal_mpc::longitudinal_mpc_strcmp(const char_T a[8])
{
  int32_T b_kstr;
  boolean_T b_bool;
  static const char_T tmp[128]{ '\x00', '\x01', '\x02', '\x03', '\x04', '\x05',
    '\x06', '\a', '\b', '\t', '\n', '\v', '\f', '\r', '\x0e', '\x0f', '\x10',
    '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18', '\x19',
    '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#', '$',
    '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2', '3',
    '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a', 'b',
    'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q',
    'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']', '^', '_', '`',
    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
    'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}', '~',
    '\x7f' };

  static const char_T tmp_0[8]{ 'q', 'u', 'a', 'd', 'p', 'r', 'o', 'g' };

  b_bool = false;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  b_kstr = 1;
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (b_kstr - 1 < 8) {
      if (tmp[static_cast<int32_T>(a[b_kstr - 1])] != tmp[static_cast<int32_T>
          (tmp_0[b_kstr - 1])]) {
        exitg1 = 1;
      } else {
        b_kstr++;
      }
    } else {
      b_bool = true;
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return b_bool;
}

void longitudinal_mpc::longitudin_computeFirstOrderOpt
  (sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution, const
   scJprC0tZnwNUG3KoXsnHVD_longi_T *objective, const
   s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset, real_T workspace[12321])
{
  real_T infNorm;
  int32_T b_k;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  std::memcpy(&workspace[0], &objective->grad[0], static_cast<uint32_T>((
    static_cast<uint8_T>(workingset->nVar) - 1) + 1) * sizeof(real_T));
  if (workingset->nActiveConstr != 0) {
    int32_T b;
    int32_T ix;
    ix = 0;
    b = (workingset->nActiveConstr - 1) * 37;
    for (b_k = 1; b_k <= b + 1; b_k += 37) {
      int32_T d;
      d = b_k + workingset->nVar;
      for (int32_T k{b_k}; k < d; k++) {
        int32_T tmp;
        tmp = k - b_k;
        workspace[tmp] += workingset->ATwset[k - 1] * solution->lambda[ix];
      }

      ix++;
    }
  }

  infNorm = 0.0;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k <= static_cast<uint8_T>(workingset->nVar) - 1)) {
    real_T abs_workspace_i;
    abs_workspace_i = std::abs(workspace[b_k]);
    if (std::isnan(abs_workspace_i)) {
      infNorm = (rtNaN);
      exitg1 = true;
    } else {
      infNorm = std::fmax(infNorm, abs_workspace_i);
      b_k++;
    }
  }

  solution->firstorderopt = infNorm;

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_phaseone_b4t(const real_T H[1296], const
  real_T f[36], sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution,
  sK6ng1KsrjtpGD3SgQmbb8_longit_T *memspace, const
  s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset, sCCE9T0P8IQkk6PxUdCnbBE_longi_T
  *qrmanager, skczSSN0IseekIuhVW4znFG_longi_T *cholmanager,
  scJprC0tZnwNUG3KoXsnHVD_longi_T *objective, s7GW9uShiIXbHYZwohNmyqD_longi_T
  *options, const sL9bDKomAYkxZSVrG9w6En_longit_T *runTimeOptions,
  s1rlrl8wYvAiB01zuzfeYY_longit_T *b_workingset)
{
  int32_T idxEndIneq;
  int32_T idxStartIneq;
  int32_T mConstr;
  boolean_T exitg1;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  solution->xstar[36] = solution->maxConstr + 1.0;
  *b_workingset = *workingset;
  if (workingset->probType == 3) {
    mConstr = 1;
  } else {
    mConstr = 4;
  }

  longitudinal_mpc_setProblemType(b_workingset, mConstr);
  idxStartIneq = b_workingset->nWConstr[0] + b_workingset->nWConstr[1];
  idxEndIneq = b_workingset->nActiveConstr;
  for (mConstr = idxStartIneq + 1; mConstr <= idxEndIneq; mConstr++) {
    b_workingset->isActiveConstr[(b_workingset->isActiveIdx[b_workingset->
      Wid[mConstr - 1] - 1] + b_workingset->Wlocalidx[mConstr - 1]) - 2] = false;
  }

  b_workingset->nWConstr[2] = 0;
  b_workingset->nWConstr[3] = 0;
  b_workingset->nWConstr[4] = 0;
  b_workingset->nActiveConstr = b_workingset->nWConstr[0] +
    b_workingset->nWConstr[1];
  objective->prev_objtype = objective->objtype;
  objective->prev_nvar = objective->nvar;
  objective->prev_hasLinear = objective->hasLinear;
  objective->objtype = 5;
  objective->nvar = 37;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  options->ObjectiveLimit = 1.0E-5 * runTimeOptions->ConstrRelTolFactor;
  options->StepTolerance = 1.4901161193847657E-10;
  solution->fstar = longitudinal_mpc_computeFval(objective,
    memspace->workspace_float, H, f, solution->xstar);
  solution->state = 5;
  longitudinal_mpc_iterate_b(H, f, solution, memspace, b_workingset, qrmanager,
    cholmanager, objective, options->ObjectiveLimit, options->StepTolerance,
    runTimeOptions->ConstrRelTolFactor, runTimeOptions->ProbRelTolFactor,
    runTimeOptions->RemainFeasible);
  if (b_workingset->isActiveConstr[(b_workingset->isActiveIdx[3] +
       b_workingset->sizes[3]) - 2]) {
    mConstr = 0;
    exitg1 = false;
    while ((!exitg1) && (mConstr + 1 <= b_workingset->nActiveConstr)) {
      if ((b_workingset->Wid[mConstr] == 4) && (b_workingset->Wlocalidx[mConstr]
           == b_workingset->sizes[3])) {
        longitudinal_mpc_removeConstr(b_workingset, mConstr + 1);
        exitg1 = true;
      } else {
        mConstr++;
      }
    }
  }

  mConstr = b_workingset->nActiveConstr;
  while ((mConstr > 0) && (mConstr > workingset->nVar)) {
    longitudinal_mpc_removeConstr(b_workingset, mConstr);
    mConstr--;
  }

  solution->maxConstr = solution->xstar[36];
  longitudinal_mpc_setProblemType(b_workingset, workingset->probType);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
  options->ObjectiveLimit = -1.0E+20;
  options->StepTolerance = 1.0E-8;

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_driver(const real_T H[1296], const
  real_T f[36], sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution, const real_T
  memspace_workspace_float[12321], const int32_T memspace_workspace_int[333],
  const int32_T memspace_workspace_sort[333], const
  s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset, skczSSN0IseekIuhVW4znFG_longi_T
  *cholmanager, sL9bDKomAYkxZSVrG9w6En_longit_T runTimeOptions,
  sK6ng1KsrjtpGD3SgQmbb8_longit_T *b_memspace, s1rlrl8wYvAiB01zuzfeYY_longit_T
  *b_workingset, sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager,
  scJprC0tZnwNUG3KoXsnHVD_longi_T *objective)
{
  s7GW9uShiIXbHYZwohNmyqD_longi_T e_options;
  real_T b;
  real_T tmp;
  int32_T i;
  static const char_T t2_FiniteDifferenceType[7]{ 'f', 'o', 'r', 'w', 'a', 'r',
    'd' };

  static const char_T t2_Algorithm[10]{ 'a', 'c', 't', 'i', 'v', 'e', '-', 's',
    'e', 't' };

  static const char_T t2_SolverName[8]{ 'q', 'u', 'a', 'd', 'p', 'r', 'o', 'g' };

  sJvHSAPlL1SbbU0gnSE72ZG_longi_T solution_0;
  boolean_T guard1;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  std::memset(&objective->grad[0], 0, 37U * sizeof(real_T));
  std::memset(&objective->Hx[0], 0, 36U * sizeof(real_T));
  objective->hasLinear = true;
  objective->nvar = 36;
  objective->maxVar = 37;
  objective->beta = 0.0;
  objective->rho = 0.0;
  objective->objtype = 3;
  objective->prev_objtype = 3;
  objective->prev_nvar = 0;
  objective->prev_hasLinear = false;
  objective->gammaScalar = 0.0;
  solution->iterations = 0;
  runTimeOptions.RemainFeasible = true;
  *b_workingset = *workingset;
  solution_0 = *solution;
  longitudi_PresolveWorkingSet_b4(&solution_0, memspace_workspace_float,
    memspace_workspace_int, memspace_workspace_sort, b_workingset, solution,
    b_memspace, qrmanager);
  e_options.NonFiniteSupport = true;
  e_options.IterDisplaySQP = false;
  e_options.InitDamping = 0.01;
  for (i = 0; i < 7; i++) {
    e_options.FiniteDifferenceType[i] = t2_FiniteDifferenceType[i];
  }

  e_options.SpecifyObjectiveGradient = false;
  e_options.ScaleProblem = false;
  e_options.SpecifyConstraintGradient = false;
  e_options.FiniteDifferenceStepSize = -1.0;
  e_options.MaxFunctionEvaluations = -1.0;
  e_options.IterDisplayQP = false;
  e_options.PricingTolerance = 0.0;
  for (i = 0; i < 10; i++) {
    e_options.Algorithm[i] = t2_Algorithm[i];
  }

  e_options.ObjectiveLimit = -1.0E+20;
  e_options.ConstraintTolerance = 1.0E-5;
  e_options.OptimalityTolerance = 1.0E-6;
  e_options.StepTolerance = 1.0E-8;
  e_options.MaxIterations = 2000.0;
  e_options.FunctionTolerance = (rtInf);
  for (i = 0; i < 8; i++) {
    e_options.SolverName[i] = t2_SolverName[i];
  }

  e_options.CheckGradients = false;
  e_options.DiffMaxChange = (rtInf);
  e_options.DiffMinChange = 0.0;
  e_options.Diagnostics[0] = 'o';
  e_options.Display[0] = 'o';
  e_options.FunValCheck[0] = 'o';
  e_options.Diagnostics[1] = 'f';
  e_options.Display[1] = 'f';
  e_options.FunValCheck[1] = 'f';
  e_options.Diagnostics[2] = 'f';
  e_options.Display[2] = 'f';
  e_options.FunValCheck[2] = 'f';
  e_options.UseParallel = false;
  e_options.LinearSolver[0] = 'a';
  e_options.LinearSolver[1] = 'u';
  e_options.LinearSolver[2] = 't';
  e_options.LinearSolver[3] = 'o';
  e_options.SubproblemAlgorithm[0] = 'c';
  e_options.SubproblemAlgorithm[1] = 'g';
  if (solution->state >= 0) {
    solution->iterations = 0;
    longitudinal_mpc_B.b_workingset = *b_workingset;
    longi_maxConstraintViolation_b4(&longitudinal_mpc_B.b_workingset,
      solution->xstar, &b, b_workingset);
    solution->maxConstr = b;
    tmp = 1.0E-5 * runTimeOptions.ConstrRelTolFactor;
    guard1 = false;
    if (b > tmp) {
      longitudinal_mpc_phaseone_b4(H, f, solution, b_memspace, b_workingset,
        qrmanager, cholmanager, &runTimeOptions, objective, &e_options);
      if (solution->state == 0) {
      } else {
        longitudinal_mpc_B.b_workingset = *b_workingset;
        longi_maxConstraintViolation_b4(&longitudinal_mpc_B.b_workingset,
          solution->xstar, &b, b_workingset);
        solution->maxConstr = b;
        if (b > tmp) {
          std::memset(&solution->lambda[0], 0, 333U * sizeof(real_T));
          solution->fstar = longitudinal_mpc_computeFval(objective,
            b_memspace->workspace_float, H, f, solution->xstar);
          solution->state = -2;
        } else {
          if (b > 0.0) {
            std::memcpy(&solution->searchDir[0], &solution->xstar[0], 36U *
                        sizeof(real_T));
            longitudinal_mpc_B.b_workingset = *b_workingset;
            solution_0 = *solution;
            longitud_PresolveWorkingSet_b4t(&solution_0, b_memspace,
              &longitudinal_mpc_B.b_workingset, qrmanager, solution);
            longi_maxConstraintViolation_b4(&longitudinal_mpc_B.b_workingset,
              solution->xstar, &b, b_workingset);
            if (b >= solution->maxConstr) {
              solution->maxConstr = b;
              std::memcpy(&solution->xstar[0], &solution->searchDir[0], 36U *
                          sizeof(real_T));
            }
          }

          guard1 = true;
        }
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      longitudinal_mpc_iterate_b(H, f, solution, b_memspace, b_workingset,
        qrmanager, cholmanager, objective, e_options.ObjectiveLimit,
        e_options.StepTolerance, runTimeOptions.ConstrRelTolFactor,
        runTimeOptions.ProbRelTolFactor, true);
      if (longitudinal_mpc_strcmp(e_options.SolverName) && (solution->state !=
           -6)) {
        longitudinal_mpc_B.b_workingset = *b_workingset;
        longi_maxConstraintViolation_b4(&longitudinal_mpc_B.b_workingset,
          solution->xstar, &solution->maxConstr, b_workingset);
        longitudin_computeFirstOrderOpt(solution, objective, b_workingset,
          b_memspace->workspace_float);
        runTimeOptions.RemainFeasible = false;
        while ((solution->iterations < 2000) && ((solution->state == -7) ||
                ((solution->state == 1) && ((solution->maxConstr > tmp) ||
                  (solution->firstorderopt > 1.0E-6 *
                   runTimeOptions.ProbRelTolFactor))))) {
          longitu_feasibleX0ForWorkingSet(b_memspace->workspace_float,
            solution->xstar, b_workingset, qrmanager);
          solution_0 = *solution;
          longitud_PresolveWorkingSet_b4t(&solution_0, b_memspace, b_workingset,
            qrmanager, solution);
          longitudinal_mpc_phaseone_b4t(H, f, solution, b_memspace, b_workingset,
            qrmanager, cholmanager, objective, &e_options, &runTimeOptions,
            &longitudinal_mpc_B.b_workingset);
          longitudinal_mpc_iterate_b(H, f, solution, b_memspace,
            &longitudinal_mpc_B.b_workingset, qrmanager, cholmanager, objective,
            e_options.ObjectiveLimit, e_options.StepTolerance,
            runTimeOptions.ConstrRelTolFactor, runTimeOptions.ProbRelTolFactor,
            false);
          longi_maxConstraintViolation_b4(&longitudinal_mpc_B.b_workingset,
            solution->xstar, &solution->maxConstr, b_workingset);
          longitudin_computeFirstOrderOpt(solution, objective, b_workingset,
            b_memspace->workspace_float);
        }
      }
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_linearForm__b(boolean_T obj_hasLinear,
  int32_T obj_nvar, real_T workspace[37], const real_T H[1296], const real_T f
  [36], const real_T x[37])
{
  int32_T fMultiplier;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  fMultiplier = 0;
  if (obj_hasLinear) {
    if (static_cast<uint8_T>(obj_nvar) - 1 >= 0) {
      std::memcpy(&workspace[0], &f[0], static_cast<uint32_T>
                  ((static_cast<uint8_T>(obj_nvar) - 1) + 1) * sizeof(real_T));
    }

    fMultiplier = 1;
  }

  if (obj_nvar != 0) {
    int32_T d;
    int32_T ix;
    if (fMultiplier != 1) {
      std::memset(&workspace[0], 0, static_cast<uint32_T>((static_cast<uint8_T>
        (obj_nvar) - 1) + 1) * sizeof(real_T));
    }

    ix = 0;
    d = (obj_nvar - 1) * obj_nvar;
    for (int32_T b_i{1}; obj_nvar < 0 ? b_i >= d + 1 : b_i <= d + 1; b_i +=
         obj_nvar) {
      int32_T e;
      e = b_i + obj_nvar;
      for (fMultiplier = b_i; fMultiplier < e; fMultiplier++) {
        int32_T tmp;
        tmp = fMultiplier - b_i;
        workspace[tmp] += H[fMultiplier - 1] * x[ix];
      }

      ix++;
    }
  }

  /* End of Start for MATLABSystem: '<S1>/MPC System' */
}

void longitudinal_mpc::longitudinal_mpc_quadprog(const real_T H[1296], const
  real_T f[36], const real_T Aineq[11952], const real_T bineq[332], const real_T
  x0[36], real_T x[36], real_T *fval, real_T *exitflag, char_T output_algorithm
  [10], real_T *output_firstorderopt, real_T *output_constrviolation, real_T
  *output_iterations, sCwhQn7ZFnEO6qmUyij3F5_longit_T *lambda)
{
  __m128d tmp;
  sJvHSAPlL1SbbU0gnSE72ZG_longi_T solution;
  sL9bDKomAYkxZSVrG9w6En_longit_T expl_temp;
  scJprC0tZnwNUG3KoXsnHVD_longi_T QPObjective;
  real_T H_infnrm;
  real_T colSum;
  real_T f_infnrm;
  real_T tol;
  int32_T memspace_workspace_int[333];
  int32_T colPos;
  int32_T i;
  int32_T idxFillStart;
  int32_T vectorUB;
  static const int16_T WorkingSet_tmp[5]{ 0, 0, 332, 0, 0 };

  static const int16_T tmp_0[5]{ 0, 0, 332, 1, 0 };

  static const int16_T tmp_1[6]{ 1, 0, 0, 332, 0, 0 };

  static const int16_T tmp_2[5]{ 0, 0, 332, 332, 0 };

  static const int16_T tmp_3[5]{ 0, 0, 332, 333, 0 };

  static const int16_T tmp_4[6]{ 1, 0, 0, 332, 1, 0 };

  static const int16_T tmp_5[6]{ 1, 0, 0, 332, 332, 0 };

  static const int16_T tmp_6[6]{ 1, 0, 0, 332, 333, 0 };

  static const char_T tmp_7[10]{ 'a', 'c', 't', 'i', 'v', 'e', '-', 's', 'e',
    't' };

  solution.fstar = 0.0;
  solution.firstorderopt = 0.0;
  std::memset(&solution.lambda[0], 0, 333U * sizeof(real_T));
  solution.state = 0;
  solution.maxConstr = 0.0;
  solution.iterations = 0;
  std::memset(&solution.searchDir[0], 0, 37U * sizeof(real_T));

  /* Start for MATLABSystem: '<S1>/MPC System' */
  std::memcpy(&solution.xstar[0], &x0[0], 36U * sizeof(real_T));
  longitudinal_m_factoryConstruct(&longitudinal_mpc_B.CholRegManager);
  longitudinal_mpc_B.CholRegManager.scaleFactor = 100.0;
  longitudinal_mpc_B.b_WorkingSet.nVarOrig = 36;
  longitudinal_mpc_B.b_WorkingSet.nVarMax = 37;
  longitudinal_mpc_B.b_WorkingSet.ldA = 37;
  std::memset(&longitudinal_mpc_B.b_WorkingSet.Aineq[0], 0, 12284U * sizeof
              (real_T));
  std::memset(&longitudinal_mpc_B.b_WorkingSet.bineq[0], 0, 332U * sizeof(real_T));
  std::memset(&longitudinal_mpc_B.b_WorkingSet.lb[0], 0, 37U * sizeof(real_T));
  std::memset(&longitudinal_mpc_B.b_WorkingSet.ub[0], 0, 37U * sizeof(real_T));
  longitudinal_mpc_B.b_WorkingSet.mEqRemoved = 0;
  std::memset(&longitudinal_mpc_B.b_WorkingSet.ATwset[0], 0, 12321U * sizeof
              (real_T));
  std::memset(&longitudinal_mpc_B.b_WorkingSet.bwset[0], 0, 333U * sizeof(real_T));
  std::memset(&longitudinal_mpc_B.b_WorkingSet.maxConstrWorkspace[0], 0, 333U *
              sizeof(real_T));
  std::memset(&longitudinal_mpc_B.b_WorkingSet.isActiveConstr[0], 0, 333U *
              sizeof(boolean_T));
  std::memset(&longitudinal_mpc_B.b_WorkingSet.Wid[0], 0, 333U * sizeof(int32_T));
  std::memset(&longitudinal_mpc_B.b_WorkingSet.Wlocalidx[0], 0, 333U * sizeof
              (int32_T));
  std::memset(&longitudinal_mpc_B.b_WorkingSet.indexLB[0], 0, 37U * sizeof
              (int32_T));
  std::memset(&longitudinal_mpc_B.b_WorkingSet.indexUB[0], 0, 37U * sizeof
              (int32_T));
  std::memset(&longitudinal_mpc_B.b_WorkingSet.indexFixed[0], 0, 37U * sizeof
              (int32_T));
  longitudinal_mpc_B.b_WorkingSet.mConstrOrig = 332;
  longitudinal_mpc_B.b_WorkingSet.mConstrMax = 333;
  for (i = 0; i < 5; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    longitudinal_mpc_B.b_WorkingSet.sizesNormal[i] = WorkingSet_tmp[i];
    longitudinal_mpc_B.b_WorkingSet.sizesPhaseOne[i] = tmp_0[i];
    longitudinal_mpc_B.b_WorkingSet.sizesRegularized[i] = tmp_2[i];
    longitudinal_mpc_B.b_WorkingSet.sizesRegPhaseOne[i] = tmp_3[i];
  }

  for (i = 0; i < 6; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i] = tmp_1[i];
  }

  for (i = 0; i < 5; i++) {
    longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
      longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i];
  }

  for (i = 0; i < 6; i++) {
    longitudinal_mpc_B.b_WorkingSet.isActiveIdxNormal[i] =
      longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i];

    /* Start for MATLABSystem: '<S1>/MPC System' */
    longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i] = tmp_4[i];
  }

  for (i = 0; i < 5; i++) {
    longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
      longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i];
  }

  for (i = 0; i < 6; i++) {
    longitudinal_mpc_B.b_WorkingSet.isActiveIdxPhaseOne[i] =
      longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i];

    /* Start for MATLABSystem: '<S1>/MPC System' */
    longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i] = tmp_5[i];
  }

  for (i = 0; i < 5; i++) {
    longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
      longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i];
  }

  for (i = 0; i < 6; i++) {
    longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegularized[i] =
      longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i];

    /* Start for MATLABSystem: '<S1>/MPC System' */
    longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i] = tmp_6[i];
  }

  for (i = 0; i < 5; i++) {
    longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i + 1] +=
      longitudinal_mpc_B.b_WorkingSet.isActiveIdxRegPhaseOne[i];
  }

  for (i = 0; i < 332; i++) {
    for (idxFillStart = 0; idxFillStart < 36; idxFillStart++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      longitudinal_mpc_B.b_WorkingSet.Aineq[idxFillStart + 37 * i] = Aineq[332 *
        idxFillStart + i];
    }

    /* Start for MATLABSystem: '<S1>/MPC System' */
    longitudinal_mpc_B.b_WorkingSet.bineq[i] = bineq[i];
  }

  longitudinal_mpc_B.b_WorkingSet.nVar = 36;
  longitudinal_mpc_B.b_WorkingSet.mConstr = 332;
  for (i = 0; i < 5; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    longitudinal_mpc_B.b_WorkingSet.sizes[i] = WorkingSet_tmp[i];
  }

  for (colPos = 0; colPos < 6; colPos++) {
    longitudinal_mpc_B.b_WorkingSet.isActiveIdx[colPos] =
      longitudinal_mpc_B.b_WorkingSet.isActiveIdxNormal[colPos];
  }

  longitudinal_mpc_B.b_WorkingSet.probType = 3;
  idxFillStart = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[2];
  for (i = idxFillStart; i < 334; i++) {
    longitudinal_mpc_B.b_WorkingSet.isActiveConstr[i - 1] = false;
  }

  longitudinal_mpc_B.b_WorkingSet.nWConstr[0] = 0;
  longitudinal_mpc_B.b_WorkingSet.nWConstr[1] = 0;
  longitudinal_mpc_B.b_WorkingSet.nWConstr[2] = 0;
  longitudinal_mpc_B.b_WorkingSet.nWConstr[3] = 0;
  longitudinal_mpc_B.b_WorkingSet.nWConstr[4] = 0;
  longitudinal_mpc_B.b_WorkingSet.nActiveConstr = 0;
  longitudinal_mpc_B.b_WorkingSet.SLACK0 = 0.0;
  tol = 1.0;
  for (i = 0; i < 332; i++) {
    colSum = 0.0;

    /* Start for MATLABSystem: '<S1>/MPC System' */
    colPos = 37 * i;
    for (idxFillStart = 0; idxFillStart < 36; idxFillStart++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      colSum += std::abs(longitudinal_mpc_B.b_WorkingSet.Aineq[idxFillStart +
                         colPos]);
    }

    /* Start for MATLABSystem: '<S1>/MPC System' */
    tol = std::fmax(tol, colSum);
  }

  H_infnrm = 0.0;
  f_infnrm = 0.0;
  for (i = 0; i < 36; i++) {
    colSum = 0.0;
    for (idxFillStart = 0; idxFillStart < 36; idxFillStart++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      colSum += std::abs(H[36 * i + idxFillStart]);
    }

    /* Start for MATLABSystem: '<S1>/MPC System' */
    H_infnrm = std::fmax(H_infnrm, colSum);
    f_infnrm = std::fmax(f_infnrm, std::abs(f[i]));
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  expl_temp.RemainFeasible = false;
  expl_temp.ProbRelTolFactor = std::fmax(std::fmax(tol, f_infnrm), H_infnrm);
  expl_temp.ConstrRelTolFactor = tol;
  expl_temp.MaxIterations = 2000;
  std::memcpy(&longitudinal_mpc_B.b_memspace_b[0],
              &longitudinal_mpc_B.b_memspace.workspace_float[0], 12321U * sizeof
              (real_T));
  longitudinal_mpc_B.b_WorkingSet_m = longitudinal_mpc_B.b_WorkingSet;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  longitudinal_mpc_driver(H, f, &solution, longitudinal_mpc_B.b_memspace_b,
    memspace_workspace_int, memspace_workspace_int,
    &longitudinal_mpc_B.b_WorkingSet_m, &longitudinal_mpc_B.CholRegManager,
    expl_temp, &longitudinal_mpc_B.b_memspace, &longitudinal_mpc_B.b_WorkingSet,
    &longitudinal_mpc_B.QRManager, &QPObjective);
  std::memcpy(&x[0], &solution.xstar[0], 36U * sizeof(real_T));
  if (solution.state > 0) {
    *fval = solution.fstar;
  } else {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    *fval = longitudinal_mpc_computeFval(&QPObjective,
      longitudinal_mpc_B.b_memspace.workspace_float, H, f, solution.xstar);
  }

  switch (solution.state) {
   case 2:
    solution.state = -3;
    break;

   case -3:
    solution.state = -2;
    break;

   case 4:
    solution.state = -2;
    break;
  }

  *exitflag = solution.state;
  if (solution.state == -2) {
    solution.firstorderopt = (rtInf);
  } else if (solution.state <= 0) {
    longitudinal_mpc_B.b_WorkingSet_m = longitudinal_mpc_B.b_WorkingSet;
    longi_maxConstraintViolation_b4(&longitudinal_mpc_B.b_WorkingSet_m,
      solution.xstar, &colSum, &longitudinal_mpc_B.b_WorkingSet);
    solution.maxConstr = colSum;

    /* Start for MATLABSystem: '<S1>/MPC System' */
    if (colSum <= 1.0E-5 * tol) {
      switch (QPObjective.objtype) {
       case 5:
        if (QPObjective.nvar - 2 >= 0) {
          std::memset(&QPObjective.grad[0], 0, static_cast<uint32_T>
                      ((QPObjective.nvar - 2) + 1) * sizeof(real_T));
        }

        QPObjective.grad[QPObjective.nvar - 1] = QPObjective.gammaScalar;
        break;

       case 3:
        longitudinal_mpc_linearForm__b(QPObjective.hasLinear, QPObjective.nvar,
          QPObjective.grad, H, f, solution.xstar);
        break;

       default:
        longitudinal_mpc_linearForm__b(QPObjective.hasLinear, QPObjective.nvar,
          QPObjective.grad, H, f, solution.xstar);
        idxFillStart = QPObjective.nvar;
        colPos = ((((36 - QPObjective.nvar) / 2) << 1) + QPObjective.nvar) + 1;
        vectorUB = colPos - 2;
        for (i = idxFillStart + 1; i <= vectorUB; i += 2) {
          tmp = _mm_loadu_pd(&solution.xstar[i - 1]);
          _mm_storeu_pd(&QPObjective.grad[i - 1], _mm_mul_pd(tmp, _mm_set1_pd
            (0.0)));
        }

        for (i = colPos; i < 37; i++) {
          QPObjective.grad[i - 1] = solution.xstar[i - 1] * 0.0;
        }
        break;
      }

      longitudin_computeFirstOrderOpt(&solution, &QPObjective,
        &longitudinal_mpc_B.b_WorkingSet,
        longitudinal_mpc_B.b_memspace.workspace_float);
    } else {
      solution.firstorderopt = (rtInf);
    }
  }

  for (colPos = 0; colPos < 10; colPos++) {
    output_algorithm[colPos] = tmp_7[colPos];
  }

  *output_firstorderopt = solution.firstorderopt;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  *output_constrviolation = std::fmax(0.0, solution.maxConstr);
  *output_iterations = solution.iterations;
  std::memset(&lambda->ineqlin[0], 0, 332U * sizeof(real_T));
  std::memset(&lambda->lower[0], 0, 36U * sizeof(real_T));
  std::memset(&lambda->upper[0], 0, 36U * sizeof(real_T));
  if (longitudinal_mpc_B.b_WorkingSet.nActiveConstr > 0) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    idxFillStart = static_cast<uint16_T>(longitudinal_mpc_B.b_WorkingSet.sizes[3]
      + 332) - 1;
    for (i = 0; i <= idxFillStart; i++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      longitudinal_mpc_B.b_memspace.workspace_float[i] = solution.lambda[i];
      solution.lambda[i] = 0.0;
    }

    idxFillStart = 0;

    /* Start for MATLABSystem: '<S1>/MPC System' */
    i = 0;
    while ((i + 1 <= longitudinal_mpc_B.b_WorkingSet.nActiveConstr) &&
           (longitudinal_mpc_B.b_WorkingSet.Wid[i] <= 2)) {
      if (longitudinal_mpc_B.b_WorkingSet.Wid[i] == 1) {
        colPos = -1;
      } else {
        colPos = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[1] - 2;
      }

      solution.lambda[colPos + longitudinal_mpc_B.b_WorkingSet.Wlocalidx[i]] =
        longitudinal_mpc_B.b_memspace.workspace_float[idxFillStart];
      idxFillStart++;
      i++;
    }

    while (i + 1 <= longitudinal_mpc_B.b_WorkingSet.nActiveConstr) {
      switch (longitudinal_mpc_B.b_WorkingSet.Wid[i]) {
       case 3:
        colPos = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[2];
        break;

       case 4:
        colPos = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[3];
        break;

       default:
        colPos = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[4];
        break;
      }

      solution.lambda[(colPos + longitudinal_mpc_B.b_WorkingSet.Wlocalidx[i]) -
        2] = longitudinal_mpc_B.b_memspace.workspace_float[idxFillStart];
      idxFillStart++;
      i++;
    }

    std::memset(&lambda->ineqlin[0], 0, 332U * sizeof(real_T));
    std::memset(&lambda->lower[0], 0, 36U * sizeof(real_T));
    std::memset(&lambda->upper[0], 0, 36U * sizeof(real_T));
    idxFillStart = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[1];
    for (i = 1; i < idxFillStart; i++) {
      /* Check node always fails. would cause program termination and was eliminated */
    }

    /* Start for MATLABSystem: '<S1>/MPC System' */
    i = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[1];
    idxFillStart = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[2];
    for (colPos = i; colPos < idxFillStart; colPos++) {
      /* Check node always fails. would cause program termination and was eliminated */
    }

    /* Start for MATLABSystem: '<S1>/MPC System' */
    i = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[2];
    idxFillStart = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[3];
    if (i <= idxFillStart - 1) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      std::memcpy(&lambda->ineqlin[0], &solution.lambda[i + -1],
                  static_cast<uint32_T>(idxFillStart - i) * sizeof(real_T));
    }

    /* Start for MATLABSystem: '<S1>/MPC System' */
    i = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[3];
    idxFillStart = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[4];
    for (colPos = i; colPos < idxFillStart; colPos++) {
      /* Start for MATLABSystem: '<S1>/MPC System' */
      lambda->lower[longitudinal_mpc_B.b_WorkingSet.indexLB[colPos - i] - 1] =
        solution.lambda[colPos - 1];
    }

    /* Start for MATLABSystem: '<S1>/MPC System' */
    i = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[4];
    idxFillStart = longitudinal_mpc_B.b_WorkingSet.isActiveIdx[5];
    for (colPos = i; colPos < idxFillStart; colPos++) {
      /* Check node always fails. would cause program termination and was eliminated */
    }
  }
}

boolean_T longitudinal_mpc::longitudinal_mpc_isMember(real_T a)
{
  int32_T ihi;
  int32_T ilo;
  int32_T n;
  static const int16_T s[14]{ -1235, -1234, -6, -3, -2, -1, 0, 1, 11, 12, 13, 14,
    15, 17 };

  boolean_T exitg1;
  n = 0;
  ilo = 1;
  ihi = 14;
  exitg1 = false;
  while ((!exitg1) && (ihi >= ilo)) {
    real_T tmp;
    int32_T imid;
    imid = ((ilo >> 1) + (ihi >> 1)) - 1;
    if (((ilo & 1) == 1) && ((ihi & 1) == 1)) {
      imid++;
    }

    tmp = s[imid];
    if (a == tmp) {
      n = imid + 1;
      exitg1 = true;
    } else {
      boolean_T p;
      if (std::isnan(a)) {
        p = false;
      } else {
        p = (a < tmp);
      }

      if (p) {
        ihi = imid;
      } else {
        ilo = imid + 2;
      }
    }
  }

  if (n > 0) {
    while ((n - 1 > 0) && (s[n - 2] == a)) {
      n--;
    }
  }

  return n > 0;
}

flags longitudinal_mpc::longitudi_convert_to_enum_flags(int32_T input)
{
  flags output;

  /* Initialize output value to default value for flags (SOLVED) */
  output = flags::SOLVED;
  if (((input >= -1235) && (input <= -1234)) || (input == -6) || ((input >= -3) &&
       (input <= 1)) || ((input >= 11) && (input <= 15)) || (input == 17)) {
    /* Set output value to input value if it is a member of flags */
    output = static_cast<flags>(input);
  }

  return output;
}

/* Model step function */
void longitudinal_mpc::step()
{
  __m128d tmp_1;
  __m128d tmp_2;
  sCwhQn7ZFnEO6qmUyij3F5_longit_T lambda;
  real_T rhs[332];
  real_T s_con_vary[328];
  real_T K[320];
  real_T K_1[320];
  real_T rtb_output_ref[243];
  real_T rtb_output_ref_0[243];
  real_T a_data[201];
  real_T pos_pred[201];
  real_T time_pred[201];
  real_T vel_pred[201];
  real_T b_y1_data[200];
  real_T Yr[96];
  real_T tmp[96];
  real_T Sr[81];
  real_T Vr[81];
  real_T rtb_control_ref[81];
  real_T rtb_s_con[81];
  real_T rtb_t_con[81];
  real_T rtb_time_ref[81];
  real_T rtb_v_con[81];
  real_T x[81];
  real_T b_x[36];
  real_T tmp_0[36];
  real_T Vq_0[33];
  real_T xT_expected[33];
  real_T T_expected[32];
  real_T Vq[32];
  real_T K_0[8];
  real_T K_2[8];
  real_T expl_temp_0;
  real_T tmp1;
  real_T tmp1_tmp;
  real_T tmp2;
  int32_T dimSize;
  int32_T i;
  int32_T idx;
  int32_T iyLead;
  int32_T partialTrueCount;
  int32_T partialTrueCount_tmp;
  int32_T tmp1_tmp_tmp;
  char_T expl_temp[10];
  uint8_T e_data[201];
  uint8_T tmp_data[201];
  uint8_T tmp_data_0[201];
  boolean_T tmp_data_1[328];
  boolean_T valid_inds[201];
  flags b_this;
  static const real_T g[81]{ 0.2, 0.4, 0.60000000000000009, 0.8, 1.0,
    1.2000000000000002, 1.4000000000000001, 1.6, 1.8, 2.0, 2.2,
    2.4000000000000004, 2.6, 2.8000000000000003, 3.0, 3.2, 3.4000000000000004,
    3.6, 3.8000000000000003, 4.0, 4.2, 4.4, 4.6000000000000005,
    4.8000000000000007, 5.0, 5.2, 5.4, 5.6000000000000005, 5.8000000000000007,
    6.0, 6.2, 6.4, 6.6000000000000005, 6.8000000000000007, 7.0, 7.2, 7.4,
    7.6000000000000005, 7.8000000000000007, 8.0, 8.2000000000000011, 8.4, 8.6,
    8.8, 9.0, 9.2000000000000011, 9.4, 9.6000000000000014, 9.8, 10.0,
    10.200000000000001, 10.4, 10.600000000000001, 10.8, 11.0, 11.200000000000001,
    11.4, 11.600000000000001, 11.8, 12.0, 12.200000000000001, 12.4,
    12.600000000000001, 12.8, 13.0, 13.200000000000001, 13.4, 13.600000000000001,
    13.8, 14.0, 14.200000000000001, 14.4, 14.600000000000001, 14.8, 15.0,
    15.200000000000001, 15.4, 15.600000000000001, 15.8, 16.0, 16.2 };

  static const boolean_T tmp_3[10]{ false, false, false, false, false, false,
    false, false, true, false };

  static const boolean_T tmp_4[8]{ false, false, false, false, false, false,
    true, false };

  static const int32_T tmp_5[16]{ 10000, 0, 0, 0, 0, 100000, 0, 0, 0, 0, 100000,
    0, 0, 0, 0, 1000000 };

  real_T work_data_idx_0;
  int32_T tmp_size[1];
  int32_T exitg2;
  uint8_T ii_data_idx_0;
  boolean_T exitg1;

  /* Outputs for Atomic SubSystem: '<Root>/longitudinal_mpc' */
  /* MATLAB Function: '<S1>/getReference' incorporates:
   *  Inport: '<Root>/time_pred'
   */
  for (i = 0; i < 201; i++) {
    valid_inds[i] = ((!std::isinf(longitudinal_mpc_U.time_pred[i])) && (!std::
      isnan(longitudinal_mpc_U.time_pred[i])) &&
                     (!(longitudinal_mpc_U.time_pred[i] <= 0.001)));
    time_pred[i] = 0.0;
    pos_pred[i] = 0.0;
    vel_pred[i] = 0.0;
  }

  valid_inds[0] = true;

  /* End of Outputs for SubSystem: '<Root>/longitudinal_mpc' */
  idx = 0;
  for (i = 0; i < 201; i++) {
    /* Outputs for Atomic SubSystem: '<Root>/longitudinal_mpc' */
    /* MATLAB Function: '<S1>/getReference' */
    if (valid_inds[i]) {
      e_data[idx] = static_cast<uint8_T>(i);
      idx++;
    }

    /* End of Outputs for SubSystem: '<Root>/longitudinal_mpc' */
  }

  idx = 0;
  partialTrueCount = 0;
  iyLead = 0;
  partialTrueCount_tmp = 0;
  dimSize = 0;
  for (i = 0; i < 201; i++) {
    /* Outputs for Atomic SubSystem: '<Root>/longitudinal_mpc' */
    /* MATLAB Function: '<S1>/getReference' incorporates:
     *  Inport: '<Root>/pos_pred'
     *  Inport: '<Root>/time_pred'
     *  Inport: '<Root>/vel_pred'
     */
    if (valid_inds[i]) {
      time_pred[e_data[idx]] = longitudinal_mpc_U.time_pred[i];
      dimSize = idx + 1;
      partialTrueCount_tmp = idx + 1;
      idx++;
      pos_pred[e_data[partialTrueCount]] = longitudinal_mpc_U.pos_pred[i];
      partialTrueCount = dimSize;
      vel_pred[e_data[iyLead]] = longitudinal_mpc_U.vel_pred[i];
      iyLead = dimSize;
    }

    /* End of Outputs for SubSystem: '<Root>/longitudinal_mpc' */
  }

  /* Outputs for Atomic SubSystem: '<Root>/longitudinal_mpc' */
  /* MATLAB Function: '<S1>/getReference' incorporates:
   *  Inport: '<Root>/ego_state'
   *  Inport: '<Root>/pos_max'
   *  Inport: '<Root>/t'
   *  Inport: '<Root>/time_pred'
   *  Inport: '<Root>/vel_max'
   */
  if (dimSize == 0) {
    iyLead = 0;
  } else {
    if (partialTrueCount_tmp - 1 <= 1) {
      i = partialTrueCount_tmp - 1;
    } else {
      i = 1;
    }

    if (i < 1) {
      iyLead = 0;
    } else {
      iyLead = static_cast<int16_T>(partialTrueCount_tmp - 1);
      if (static_cast<int16_T>(partialTrueCount_tmp - 1) != 0) {
        idx = 0;
        for (i = 0; i < 201; i++) {
          if (valid_inds[i]) {
            tmp_data[idx] = static_cast<uint8_T>(i);
            idx++;
          }
        }

        work_data_idx_0 = longitudinal_mpc_U.time_pred[tmp_data[0]];
        for (partialTrueCount = 2; partialTrueCount <= partialTrueCount_tmp;
             partialTrueCount++) {
          idx = 0;
          for (i = 0; i < 201; i++) {
            if (valid_inds[i]) {
              tmp_data_0[idx] = static_cast<uint8_T>(i);
              idx++;
            }
          }

          tmp1 = longitudinal_mpc_U.time_pred[tmp_data_0[partialTrueCount - 1]];
          tmp2 = work_data_idx_0;
          work_data_idx_0 = tmp1;
          b_y1_data[partialTrueCount - 2] = tmp1 - tmp2;
        }
      }
    }
  }

  dimSize = 201;
  exitg1 = false;
  while ((!exitg1) && (dimSize > 0)) {
    if (valid_inds[dimSize - 1]) {
      ii_data_idx_0 = static_cast<uint8_T>(dimSize);
      exitg1 = true;
    } else {
      dimSize--;
    }
  }

  tmp1 = time_pred[ii_data_idx_0 - 1];
  work_data_idx_0 = b_y1_data[iyLead - 1];
  partialTrueCount_tmp = 201 - ii_data_idx_0;
  for (partialTrueCount = 0; partialTrueCount <= partialTrueCount_tmp;
       partialTrueCount++) {
    time_pred[(ii_data_idx_0 + partialTrueCount) - 1] = work_data_idx_0 *
      static_cast<real_T>(partialTrueCount) + tmp1;
  }

  tmp1 = vel_pred[ii_data_idx_0 - 1];
  i = 202 - ii_data_idx_0;
  for (partialTrueCount = 0; partialTrueCount < i; partialTrueCount++) {
    vel_pred[(ii_data_idx_0 + partialTrueCount) - 1] = tmp1;
  }

  iyLead = 0;
  if (202 - ii_data_idx_0 != 1) {
    iyLead = -1;
    dimSize = 200 - ii_data_idx_0;
  } else {
    dimSize = -1;
  }

  idx = 1;
  for (partialTrueCount = 0; partialTrueCount <= iyLead; partialTrueCount++) {
    idx *= 202 - ii_data_idx_0;
  }

  for (partialTrueCount = 0; partialTrueCount < idx; partialTrueCount++) {
    tmp1 = 0.0;
    partialTrueCount_tmp = (ii_data_idx_0 + partialTrueCount) - 2;
    tmp2 = vel_pred[partialTrueCount_tmp + 1];
    a_data[partialTrueCount] = 0.0;
    for (iyLead = 0; iyLead <= dimSize; iyLead++) {
      tmp1_tmp_tmp = (iyLead + 1) * idx;
      tmp1_tmp = vel_pred[(partialTrueCount_tmp + tmp1_tmp_tmp) + 1];
      tmp1 += (tmp1_tmp + tmp2) / 2.0;
      tmp2 = tmp1_tmp;
      a_data[partialTrueCount + tmp1_tmp_tmp] = tmp1;
    }
  }

  tmp1 = pos_pred[ii_data_idx_0 - 1];
  dimSize = ((202 - ii_data_idx_0) / 2) << 1;
  idx = dimSize - 2;
  for (partialTrueCount = 0; partialTrueCount <= idx; partialTrueCount += 2) {
    tmp_1 = _mm_loadu_pd(&a_data[partialTrueCount]);
    _mm_storeu_pd(&pos_pred[(ii_data_idx_0 + partialTrueCount) - 1], _mm_add_pd
                  (_mm_mul_pd(tmp_1, _mm_set1_pd(work_data_idx_0)), _mm_set1_pd
                   (tmp1)));
  }

  for (partialTrueCount = dimSize; partialTrueCount < i; partialTrueCount++) {
    pos_pred[(ii_data_idx_0 + partialTrueCount) - 1] = a_data[partialTrueCount] *
      work_data_idx_0 + tmp1;
  }

  for (i = 0; i < 81; i++) {
    rtb_t_con[i] = 0.2 * static_cast<real_T>(i) + longitudinal_mpc_U.t;
  }

  longitudinal_mpc_interp1_h(time_pred, pos_pred, rtb_t_con, rtb_s_con);
  longitudinal_mpc_interp1_h(time_pred, vel_pred, rtb_t_con, rtb_v_con);
  tmp1 = longitudinal_mpc_U.vel_max * 0.95;
  dimSize = -1;
  for (idx = 0; idx < 81; idx++) {
    tmp2 = longitudinal_mpc_U.t + g[idx];
    rtb_time_ref[idx] = tmp2;
    rtb_control_ref[idx] = 0.0;
    rtb_output_ref[dimSize + 1] = 0.0;
    rtb_output_ref[dimSize + 2] = static_cast<real_T>(tmp2 < (rtInf)) * tmp1;
    rtb_output_ref[dimSize + 3] = 0.0;
    dimSize += 3;
  }

  std::memcpy(&x[0], &rtb_time_ref[0], 81U * sizeof(real_T));
  for (i = 0; i < 81; i++) {
    Vr[i] = rtb_output_ref[3 * i + 1];
  }

  tmp1 = 0.0;
  tmp2 = rtb_output_ref[1];
  Sr[0] = 0.0;
  for (idx = 0; idx < 80; idx++) {
    work_data_idx_0 = x[idx + 1] - x[idx];
    x[idx] = work_data_idx_0;
    tmp1_tmp = rtb_output_ref[(idx + 1) * 3 + 1];
    tmp1 += (tmp2 + tmp1_tmp) / 2.0 * work_data_idx_0;
    tmp2 = tmp1_tmp;
    Sr[idx + 1] = tmp1;
  }

  tmp1 = rtb_output_ref[1] * 0.2 + longitudinal_mpc_U.ego_state[0];
  for (i = 0; i <= 78; i += 2) {
    tmp_1 = _mm_loadu_pd(&Sr[i]);
    _mm_storeu_pd(&Sr[i], _mm_add_pd(_mm_set1_pd(tmp1), tmp_1));
  }

  for (i = 80; i < 81; i++) {
    Sr[i] += tmp1;
  }

  if (pos_pred[200] < Sr[80]) {
    for (idx = 0; idx < 81; idx++) {
      Vr[idx] = std::fmin(rtb_output_ref[3 * idx + 1], rtb_v_con[idx]);
      Sr[idx] = std::fmin(Sr[idx], rtb_s_con[idx]);
    }
  }

  if (longitudinal_mpc_U.pos_max < Sr[80]) {
    Vr[80] = 0.0;
    Sr[80] = longitudinal_mpc_U.pos_max;
  }

  for (i = 0; i < 81; i++) {
    rtb_output_ref[3 * i] = Sr[i];
    rtb_output_ref[3 * i + 1] = Vr[i];
  }

  /* MATLABSystem: '<S1>/MPC System' incorporates:
   *  Inport: '<Root>/pos_max'
   *  Inport: '<Root>/vel_max'
   */
  /*  Set MPC matrices for optimization */
  /*  Check input arguments */
  longitudinal_mpc_DW.obj.pos_max = longitudinal_mpc_U.pos_max;
  longitudinal_mpc_DW.obj.vel_max = longitudinal_mpc_U.vel_max;

  /* %% Calculations that vary on input signals */
  /*  Control model */
  /*  Cost function */
  /*  Standard constraints - rhs */
  longitudinal_mpc_DW.obj.bi[0] = 3.0;
  longitudinal_mpc_DW.obj.bi[1] = 3.0;
  longitudinal_mpc_DW.obj.bi[2] = 0.0;
  longitudinal_mpc_DW.obj.bi[3] = 0.0;
  longitudinal_mpc_DW.obj.bi[4] = (rtInf);
  longitudinal_mpc_DW.obj.bi[5] = longitudinal_mpc_DW.obj.pos_max - 3.0;
  longitudinal_mpc_DW.obj.bi[6] = longitudinal_mpc_DW.obj.vel_max;
  longitudinal_mpc_DW.obj.bi[7] = (rtInf);
  longitudinal_mpc_DW.obj.bi[8] = 0.0;
  longitudinal_mpc_DW.obj.bi[9] = 0.25;

  /*  RHS bound for Mi xi + Ei ui \leq bi */
  /*  Standard lower and upper bounds on control */
  /*  -ua */
  /*  ua */
  /*  Standard lower and upper bounds on outputs */
  /*  -s */
  /*  -v */
  /*  -a */
  /*  s */
  /*  v */
  /*  a */
  /*  Affine constraints */
  /*  s + v t_con \leq E{s_pv} - d_0 - d_c: time varying parameter */
  /*  -(mv + u) \leq min_braking */
  /*  Integer constraints */
  std::memcpy(&longitudinal_mpc_DW.obj.bn[0], &longitudinal_mpc_DW.obj.bi[2],
              sizeof(real_T) << 3U);
  dimSize = -1;
  for (idx = 0; idx < 32; idx++) {
    std::memcpy(&K[dimSize + 1], &longitudinal_mpc_DW.obj.bi[0], 10U * sizeof
                (real_T));
    dimSize += 10;
  }

  std::memcpy(&longitudinal_mpc_DW.obj.b[0], &K[0], 320U * sizeof(real_T));
  std::memcpy(&longitudinal_mpc_DW.obj.b[320], &longitudinal_mpc_DW.obj.bn[0],
              sizeof(real_T) << 3U);

  /*  Chance constraints */
  dimSize = -1;
  for (idx = 0; idx < 32; idx++) {
    for (iyLead = 0; iyLead <= 8; iyLead += 2) {
      tmp_1 = _mm_mul_pd(_mm_set1_pd(longitudinal_mpc_DW.obj.s_chance[idx]),
                         _mm_set_pd(static_cast<real_T>(tmp_3[iyLead + 1]),
        static_cast<real_T>(tmp_3[iyLead])));
      _mm_storeu_pd(&K[(dimSize + iyLead) + 1], tmp_1);
    }

    dimSize += 10;
  }

  dimSize = -1;
  for (iyLead = 0; iyLead <= 6; iyLead += 2) {
    tmp_1 = _mm_mul_pd(_mm_set1_pd(longitudinal_mpc_DW.obj.s_chance[32]),
                       _mm_set_pd(static_cast<real_T>(tmp_4[iyLead + 1]),
      static_cast<real_T>(tmp_4[iyLead])));
    _mm_storeu_pd(&K_0[iyLead], tmp_1);
  }

  std::memcpy(&longitudinal_mpc_DW.obj.s_chance_vary[0], &K[0], 320U * sizeof
              (real_T));
  std::memcpy(&longitudinal_mpc_DW.obj.s_chance_vary[320], &K_0[0], sizeof
              (real_T) << 3U);
  for (idx = 0; idx < 32; idx++) {
    for (iyLead = 0; iyLead <= 8; iyLead += 2) {
      tmp_1 = _mm_mul_pd(_mm_set1_pd(longitudinal_mpc_DW.obj.v_chance[idx]),
                         _mm_set_pd(static_cast<real_T>(tmp_3[iyLead + 1]),
        static_cast<real_T>(tmp_3[iyLead])));
      _mm_storeu_pd(&K[(dimSize + iyLead) + 1], tmp_1);
    }

    dimSize += 10;
  }

  for (iyLead = 0; iyLead <= 6; iyLead += 2) {
    tmp_1 = _mm_mul_pd(_mm_set1_pd(longitudinal_mpc_DW.obj.v_chance[32]),
                       _mm_set_pd(static_cast<real_T>(tmp_4[iyLead + 1]),
      static_cast<real_T>(tmp_4[iyLead])));
    _mm_storeu_pd(&K_0[iyLead], tmp_1);
  }

  std::memcpy(&longitudinal_mpc_DW.obj.v_chance_vary[0], &K[0], 320U * sizeof
              (real_T));
  std::memcpy(&longitudinal_mpc_DW.obj.v_chance_vary[320], &K_0[0], sizeof
              (real_T) << 3U);

  /* %% Error check the setup */
  /*  Cost function */
  /*  Standard linear constraints */
  /*  Softened constraints */
  /*  Chance constraints */
  /*  Binary constraints */
  /*  Decision variables */
  /* %% Report */
  /*  Return */
  /* %% Check inputs */
  /*  Cost function */
  /*  Inputs */
  /*  Time vector */
  /*  Constraints */
  /* %% Report */
  /*  Follows the syntax of the assert statement but just prints the msg instead of erroring as well */
  if (!(rtb_time_ref[80] >= 16.0)) {
    std::printf("%s\n",
                "Error: reference time vector and MPC dimensions different sizes or time vector too short.");
    std::fflush(stdout);
  }

  /*  Follows the syntax of the assert statement but just prints the msg instead of erroring as well */
  if (!(rtb_t_con[80] >= 16.0)) {
    std::printf("%s\n",
                "Error: time-varying constraint time vector and MPC dimensions different sizes or constraint vector too short.");
    std::fflush(stdout);
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Follows the syntax of the assert statement but just prints the msg instead of erroring as well */
  tmp_size[0] = 243;
  for (i = 0; i < 243; i++) {
    tmp1_tmp = rtb_output_ref[i];
    tmp_data_1[i] = (std::isinf(tmp1_tmp) || std::isnan(tmp1_tmp));
  }

  /* MATLABSystem: '<S1>/MPC System' */
  if (longitudinal_mpc_vectorAny(tmp_data_1, tmp_size)) {
    std::printf("%s\n", "Error: non-finite output reference variable detected!");
    std::fflush(stdout);
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Follows the syntax of the assert statement but just prints the msg instead of erroring as well */
  tmp_size[0] = 81;
  std::memset(&tmp_data_1[0], 0, 81U * sizeof(boolean_T));

  /* MATLABSystem: '<S1>/MPC System' */
  if (longitudinal_mpc_vectorAny(tmp_data_1, tmp_size)) {
    std::printf("%s\n", "Error: non-finite input reference variable detected!");
    std::fflush(stdout);
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Follows the syntax of the assert statement but just prints the msg instead of erroring as well */
  tmp_size[0] = 328;
  for (i = 0; i < 328; i++) {
    tmp_data_1[i] = std::isnan(longitudinal_mpc_DW.obj.b[i]);
  }

  /* MATLABSystem: '<S1>/MPC System' */
  if (longitudinal_mpc_vectorAny(tmp_data_1, tmp_size)) {
    std::printf("%s\n", "Error: non-finite rhs constraint variable detected!");
    std::fflush(stdout);
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Follows the syntax of the assert statement but just prints the msg instead of erroring as well */
  tmp_size[0] = 81;
  for (i = 0; i < 81; i++) {
    tmp1 = rtb_s_con[i];
    tmp_data_1[i] = (std::isinf(tmp1) || std::isnan(tmp1));
  }

  /* MATLABSystem: '<S1>/MPC System' */
  if (longitudinal_mpc_vectorAny(tmp_data_1, tmp_size)) {
    std::printf("%s\n", "Error: non-finite prediction variable detected!");
    std::fflush(stdout);
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Follows the syntax of the assert statement but just prints the msg instead of erroring as well */
  tmp_size[0] = 81;
  for (i = 0; i < 81; i++) {
    tmp1 = rtb_t_con[i];
    tmp_data_1[i] = (std::isinf(tmp1) || std::isnan(tmp1));
  }

  /* MATLABSystem: '<S1>/MPC System' */
  if (longitudinal_mpc_vectorAny(tmp_data_1, tmp_size)) {
    std::printf("%s\n", "Error: non-finite prediction time variable detected!");
    std::fflush(stdout);
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Return */
  /* %% Process reference trajectory */
  /*  Check if we need to down/up sample the reference trajectory */
  tmp2 = rtb_time_ref[0];

  /* MATLABSystem: '<S1>/MPC System' */
  for (i = 0; i < 32; i++) {
    T_expected[i] = (0.5 * static_cast<real_T>(i) + 0.5) + tmp2;
  }

  std::memcpy(&x[0], &rtb_time_ref[0], 81U * sizeof(real_T));
  std::memset(&Vq[0], 0, sizeof(real_T) << 5U);
  iyLead = 0;
  do {
    exitg2 = 0;
    if (iyLead < 81) {
      if (std::isnan(rtb_time_ref[iyLead])) {
        exitg2 = 1;
      } else {
        iyLead++;
      }
    } else {
      if (rtb_time_ref[1] < rtb_time_ref[0]) {
        for (idx = 0; idx < 40; idx++) {
          tmp1 = x[idx];
          x[idx] = x[80 - idx];
          x[80 - idx] = tmp1;
          rtb_control_ref[idx] = 0.0;
          rtb_control_ref[80 - idx] = 0.0;
        }
      }

      longitudinal_mpc_pchip(x, rtb_control_ref, Vr, K);
      for (iyLead = 0; iyLead < 32; iyLead++) {
        Vq[iyLead] = 0.0;
        tmp1 = T_expected[iyLead];
        if (std::isnan(tmp1)) {
          Vq[iyLead] = (rtNaN);
        } else {
          Vq[iyLead] = longitudinal_mpc_ppval(Vr, K, tmp1);
        }
      }

      exitg2 = 1;
    }
  } while (exitg2 == 0);

  for (i = 0; i < 81; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    rtb_output_ref_0[i] = rtb_output_ref[3 * i];
    rtb_output_ref_0[i + 81] = rtb_output_ref[3 * i + 1];
    rtb_output_ref_0[i + 162] = rtb_output_ref[3 * i + 2];
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  longitudinal_mpc_interp1_b(rtb_time_ref, rtb_output_ref_0, T_expected, tmp);

  /* MATLABSystem: '<S1>/MPC System' */
  for (i = 0; i < 32; i++) {
    Yr[3 * i] = tmp[i];
    Yr[3 * i + 1] = tmp[i + 32];
    Yr[3 * i + 2] = tmp[i + 64];
  }

  /*  Modify with standstill gap margin */
  for (i = 0; i < 81; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' */
    rtb_output_ref_0[i] = rtb_output_ref[3 * i];
    rtb_output_ref_0[i + 81] = rtb_output_ref[3 * i + 1];
    rtb_output_ref_0[i + 162] = rtb_output_ref[3 * i + 2];
  }

  /* Start for MATLABSystem: '<S1>/MPC System' */
  longitudinal_mpc_interp1_b(rtb_time_ref, rtb_output_ref_0, T_expected, tmp);
  for (i = 0; i < 32; i++) {
    Yr[3 * i] = tmp[i] - 3.0;
  }

  /* %% Process time varying constraint */
  /*  Check if we need to down/up sample the time varying constraint */
  tmp1 = rtb_t_con[0];

  /* MATLABSystem: '<S1>/MPC System' */
  for (i = 0; i < 33; i++) {
    xT_expected[i] = 0.5 * static_cast<real_T>(i) + tmp1;
  }

  std::memcpy(&rtb_control_ref[0], &rtb_s_con[0], 81U * sizeof(real_T));
  std::memcpy(&x[0], &rtb_t_con[0], 81U * sizeof(real_T));
  std::memset(&Vq_0[0], 0, 33U * sizeof(real_T));
  iyLead = 0;
  do {
    exitg2 = 0;
    if (iyLead < 81) {
      if (std::isnan(rtb_t_con[iyLead])) {
        exitg2 = 1;
      } else {
        iyLead++;
      }
    } else {
      if (rtb_t_con[1] < rtb_t_con[0]) {
        for (idx = 0; idx < 40; idx++) {
          tmp1 = x[idx];
          x[idx] = x[80 - idx];
          x[80 - idx] = tmp1;
          tmp1 = rtb_control_ref[idx];
          rtb_control_ref[idx] = rtb_control_ref[80 - idx];
          rtb_control_ref[80 - idx] = tmp1;
        }
      }

      longitudinal_mpc_pchip(x, rtb_control_ref, Vr, K);
      for (iyLead = 0; iyLead < 33; iyLead++) {
        Vq_0[iyLead] = 0.0;
        tmp1 = xT_expected[iyLead];
        if (std::isnan(tmp1)) {
          Vq_0[iyLead] = (rtNaN);
        } else {
          Vq_0[iyLead] = longitudinal_mpc_ppval(Vr, K, tmp1);
        }
      }

      exitg2 = 1;
    }
  } while (exitg2 == 0);

  /*  Store reference trajectory terminal output */
  longitudinal_mpc_DW.obj.Ref[0] = rtb_output_ref[240];
  longitudinal_mpc_DW.obj.Ref[1] = rtb_output_ref[241];
  longitudinal_mpc_DW.obj.Ref[2] = rtb_output_ref[242];

  /*  Build time varying constraint that is added to b later */
  dimSize = -1;
  for (idx = 0; idx < 32; idx++) {
    for (iyLead = 0; iyLead <= 8; iyLead += 2) {
      _mm_storeu_pd(&K[(dimSize + iyLead) + 1], _mm_mul_pd(_mm_set1_pd(Vq_0[idx]),
        _mm_set_pd(static_cast<real_T>(tmp_3[iyLead + 1]), static_cast<real_T>
                   (tmp_3[iyLead]))));
    }

    dimSize += 10;
  }

  dimSize = -1;
  for (iyLead = 0; iyLead <= 6; iyLead += 2) {
    _mm_storeu_pd(&K_0[iyLead], _mm_mul_pd(_mm_set1_pd(Vq_0[32]), _mm_set_pd(
      static_cast<real_T>(tmp_4[iyLead + 1]), static_cast<real_T>(tmp_4[iyLead]))));
  }

  for (idx = 0; idx < 32; idx++) {
    for (iyLead = 0; iyLead <= 8; iyLead += 2) {
      _mm_storeu_pd(&K_1[(dimSize + iyLead) + 1], _mm_mul_pd(_mm_set1_pd
        (rtb_v_con[idx]), _mm_set_pd(static_cast<real_T>(tmp_3[iyLead + 1]),
        static_cast<real_T>(tmp_3[iyLead]))));
    }

    dimSize += 10;
  }

  for (iyLead = 0; iyLead <= 6; iyLead += 2) {
    _mm_storeu_pd(&K_2[iyLead], _mm_mul_pd(_mm_set1_pd(rtb_v_con[32]),
      _mm_set_pd(static_cast<real_T>(tmp_4[iyLead + 1]), static_cast<real_T>
                 (tmp_4[iyLead]))));
  }

  /*  Modify with standstill gap margin and chance constraint risk margin */
  for (i = 0; i <= 318; i += 2) {
    tmp_1 = _mm_sub_pd(_mm_loadu_pd(&K[i]), _mm_mul_pd(_mm_set_pd
      (static_cast<real_T>(longitudinal_mpc_DW.obj.v_chance_vary[i + 1] < K_1[i
      + 1]), static_cast<real_T>(longitudinal_mpc_DW.obj.v_chance_vary[i] <
      K_1[i])), _mm_loadu_pd(&longitudinal_mpc_DW.obj.s_chance_vary[i])));
    _mm_storeu_pd(&s_con_vary[i], tmp_1);
  }

  for (i = 0; i <= 6; i += 2) {
    tmp_1 = _mm_sub_pd(_mm_loadu_pd(&K_0[i]), _mm_mul_pd(_mm_set_pd(static_cast<
      real_T>(longitudinal_mpc_DW.obj.v_chance_vary[i + 321] < K_2[i + 1]),
      static_cast<real_T>(longitudinal_mpc_DW.obj.v_chance_vary[i + 320] < K_2[i])),
      _mm_loadu_pd(&longitudinal_mpc_DW.obj.s_chance_vary[i + 320])));
    _mm_storeu_pd(&s_con_vary[i + 320], tmp_1);
  }

  /*  Marginalized by chance constraint and standstill gap margin - standstill gap margin was included in s_chance_vary */
  /*  Store time varying constraint terminal output */
  longitudinal_mpc_DW.obj.Con = s_con_vary[326];

  /* %% Get OCP matrices */
  /*  Expected disturbance input over the horizon */
  /*  Final matrices */
  std::memset(&longitudinal_mpc_B.H[0], 0, 1296U * sizeof(real_T));
  for (i = 0; i < 32; i++) {
    std::memcpy(&longitudinal_mpc_B.H[i * 36], &longitudinal_mpc_DW.obj.G[i << 5],
                sizeof(real_T) << 5U);
  }

  for (i = 0; i < 4; i++) {
    partialTrueCount = i << 2;
    partialTrueCount_tmp = (i + 32) * 36;
    longitudinal_mpc_B.H[partialTrueCount_tmp + 32] = tmp_5[partialTrueCount];
    longitudinal_mpc_B.H[partialTrueCount_tmp + 33] = tmp_5[partialTrueCount + 1];
    longitudinal_mpc_B.H[partialTrueCount_tmp + 34] = tmp_5[partialTrueCount + 2];
    longitudinal_mpc_B.H[partialTrueCount_tmp + 35] = tmp_5[partialTrueCount + 3];
  }

  for (i = 0; i < 328; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' incorporates:
     *  Inport: '<Root>/ego_state'
     */
    tmp1 = longitudinal_mpc_DW.obj.W[i] * longitudinal_mpc_U.ego_state[0];
    tmp1 += longitudinal_mpc_DW.obj.W[i + 328] * longitudinal_mpc_U.ego_state[1];
    tmp1 += longitudinal_mpc_DW.obj.W[i + 656] * longitudinal_mpc_U.ego_state[2];
    tmp1 = ((longitudinal_mpc_DW.obj.b[i] + s_con_vary[i]) +
            longitudinal_mpc_DW.obj.beta_rhs[i]) - tmp1;
    tmp2 = 0.0;
    for (partialTrueCount = 0; partialTrueCount < 32; partialTrueCount++) {
      work_data_idx_0 = 0.0;
      for (partialTrueCount_tmp = 0; partialTrueCount_tmp < 96;
           partialTrueCount_tmp++) {
        work_data_idx_0 += longitudinal_mpc_DW.obj.M[328 * partialTrueCount_tmp
          + i] * longitudinal_mpc_DW.obj.GammaW[96 * partialTrueCount +
          partialTrueCount_tmp];
      }

      tmp2 += work_data_idx_0 * 0.0;
    }

    /* MATLABSystem: '<S1>/MPC System' */
    rhs[i] = tmp1 - tmp2;
  }

  /* MATLABSystem: '<S1>/MPC System' */
  rhs[328] = longitudinal_mpc_DW.obj.Upsilonb[0];
  rhs[329] = longitudinal_mpc_DW.obj.Upsilonb[1];
  rhs[330] = longitudinal_mpc_DW.obj.Upsilonb[2];
  rhs[331] = longitudinal_mpc_DW.obj.Upsilonb[3];

  /* Start for MATLABSystem: '<S1>/MPC System' */
  /*  Solve and then */
  /*  Xopt = [U, eps, beta] */
  /*  Roll forward solution from previous solve for use in warm-starting solver */
  std::memcpy(&T_expected[0], &longitudinal_mpc_DW.obj.U[0], sizeof(real_T) <<
              5U);

  /* MATLABSystem: '<S1>/MPC System' */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* %% Tyler Ard                %%% */
  /* %% Argonne National Lab     %%% */
  /* %% Vehicle Mobility Systems %%% */
  /* %% tard(at)anl.gov          %%% */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  GETNEXTSTEPPREDICTION Get the prediction horizon for the next MPC */
  /*  stage */
  /*  U is the continuous control variables */
  /*  E is the slack variables */
  /*  B is the integer constraint variables */
  /*  ni is the number of continuous control variables in one stage */
  /*  ns is the number of slack variables in one stage */
  /*  nb is the number of integer constraint variables in one stage */
  /*  Assumes X = [U; E; B] */
  /*  Process */
  /*  Get U as vector */
  /*  Roll forward 1 stage */
  /*  zeros(ni,1); */
  /*  Construct */
  std::memcpy(&longitudinal_mpc_DW.obj.Xk[0], &T_expected[1], 31U * sizeof
              (real_T));
  longitudinal_mpc_DW.obj.Xk[31] = T_expected[31];
  longitudinal_mpc_DW.obj.Xk[32] = longitudinal_mpc_DW.obj.E[0];
  longitudinal_mpc_DW.obj.Xk[33] = longitudinal_mpc_DW.obj.E[1];
  longitudinal_mpc_DW.obj.Xk[34] = longitudinal_mpc_DW.obj.E[2];
  longitudinal_mpc_DW.obj.Xk[35] = longitudinal_mpc_DW.obj.E[3];

  /* End of Outputs for SubSystem: '<Root>/longitudinal_mpc' */
  /*  Check if mixed integer problem or continuous */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /* %% Tyler Ard                %%% */
  /* %% Argonne National Lab     %%% */
  /* %% Vehicle Mobility Systems %%% */
  /* %% tard(at)anl.gov          %%% */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  QUADPROG Run mixed-integer quadratic program through quadprog solver */
  /*  Solve a quadratic program of the form  */
  /*  min_x 1/2 x'Px + f'x */
  /*  s.t.  AU \leq b */
  /*        l \leq x \leq u */
  /*        x \in R */
  /*  Handle input arguments */
  /*  Setup workspace */
  /*  % Must use active-set with code generation */
  /*  % 1e-8 default */
  /*  % '10*(numberOfVariables + numberOfConstraints)' default */
  /*  % -1e20 default */
  /*  % 1e-8 default */
  /*  Additional options */
  /*  Bounds check */
  for (i = 0; i < 332; i++) {
    /* Outputs for Atomic SubSystem: '<Root>/longitudinal_mpc' */
    /* Start for MATLABSystem: '<S1>/MPC System' */
    tmp1 = rhs[i];

    /* MATLABSystem: '<S1>/MPC System' */
    if (tmp1 < -1.0E+12) {
      tmp1 = -1.0E+12;
      rhs[i] = -1.0E+12;
    }

    if (tmp1 > 1.0E+12) {
      rhs[i] = 1.0E+12;
    }

    /* End of Outputs for SubSystem: '<Root>/longitudinal_mpc' */
  }

  /* Outputs for Atomic SubSystem: '<Root>/longitudinal_mpc' */
  /* Start for MATLABSystem: '<S1>/MPC System' incorporates:
   *  Inport: '<Root>/ego_state'
   */
  /* %% Solve problem */
  tmp_0[32] = 1.0;
  tmp_0[33] = 0.0;
  tmp_0[34] = 0.0;
  tmp_0[35] = 1.0;
  for (i = 0; i < 32; i++) {
    tmp1 = longitudinal_mpc_DW.obj.F[i] * longitudinal_mpc_U.ego_state[0];
    tmp1 += longitudinal_mpc_DW.obj.F[i + 32] * longitudinal_mpc_U.ego_state[1];
    tmp1 += longitudinal_mpc_DW.obj.F[i + 64] * longitudinal_mpc_U.ego_state[2];
    tmp2 = 0.0;
    for (partialTrueCount = 0; partialTrueCount < 32; partialTrueCount++) {
      tmp2 += longitudinal_mpc_DW.obj.Fw[(partialTrueCount << 5) + i] * 0.0;
    }

    work_data_idx_0 = 0.0;
    for (partialTrueCount = 0; partialTrueCount < 96; partialTrueCount++) {
      work_data_idx_0 += longitudinal_mpc_DW.obj.Ty[(partialTrueCount << 5) + i]
        * Yr[partialTrueCount];
    }

    tmp1_tmp = 0.0;
    for (partialTrueCount = 0; partialTrueCount < 32; partialTrueCount++) {
      tmp1_tmp += longitudinal_mpc_DW.obj.Tu[(partialTrueCount << 5) + i] *
        Vq[partialTrueCount];
    }

    tmp_0[i] = (tmp1 + tmp2) - (work_data_idx_0 + tmp1_tmp);
    std::memcpy(&longitudinal_mpc_B.dv[i * 332], &longitudinal_mpc_DW.obj.a[i *
                328], 328U * sizeof(real_T));
  }

  for (i = 0; i <= 326; i += 2) {
    tmp_1 = _mm_loadu_pd(&longitudinal_mpc_DW.obj.Upsilon[i]);
    tmp_2 = _mm_set1_pd(-1.0);
    _mm_storeu_pd(&longitudinal_mpc_B.dv[i + 10624], _mm_mul_pd(tmp_1, tmp_2));
    tmp_1 = _mm_loadu_pd(&longitudinal_mpc_DW.obj.Upsilon[i + 328]);
    _mm_storeu_pd(&longitudinal_mpc_B.dv[i + 10956], _mm_mul_pd(tmp_1, tmp_2));
    tmp_1 = _mm_loadu_pd(&longitudinal_mpc_DW.obj.Upsilon[i + 656]);
    _mm_storeu_pd(&longitudinal_mpc_B.dv[i + 11288], _mm_mul_pd(tmp_1, tmp_2));
    tmp_1 = _mm_loadu_pd(&longitudinal_mpc_DW.obj.Upsilon[i + 984]);
    _mm_storeu_pd(&longitudinal_mpc_B.dv[i + 11620], _mm_mul_pd(tmp_1, tmp_2));
  }

  for (i = 0; i < 32; i++) {
    longitudinal_mpc_B.dv[332 * i + 328] = 0.0;
    longitudinal_mpc_B.dv[332 * i + 329] = 0.0;
    longitudinal_mpc_B.dv[332 * i + 330] = 0.0;
    longitudinal_mpc_B.dv[332 * i + 331] = 0.0;
  }

  for (i = 0; i < 4; i++) {
    partialTrueCount = i << 2;
    partialTrueCount_tmp = (i + 32) * 332;
    longitudinal_mpc_B.dv[partialTrueCount_tmp + 328] =
      -longitudinal_mpc_DW.obj.UpsilonI[partialTrueCount];
    longitudinal_mpc_B.dv[partialTrueCount_tmp + 329] =
      -longitudinal_mpc_DW.obj.UpsilonI[partialTrueCount + 1];
    longitudinal_mpc_B.dv[partialTrueCount_tmp + 330] =
      -longitudinal_mpc_DW.obj.UpsilonI[partialTrueCount + 2];
    longitudinal_mpc_B.dv[partialTrueCount_tmp + 331] =
      -longitudinal_mpc_DW.obj.UpsilonI[partialTrueCount + 3];
  }

  /* MATLABSystem: '<S1>/MPC System' */
  longitudinal_mpc_quadprog(longitudinal_mpc_B.H, tmp_0, longitudinal_mpc_B.dv,
    rhs, longitudinal_mpc_DW.obj.Xk, b_x, &tmp1, &tmp2, expl_temp,
    &work_data_idx_0, &tmp1_tmp, &expl_temp_0, &lambda);

  /*  Pack */
  if (longitudinal_mpc_isMember(tmp2)) {
    b_this = longitudi_convert_to_enum_flags(static_cast<int32_T>(tmp2));
  } else {
    b_this = flags::UNKNOWN;
  }

  /*  Set solution */
  std::memcpy(&longitudinal_mpc_DW.obj.Xopt[0], &b_x[0], 36U * sizeof(real_T));
  longitudinal_mpc_DW.obj.J = tmp1;
  longitudinal_mpc_DW.obj.flag = b_this;

  /*  Sanity check */
  /*  Post process */
  /*  Postprocess solution */
  std::memcpy(&longitudinal_mpc_DW.obj.U[0], &longitudinal_mpc_DW.obj.Xopt[0],
              sizeof(real_T) << 5U);
  longitudinal_mpc_DW.obj.u = longitudinal_mpc_DW.obj.U[0];

  /* Start for MATLABSystem: '<S1>/MPC System' */
  std::memcpy(&longitudinal_mpc_B.a[0], &longitudinal_mpc_DW.obj.Gamma[0], 3072U
              * sizeof(real_T));

  /* MATLABSystem: '<S1>/MPC System' */
  /*  State sequence for current k timestep */
  longitudinal_mpc_DW.obj.E[0] = longitudinal_mpc_DW.obj.Xopt[32];
  longitudinal_mpc_DW.obj.E[1] = longitudinal_mpc_DW.obj.Xopt[33];
  longitudinal_mpc_DW.obj.E[2] = longitudinal_mpc_DW.obj.Xopt[34];
  longitudinal_mpc_DW.obj.E[3] = longitudinal_mpc_DW.obj.Xopt[35];

  /*  Return */
  for (i = 0; i < 96; i++) {
    /* Start for MATLABSystem: '<S1>/MPC System' incorporates:
     *  Inport: '<Root>/ego_state'
     */
    tmp1 = longitudinal_mpc_DW.obj.Phi[i] * longitudinal_mpc_U.ego_state[0];
    tmp1 += longitudinal_mpc_DW.obj.Phi[i + 96] * longitudinal_mpc_U.ego_state[1];
    tmp1 += longitudinal_mpc_DW.obj.Phi[i + 192] * longitudinal_mpc_U.ego_state
      [2];
    tmp2 = 0.0;
    for (partialTrueCount = 0; partialTrueCount < 32; partialTrueCount++) {
      tmp2 += longitudinal_mpc_B.a[96 * partialTrueCount + i] *
        longitudinal_mpc_DW.obj.U[partialTrueCount];
    }

    /* MATLABSystem: '<S1>/MPC System' */
    longitudinal_mpc_DW.obj.X[i] = tmp1 + tmp2;
    Yr[i] = longitudinal_mpc_DW.obj.X[i];
  }

  /* MATLABSystem: '<S1>/MPC System' */
  std::memcpy(&T_expected[0], &longitudinal_mpc_DW.obj.U[0], sizeof(real_T) <<
              5U);

  /* Outport: '<Root>/slacks' incorporates:
   *  MATLABSystem: '<S1>/MPC System'
   * */
  longitudinal_mpc_Y.slacks[0] = longitudinal_mpc_DW.obj.E[0];
  longitudinal_mpc_Y.slacks[1] = longitudinal_mpc_DW.obj.E[1];
  longitudinal_mpc_Y.slacks[2] = longitudinal_mpc_DW.obj.E[2];
  longitudinal_mpc_Y.slacks[3] = longitudinal_mpc_DW.obj.E[3];

  /* Outport: '<Root>/reference' incorporates:
   *  MATLABSystem: '<S1>/MPC System'
   * */
  longitudinal_mpc_Y.reference[0] = longitudinal_mpc_DW.obj.Ref[0];
  longitudinal_mpc_Y.reference[1] = longitudinal_mpc_DW.obj.Ref[1];
  longitudinal_mpc_Y.reference[2] = longitudinal_mpc_DW.obj.Ref[2];

  /* Outport: '<Root>/acc_des' incorporates:
   *  MATLABSystem: '<S1>/MPC System'
   * */
  longitudinal_mpc_Y.acc_des = longitudinal_mpc_DW.obj.u;

  /* Outport: '<Root>/constraint' incorporates:
   *  MATLABSystem: '<S1>/MPC System'
   * */
  longitudinal_mpc_Y.constraint = longitudinal_mpc_DW.obj.Con;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  tmp1 = longitudinal_mpc_DW.obj.J;

  /* Outport: '<Root>/cost' incorporates:
   *  MATLABSystem: '<S1>/MPC System'
   * */
  longitudinal_mpc_Y.cost = tmp1;

  /* Start for MATLABSystem: '<S1>/MPC System' */
  b_this = longitudinal_mpc_DW.obj.flag;

  /* Outport: '<Root>/exitflag' incorporates:
   *  MATLABSystem: '<S1>/MPC System'
   * */
  longitudinal_mpc_Y.exitflag = b_this;

  /* MATLABSystem: '<S1>/MPC System' */
  std::memcpy(&longitudinal_mpc_DW.Xopt[0], &longitudinal_mpc_DW.obj.Xopt[0],
              36U * sizeof(real_T));
  std::memcpy(&longitudinal_mpc_DW.Xk[0], &longitudinal_mpc_DW.obj.Xk[0], 36U *
              sizeof(real_T));
  longitudinal_mpc_DW.J = tmp1;
  longitudinal_mpc_DW.flag = b_this;

  /* Outport: '<Root>/state_trajectory' incorporates:
   *  MATLABSystem: '<S1>/MPC System'
   */
  std::memcpy(&longitudinal_mpc_Y.state_trajectory[0], &Yr[0], 96U * sizeof
              (real_T));

  /* Start for MATLABSystem: '<S1>/MPC System' */
  tmp1 = rtb_t_con[0];

  /* End of Outputs for SubSystem: '<Root>/longitudinal_mpc' */
  for (i = 0; i < 32; i++) {
    /* Outputs for Atomic SubSystem: '<Root>/longitudinal_mpc' */
    /* Outport: '<Root>/control_trajectory' incorporates:
     *  MATLABSystem: '<S1>/MPC System'
     */
    longitudinal_mpc_Y.control_trajectory[i] = T_expected[i];

    /* Outport: '<Root>/time_trajectory' incorporates:
     *  MATLABSystem: '<S1>/MPC System'
     * */
    longitudinal_mpc_Y.time_trajectory[i] = 0.5 * static_cast<real_T>(i) + tmp1;

    /* End of Outputs for SubSystem: '<Root>/longitudinal_mpc' */
  }
}

/* Model initialize function */
void longitudinal_mpc::initialize()
{
  /* Registration code */

  /* external outputs */
  longitudinal_mpc_Y.exitflag = flags::SOLVED;

  /* SystemInitialize for Atomic SubSystem: '<Root>/longitudinal_mpc' */
  /* Start for MATLABSystem: '<S1>/MPC System' */
  longitudinal_mpc_solver_solver(&longitudinal_mpc_DW.obj);
  longitudinal_mpc_DW.objisempty = true;
  longitudinal_mpc_DW.obj.isInitialized = 1;

  /*         %% Matlab system functions */
  /*  Initialize MPC parent class */
  longitudinal_mpc_PCCMPC_initMPC(&longitudinal_mpc_DW.obj);

  /*  Initialize solver */
  longitudinal__solver_initSolver(&longitudinal_mpc_DW.obj);

  /* InitializeConditions for MATLABSystem: '<S1>/MPC System' */
  /*  Initialize / reset internal properties */
  longitudinal__solver_initSolver(&longitudinal_mpc_DW.obj);
  std::memcpy(&longitudinal_mpc_DW.Xopt[0], &longitudinal_mpc_DW.obj.Xopt[0],
              36U * sizeof(real_T));
  std::memcpy(&longitudinal_mpc_DW.Xk[0], &longitudinal_mpc_DW.obj.Xk[0], 36U *
              sizeof(real_T));
  longitudinal_mpc_DW.J = longitudinal_mpc_DW.obj.J;
  longitudinal_mpc_DW.flag = longitudinal_mpc_DW.obj.flag;

  /* End of SystemInitialize for SubSystem: '<Root>/longitudinal_mpc' */
}

/* Model terminate function */
void longitudinal_mpc::terminate()
{
  /* Terminate for Atomic SubSystem: '<Root>/longitudinal_mpc' */
  /* Terminate for MATLABSystem: '<S1>/MPC System' */
  std::memcpy(&longitudinal_mpc_DW.Xopt[0], &longitudinal_mpc_DW.obj.Xopt[0],
              36U * sizeof(real_T));
  std::memcpy(&longitudinal_mpc_DW.Xk[0], &longitudinal_mpc_DW.obj.Xk[0], 36U *
              sizeof(real_T));
  longitudinal_mpc_DW.J = longitudinal_mpc_DW.obj.J;
  longitudinal_mpc_DW.flag = longitudinal_mpc_DW.obj.flag;

  /* End of Terminate for SubSystem: '<Root>/longitudinal_mpc' */
}

/* Constructor */
longitudinal_mpc::longitudinal_mpc() :
  longitudinal_mpc_U(),
  longitudinal_mpc_Y(),
  longitudinal_mpc_B(),
  longitudinal_mpc_DW(),
  longitudinal_mpc_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
/* Currently there is no destructor body generated.*/
longitudinal_mpc::~longitudinal_mpc() = default;

/* Real-Time Model get method */
RT_MODEL_longitudinal_mpc_T * longitudinal_mpc::getRTM()
{
  return (&longitudinal_mpc_M);
}
