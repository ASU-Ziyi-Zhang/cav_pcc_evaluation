//
// File: mldivide.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "mldivide.h"
#include "qrsolve.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double A_data[]
//                const int A_size[2]
//                double B_data[]
//                int &B_size
// Return Type  : void
//
namespace coder {
void mldivide(const double A_data[], const int A_size[2], double B_data[],
              int &B_size)
{
  double b_A_data[169];
  double b_B_data[13];
  if ((A_size[0] == 0) || (A_size[1] == 0) || (B_size == 0)) {
    int yk;
    yk = A_size[1];
    B_size = A_size[1];
    if (yk - 1 >= 0) {
      std::memset(&B_data[0], 0,
                  static_cast<unsigned int>(yk) * sizeof(double));
    }
  } else if (A_size[0] == A_size[1]) {
    double smax;
    int LDA_tmp;
    int i1;
    int n;
    int temp_tmp;
    int u0;
    int yk;
    signed char ipiv_data[13];
    u0 = A_size[0];
    n = A_size[1];
    if (u0 <= n) {
      n = u0;
    }
    u0 = B_size;
    if (u0 <= n) {
      n = u0;
    }
    LDA_tmp = A_size[0];
    yk = A_size[0] * A_size[1];
    std::copy(&A_data[0], &A_data[yk], &b_A_data[0]);
    if (n > 0) {
      ipiv_data[0] = 1;
      yk = 1;
      for (int k{2}; k <= n; k++) {
        yk++;
        ipiv_data[k - 1] = static_cast<signed char>(yk);
      }
    }
    if (n >= 1) {
      u0 = n - 1;
      if (u0 > n) {
        u0 = n;
      }
      for (int j{0}; j < u0; j++) {
        int b_tmp;
        int jA;
        int jp1j;
        int mmj_tmp;
        mmj_tmp = n - j;
        b_tmp = j * (LDA_tmp + 1);
        jp1j = b_tmp + 2;
        if (mmj_tmp < 1) {
          yk = -1;
        } else {
          yk = 0;
          if (mmj_tmp > 1) {
            smax = std::abs(b_A_data[b_tmp]);
            for (int k{2}; k <= mmj_tmp; k++) {
              double s;
              s = std::abs(b_A_data[(b_tmp + k) - 1]);
              if (s > smax) {
                yk = k - 1;
                smax = s;
              }
            }
          }
        }
        if (b_A_data[b_tmp + yk] != 0.0) {
          if (yk != 0) {
            jA = j + yk;
            ipiv_data[j] = static_cast<signed char>(jA + 1);
            for (int k{0}; k < n; k++) {
              yk = k * LDA_tmp;
              temp_tmp = j + yk;
              smax = b_A_data[temp_tmp];
              i1 = jA + yk;
              b_A_data[temp_tmp] = b_A_data[i1];
              b_A_data[i1] = smax;
            }
          }
          i1 = b_tmp + mmj_tmp;
          for (temp_tmp = jp1j; temp_tmp <= i1; temp_tmp++) {
            b_A_data[temp_tmp - 1] /= b_A_data[b_tmp];
          }
        }
        yk = b_tmp + LDA_tmp;
        jA = yk;
        for (jp1j = 0; jp1j <= mmj_tmp - 2; jp1j++) {
          smax = b_A_data[yk + jp1j * LDA_tmp];
          if (smax != 0.0) {
            i1 = jA + 2;
            temp_tmp = mmj_tmp + jA;
            for (int k{i1}; k <= temp_tmp; k++) {
              b_A_data[k - 1] += b_A_data[((b_tmp + k) - jA) - 1] * -smax;
            }
          }
          jA += LDA_tmp;
        }
      }
    }
    for (temp_tmp = 0; temp_tmp <= n - 2; temp_tmp++) {
      signed char i;
      i = ipiv_data[temp_tmp];
      if (i != temp_tmp + 1) {
        smax = B_data[temp_tmp];
        B_data[temp_tmp] = B_data[i - 1];
        B_data[i - 1] = smax;
      }
    }
    for (int k{0}; k < n; k++) {
      yk = LDA_tmp * k;
      if (B_data[k] != 0.0) {
        i1 = k + 2;
        for (temp_tmp = i1; temp_tmp <= n; temp_tmp++) {
          B_data[temp_tmp - 1] -= B_data[k] * b_A_data[(temp_tmp + yk) - 1];
        }
      }
    }
    for (int k{n}; k >= 1; k--) {
      yk = LDA_tmp * (k - 1);
      smax = B_data[k - 1];
      if (smax != 0.0) {
        smax /= b_A_data[(k + yk) - 1];
        B_data[k - 1] = smax;
        for (temp_tmp = 0; temp_tmp <= k - 2; temp_tmp++) {
          B_data[temp_tmp] -= B_data[k - 1] * b_A_data[temp_tmp + yk];
        }
      }
    }
  } else {
    int yk;
    yk = B_size - 1;
    if (yk >= 0) {
      std::copy(&B_data[0], &B_data[yk + 1], &b_B_data[0]);
    }
    B_size = internal::qrsolve(A_data, A_size, b_B_data, B_size, B_data);
  }
}

} // namespace coder

//
// File trailer for mldivide.cpp
//
// [EOF]
//
