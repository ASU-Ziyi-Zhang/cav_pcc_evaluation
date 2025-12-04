//
// File: xzlarf.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xzlarf.h"
#include "rt_nonfinite.h"
#include "xgerc.h"
#include <cstring>

// Function Definitions
//
// Arguments    : int m
//                int n
//                int iv0
//                double tau
//                double C_data[]
//                int ic0
//                int ldc
//                double work_data[]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void xzlarf(int m, int n, int iv0, double tau, double C_data[], int ic0,
            int ldc, double work_data[])
{
  int i;
  int ia;
  int lastc;
  int lastv;
  if (tau != 0.0) {
    boolean_T exitg2;
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C_data[i - 2] == 0.0)) {
      lastv--;
      i--;
    }
    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      int exitg1;
      i = ic0 + (lastc - 1) * ldc;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C_data[ia - 1] != 0.0) {
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
    lastc = 0;
  }
  if (lastv > 0) {
    if (lastc != 0) {
      int b_i;
      if (lastc - 1 >= 0) {
        std::memset(&work_data[0], 0,
                    static_cast<unsigned int>(lastc) * sizeof(double));
      }
      i = 0;
      b_i = ic0 + ldc * (lastc - 1);
      for (int iac{ic0}; ldc < 0 ? iac >= b_i : iac <= b_i; iac += ldc) {
        double c;
        int i1;
        c = 0.0;
        i1 = (iac + lastv) - 1;
        for (ia = iac; ia <= i1; ia++) {
          c += C_data[ia - 1] * C_data[((iv0 + ia) - iac) - 1];
        }
        work_data[i] += c;
        i++;
      }
    }
    blas::xgerc(lastv, lastc, -tau, iv0, work_data, C_data, ic0, ldc);
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzlarf.cpp
//
// [EOF]
//
