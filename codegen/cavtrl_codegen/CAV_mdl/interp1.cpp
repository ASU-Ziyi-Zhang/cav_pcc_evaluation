//
// File: interp1.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "interp1.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double varargin_1_data[]
//                const int varargin_1_size[2]
//                const double varargin_2_data[]
//                const int varargin_2_size[2]
//                double varargin_3
// Return Type  : double
//
namespace coder {
double interp1(const double varargin_1_data[], const int varargin_1_size[2],
               const double varargin_2_data[], const int varargin_2_size[2],
               double varargin_3)
{
  double x_data[1000];
  double y_data[1000];
  double Vq;
  int high_i;
  int j2;
  int mid_i;
  j2 = varargin_2_size[1];
  if (j2 - 1 >= 0) {
    std::copy(&varargin_2_data[0], &varargin_2_data[j2], &y_data[0]);
  }
  high_i = varargin_1_size[1];
  if (high_i - 1 >= 0) {
    std::copy(&varargin_1_data[0], &varargin_1_data[high_i], &x_data[0]);
  }
  mid_i = varargin_1_size[1] - 1;
  j2 = 0;
  int exitg1;
  do {
    exitg1 = 0;
    if (j2 <= mid_i) {
      if (std::isnan(varargin_1_data[j2])) {
        exitg1 = 1;
      } else {
        j2++;
      }
    } else {
      double xtmp;
      int low_ip1;
      if (varargin_1_data[1] < varargin_1_data[0]) {
        low_ip1 = (mid_i + 1) >> 1;
        for (int b_j1{0}; b_j1 < low_ip1; b_j1++) {
          xtmp = x_data[b_j1];
          j2 = mid_i - b_j1;
          x_data[b_j1] = x_data[j2];
          x_data[j2] = xtmp;
        }
        low_ip1 = varargin_2_size[1] >> 1;
        for (int b_j1{0}; b_j1 < low_ip1; b_j1++) {
          j2 = (varargin_2_size[1] - b_j1) - 1;
          xtmp = y_data[b_j1];
          y_data[b_j1] = y_data[j2];
          y_data[j2] = xtmp;
        }
      }
      Vq = rtNaN;
      if ((!std::isnan(varargin_3)) && (!(varargin_3 > x_data[mid_i])) &&
          (!(varargin_3 < x_data[0]))) {
        j2 = 1;
        low_ip1 = 2;
        while (high_i > low_ip1) {
          mid_i = (j2 >> 1) + (high_i >> 1);
          if (((j2 & 1) == 1) && ((high_i & 1) == 1)) {
            mid_i++;
          }
          if (varargin_3 >= x_data[mid_i - 1]) {
            j2 = mid_i;
            low_ip1 = mid_i + 1;
          } else {
            high_i = mid_i;
          }
        }
        xtmp = x_data[j2 - 1];
        xtmp = (varargin_3 - xtmp) / (x_data[j2] - xtmp);
        if (xtmp == 0.0) {
          Vq = y_data[j2 - 1];
        } else if (xtmp == 1.0) {
          Vq = y_data[j2];
        } else {
          Vq = y_data[j2 - 1];
          if (!(Vq == y_data[j2])) {
            Vq = (1.0 - xtmp) * Vq + xtmp * y_data[j2];
          }
        }
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  return Vq;
}

} // namespace coder

//
// File trailer for interp1.cpp
//
// [EOF]
//
