//
// File: xnrm2.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : int n
//                const double x[3]
// Return Type  : double
//
namespace coder {
namespace internal {
namespace blas {
double b_xnrm2(int n, const double x[3])
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[1]);
    } else {
      double absxk;
      double scale;
      double t;
      scale = 3.3121686421112381E-170;
      absxk = std::abs(x[1]);
      if (absxk > 3.3121686421112381E-170) {
        y = 1.0;
        scale = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        y = t * t;
      }
      absxk = std::abs(x[2]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

//
// Arguments    : int n
//                const double x_data[]
// Return Type  : double
//
double c_xnrm2(int n, const double x_data[])
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x_data[1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = n + 1;
      for (int k{2}; k <= kend; k++) {
        double absxk;
        absxk = std::abs(x_data[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

//
// Arguments    : int n
//                const double x_data[]
// Return Type  : double
//
double xnrm2(int n, const double x_data[])
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x_data[0]);
    } else {
      double scale;
      int i;
      scale = 3.3121686421112381E-170;
      i = static_cast<unsigned char>(n);
      for (int k{0}; k < i; k++) {
        double absxk;
        absxk = std::abs(x_data[k]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

//
// Arguments    : int n
//                const double x_data[]
//                int ix0
// Return Type  : double
//
double xnrm2(int n, const double x_data[], int ix0)
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x_data[ix0 - 1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int k{ix0}; k <= kend; k++) {
        double absxk;
        absxk = std::abs(x_data[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xnrm2.cpp
//
// [EOF]
//
