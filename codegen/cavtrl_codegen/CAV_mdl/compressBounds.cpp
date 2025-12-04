//
// File: compressBounds.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "compressBounds.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : int nVar
//                int indexLB_data[]
//                int indexUB_data[]
//                int indexFixed_data[]
//                const double lb_data[]
//                const int lb_size[2]
//                const double ub_data[]
//                const int ub_size[2]
//                int &mUB
//                int &mFixed
// Return Type  : int
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
int compressBounds(int nVar, int indexLB_data[], int indexUB_data[],
                   int indexFixed_data[], const double lb_data[],
                   const int lb_size[2], const double ub_data[],
                   const int ub_size[2], int &mUB, int &mFixed)
{
  int mLB;
  mLB = 0;
  mUB = 0;
  mFixed = 0;
  if (ub_size[1] != 0) {
    if (lb_size[1] != 0) {
      int i;
      i = static_cast<unsigned char>(nVar);
      for (int idx{0}; idx < i; idx++) {
        double d;
        boolean_T guard1;
        d = lb_data[idx];
        guard1 = false;
        if ((!std::isinf(d)) && (!std::isnan(d))) {
          if (std::abs(d - ub_data[idx]) < 1.0E-6) {
            mFixed++;
            indexFixed_data[mFixed - 1] = idx + 1;
          } else {
            mLB++;
            indexLB_data[mLB - 1] = idx + 1;
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          d = ub_data[idx];
          if ((!std::isinf(d)) && (!std::isnan(d))) {
            mUB++;
            indexUB_data[mUB - 1] = idx + 1;
          }
        }
      }
    } else {
      int i;
      i = static_cast<unsigned char>(nVar);
      for (int idx{0}; idx < i; idx++) {
        double d;
        d = ub_data[idx];
        if ((!std::isinf(d)) && (!std::isnan(d))) {
          mUB++;
          indexUB_data[mUB - 1] = idx + 1;
        }
      }
    }
  } else if (lb_size[1] != 0) {
    int i;
    i = static_cast<unsigned char>(nVar);
    for (int idx{0}; idx < i; idx++) {
      double d;
      d = lb_data[idx];
      if ((!std::isinf(d)) && (!std::isnan(d))) {
        mLB++;
        indexLB_data[mLB - 1] = idx + 1;
      }
    }
  }
  return mLB;
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for compressBounds.cpp
//
// [EOF]
//
