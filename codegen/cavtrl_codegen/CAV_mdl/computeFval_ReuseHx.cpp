//
// File: computeFval_ReuseHx.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "computeFval_ReuseHx.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const g_struct_T &obj
//                double workspace_data[]
//                const double f_data[]
//                const double x_data[]
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval_ReuseHx(const g_struct_T &obj, double workspace_data[],
                           const double f_data[], const double x_data[])
{
  double val;
  switch (obj.objtype) {
  case 5:
    val = obj.gammaScalar * x_data[obj.nvar - 1];
    break;
  case 3: {
    if (obj.hasLinear) {
      int i;
      int idx;
      int k;
      i = static_cast<unsigned char>(obj.nvar);
      k = (i / 2) << 1;
      idx = k - 2;
      for (int b_i{0}; b_i <= idx; b_i += 2) {
        __m128d r;
        r = _mm_loadu_pd(&obj.Hx.data[b_i]);
        _mm_storeu_pd(&workspace_data[b_i],
                      _mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.5), r),
                                 _mm_loadu_pd(&f_data[b_i])));
      }
      for (int b_i{k}; b_i < i; b_i++) {
        workspace_data[b_i] = 0.5 * obj.Hx.data[b_i] + f_data[b_i];
      }
      val = 0.0;
      if (obj.nvar >= 1) {
        for (k = 0; k < i; k++) {
          val += x_data[k] * workspace_data[k];
        }
      }
    } else {
      val = 0.0;
      if (obj.nvar >= 1) {
        int i;
        i = static_cast<unsigned char>(obj.nvar);
        for (int k{0}; k < i; k++) {
          val += x_data[k] * obj.Hx.data[k];
        }
      }
      val *= 0.5;
    }
  } break;
  default: {
    int maxRegVar_tmp;
    maxRegVar_tmp = obj.maxVar - 1;
    if (obj.hasLinear) {
      int i;
      int idx;
      int k;
      i = static_cast<unsigned char>(obj.nvar);
      if (i - 1 >= 0) {
        std::copy(&f_data[0], &f_data[i], &workspace_data[0]);
      }
      i = obj.maxVar - obj.nvar;
      for (int b_i{0}; b_i <= i - 2; b_i++) {
        workspace_data[obj.nvar + b_i] = obj.rho;
      }
      i = static_cast<unsigned char>(obj.maxVar - 1);
      k = (i / 2) << 1;
      idx = k - 2;
      for (int b_i{0}; b_i <= idx; b_i += 2) {
        __m128d r;
        __m128d r1;
        r = _mm_loadu_pd(&obj.Hx.data[b_i]);
        r1 = _mm_loadu_pd(&workspace_data[b_i]);
        _mm_storeu_pd(&workspace_data[b_i],
                      _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(0.5), r)));
      }
      for (int b_i{k}; b_i < i; b_i++) {
        workspace_data[b_i] += 0.5 * obj.Hx.data[b_i];
      }
      val = 0.0;
      if (maxRegVar_tmp >= 1) {
        for (k = 0; k < i; k++) {
          val += x_data[k] * workspace_data[k];
        }
      }
    } else {
      int i;
      val = 0.0;
      if (maxRegVar_tmp >= 1) {
        i = static_cast<unsigned char>(maxRegVar_tmp);
        for (int k{0}; k < i; k++) {
          val += x_data[k] * obj.Hx.data[k];
        }
      }
      val *= 0.5;
      i = obj.nvar + 1;
      for (int idx{i}; idx <= maxRegVar_tmp; idx++) {
        val += x_data[idx - 1] * obj.rho;
      }
    }
  } break;
  }
  return val;
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeFval_ReuseHx.cpp
//
// [EOF]
//
