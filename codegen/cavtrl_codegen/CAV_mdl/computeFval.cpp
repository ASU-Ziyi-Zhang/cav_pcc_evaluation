//
// File: computeFval.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "computeFval.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "linearForm_.h"
#include "rt_nonfinite.h"
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const g_struct_T &obj
//                double workspace_data[]
//                const double H_data[]
//                const double f_data[]
//                const double x_data[]
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval(const g_struct_T &obj, double workspace_data[],
                   const double H_data[], const double f_data[],
                   const double x_data[])
{
  double val;
  switch (obj.objtype) {
  case 5:
    val = obj.gammaScalar * x_data[obj.nvar - 1];
    break;
  case 3: {
    linearForm_(obj.hasLinear, obj.nvar, workspace_data, H_data, f_data,
                x_data);
    val = 0.0;
    if (obj.nvar >= 1) {
      int i;
      i = static_cast<unsigned char>(obj.nvar);
      for (int k{0}; k < i; k++) {
        val += x_data[k] * workspace_data[k];
      }
    }
  } break;
  default: {
    int i;
    int k;
    int scalarLB;
    int vectorUB;
    linearForm_(obj.hasLinear, obj.nvar, workspace_data, H_data, f_data,
                x_data);
    i = obj.nvar + 1;
    k = obj.maxVar - 1;
    scalarLB = ((((k - i) + 1) / 2) << 1) + i;
    vectorUB = scalarLB - 2;
    for (int idx{i}; idx <= vectorUB; idx += 2) {
      _mm_storeu_pd(&workspace_data[idx - 1],
                    _mm_add_pd(_mm_mul_pd(_mm_set1_pd(0.5 * obj.beta),
                                          _mm_loadu_pd(&x_data[idx - 1])),
                               _mm_set1_pd(obj.rho)));
    }
    for (int idx{scalarLB}; idx <= k; idx++) {
      workspace_data[idx - 1] = 0.5 * obj.beta * x_data[idx - 1] + obj.rho;
    }
    val = 0.0;
    if (k >= 1) {
      i = static_cast<unsigned char>(k);
      for (k = 0; k < i; k++) {
        val += x_data[k] * workspace_data[k];
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
// File trailer for computeFval.cpp
//
// [EOF]
//
