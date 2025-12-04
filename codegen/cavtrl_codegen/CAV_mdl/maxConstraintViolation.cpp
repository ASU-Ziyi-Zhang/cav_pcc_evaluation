//
// File: maxConstraintViolation.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "maxConstraintViolation.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : j_struct_T &obj
//                const double x_data[]
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
double maxConstraintViolation(j_struct_T &obj, const double x_data[])
{
  double v;
  int i;
  int idxLB;
  if (obj.probType == 2) {
    v = 0.0;
    if (obj.Aineq.size[0] != 0) {
      i = static_cast<unsigned char>(obj.sizes[2]);
      if (i - 1 >= 0) {
        std::copy(&obj.bineq.data[0], &obj.bineq.data[i],
                  &obj.maxConstrWorkspace.data[0]);
      }
      internal::blas::b_xgemv(obj.nVarOrig, obj.sizes[2], obj.Aineq.data,
                              obj.ldA, x_data, obj.maxConstrWorkspace.data);
      for (int idx{0}; idx < i; idx++) {
        obj.maxConstrWorkspace.data[idx] -= x_data[obj.nVarOrig + idx];
        v = std::fmax(v, obj.maxConstrWorkspace.data[idx]);
      }
    }
  } else {
    v = 0.0;
    if (obj.Aineq.size[0] != 0) {
      i = static_cast<unsigned char>(obj.sizes[2]);
      if (i - 1 >= 0) {
        std::copy(&obj.bineq.data[0], &obj.bineq.data[i],
                  &obj.maxConstrWorkspace.data[0]);
      }
      internal::blas::b_xgemv(obj.nVar, obj.sizes[2], obj.Aineq.data, obj.ldA,
                              x_data, obj.maxConstrWorkspace.data);
      for (int idx{0}; idx < i; idx++) {
        v = std::fmax(v, obj.maxConstrWorkspace.data[idx]);
      }
    }
  }
  if (obj.sizes[3] > 0) {
    i = static_cast<unsigned char>(obj.sizes[3]);
    for (int idx{0}; idx < i; idx++) {
      idxLB = obj.indexLB.data[idx] - 1;
      v = std::fmax(v, -x_data[idxLB] - obj.lb.data[idxLB]);
    }
  }
  if (obj.sizes[4] > 0) {
    i = static_cast<unsigned char>(obj.sizes[4]);
    for (int idx{0}; idx < i; idx++) {
      idxLB = obj.indexUB.data[idx] - 1;
      v = std::fmax(v, x_data[idxLB] - obj.ub.data[idxLB]);
    }
  }
  if (obj.sizes[0] > 0) {
    i = static_cast<unsigned char>(obj.sizes[0]);
    for (int idx{0}; idx < i; idx++) {
      v = std::fmax(v, std::abs(x_data[obj.indexFixed.data[idx] - 1] -
                                obj.ub.data[obj.indexFixed.data[idx] - 1]));
    }
  }
  return v;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for maxConstraintViolation.cpp
//
// [EOF]
//
