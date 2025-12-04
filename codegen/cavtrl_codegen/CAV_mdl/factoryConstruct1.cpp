//
// File: factoryConstruct1.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "factoryConstruct1.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "anonymous_function.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const anonymous_function &objfun
//                int nVar
//                const double lb_data[]
//                const int lb_size[2]
//                const double ub_data[]
//                const int ub_size[2]
//                m_struct_T &obj
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
void factoryConstruct(const anonymous_function &objfun, int nVar,
                      const double lb_data[], const int lb_size[2],
                      const double ub_data[], const int ub_size[2],
                      m_struct_T &obj)
{
  int idx;
  boolean_T b;
  obj.objfun = objfun;
  obj.f_1 = 0.0;
  obj.f_2 = 0.0;
  obj.nVar = nVar;
  obj.mIneq = 0;
  obj.mEq = 0;
  obj.numEvals = 0;
  obj.SpecifyObjectiveGradient = false;
  obj.SpecifyConstraintGradient = false;
  obj.isEmptyNonlcon = true;
  obj.hasLB.size[0] = nVar;
  obj.hasUB.size[0] = nVar;
  obj.FiniteDifferenceType = 1;
  b = false;
  idx = 0;
  switch (static_cast<unsigned int>(ub_size[1] == 0) << 1 |
          static_cast<unsigned int>(lb_size[1] == 0)) {
  case 0U:
    while ((!b) && (idx + 1 <= nVar)) {
      obj.hasLB.data[idx] =
          ((!std::isinf(lb_data[idx])) && (!std::isnan(lb_data[idx])));
      obj.hasUB.data[idx] =
          ((!std::isinf(ub_data[idx])) && (!std::isnan(ub_data[idx])));
      if (obj.hasLB.data[idx] || obj.hasUB.data[idx]) {
        b = true;
      }
      idx++;
    }
    while (idx + 1 <= nVar) {
      obj.hasLB.data[idx] =
          ((!std::isinf(lb_data[idx])) && (!std::isnan(lb_data[idx])));
      obj.hasUB.data[idx] =
          ((!std::isinf(ub_data[idx])) && (!std::isnan(ub_data[idx])));
      idx++;
    }
    break;
  case 1U:
    while ((!b) && (idx + 1 <= nVar)) {
      obj.hasLB.data[idx] = false;
      obj.hasUB.data[idx] =
          ((!std::isinf(ub_data[idx])) && (!std::isnan(ub_data[idx])));
      b = obj.hasUB.data[idx];
      idx++;
    }
    while (idx + 1 <= nVar) {
      obj.hasLB.data[idx] = false;
      obj.hasUB.data[idx] =
          ((!std::isinf(ub_data[idx])) && (!std::isnan(ub_data[idx])));
      idx++;
    }
    break;
  case 2U:
    while ((!b) && (idx + 1 <= nVar)) {
      obj.hasLB.data[idx] =
          ((!std::isinf(lb_data[idx])) && (!std::isnan(lb_data[idx])));
      obj.hasUB.data[idx] = false;
      b = obj.hasLB.data[idx];
      idx++;
    }
    while (idx + 1 <= nVar) {
      obj.hasLB.data[idx] =
          ((!std::isinf(lb_data[idx])) && (!std::isnan(lb_data[idx])));
      obj.hasUB.data[idx] = false;
      idx++;
    }
    break;
  default:
    idx = static_cast<unsigned char>(nVar);
    if (idx - 1 >= 0) {
      std::memset(&obj.hasLB.data[0], 0,
                  static_cast<unsigned int>(idx) * sizeof(boolean_T));
      std::memset(&obj.hasUB.data[0], 0,
                  static_cast<unsigned int>(idx) * sizeof(boolean_T));
    }
    break;
  }
  obj.hasBounds = b;
}

} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for factoryConstruct1.cpp
//
// [EOF]
//
