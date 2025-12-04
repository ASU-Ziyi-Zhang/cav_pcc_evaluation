//
// File: computeFiniteDifferences.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "computeFiniteDifferences.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types1.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types2.h"
#include "RefTrjGnrtr_240503.h"
#include "anonymous_function.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : m_struct_T &obj
//                double fCurrent
//                double xk_data[]
//                const int xk_size[2]
//                double gradf_data[]
//                const double lb_data[]
//                const double ub_data[]
// Return Type  : boolean_T
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
boolean_T computeFiniteDifferences(m_struct_T &obj, double fCurrent,
                                   double xk_data[], const int xk_size[2],
                                   double gradf_data[], const double lb_data[],
                                   const double ub_data[])
{
  int i;
  int idx;
  boolean_T evalOK;
  boolean_T exitg1;
  evalOK = true;
  obj.numEvals = 0;
  i = static_cast<unsigned char>(obj.nVar);
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= i - 1)) {
    double delta1;
    double distFar;
    double distNear;
    double distNear_tmp;
    int formulaType;
    distNear = 1.0E-8 * std::fmax(std::abs(xk_data[idx]), 1.0);
    if (obj.hasLB.data[idx] || obj.hasUB.data[idx]) {
      if (obj.hasLB.data[idx] && obj.hasUB.data[idx]) {
        formulaType = 0;
        if ((lb_data[idx] != ub_data[idx]) && (xk_data[idx] >= lb_data[idx]) &&
            (xk_data[idx] <= ub_data[idx])) {
          if (xk_data[idx] - distNear < lb_data[idx]) {
            if (ub_data[idx] < xk_data[idx] + distNear) {
              delta1 = xk_data[idx] - lb_data[idx];
              distNear_tmp = ub_data[idx] - xk_data[idx];
              distNear = std::fmin(delta1, distNear_tmp);
              distFar = std::fmax(delta1, distNear_tmp);
              if (!(distNear >= distFar / 2.0)) {
                distNear = distFar / 2.0;
                if (delta1 >= distNear_tmp) {
                  formulaType = -1;
                } else {
                  formulaType = 1;
                }
              }
            } else if (xk_data[idx] + 2.0 * distNear <= ub_data[idx]) {
              formulaType = 1;
            } else {
              delta1 = xk_data[idx] - lb_data[idx];
              distNear = (ub_data[idx] - xk_data[idx]) / 2.0;
              if (delta1 >= distNear) {
                distNear = delta1;
              } else {
                formulaType = 1;
              }
            }
          } else if (ub_data[idx] < xk_data[idx] + distNear) {
            if (lb_data[idx] <= xk_data[idx] - 2.0 * distNear) {
              formulaType = -1;
            } else {
              delta1 = ub_data[idx] - xk_data[idx];
              distNear = (xk_data[idx] - lb_data[idx]) / 2.0;
              if (delta1 >= distNear) {
                distNear = delta1;
              } else {
                formulaType = -1;
              }
            }
          }
        }
      } else if (obj.hasUB.data[idx]) {
        formulaType = 0;
        if ((xk_data[idx] <= ub_data[idx]) &&
            (ub_data[idx] < xk_data[idx] + distNear)) {
          formulaType = -1;
        }
      } else {
        formulaType = 0;
        if ((xk_data[idx] >= lb_data[idx]) &&
            (xk_data[idx] - distNear < lb_data[idx])) {
          formulaType = 1;
        }
      }
    } else {
      formulaType = 0;
    }
    switch (formulaType) {
    case 0:
      delta1 = -distNear;
      distFar = distNear;
      break;
    case -1:
      delta1 = -2.0 * distNear;
      distFar = -distNear;
      break;
    default:
      delta1 = distNear;
      distFar = 2.0 * distNear;
      break;
    }
    double temp;
    int exitg2;
    do {
      exitg2 = 0;
      temp = xk_data[idx];
      xk_data[idx] += delta1;
      distNear_tmp = OptEntTimeSearch_v2_anonFcn1(
          obj.objfun.workspace.si_arr.data, obj.objfun.workspace.si_arr.size,
          obj.objfun.workspace.t0s, obj.objfun.workspace.tf0s,
          obj.objfun.workspace.vfs, obj.objfun.workspace.v0s,
          obj.objfun.workspace.ViolInfo.idxTLStp.data,
          obj.objfun.workspace.ViolInfo.idxTLStp.size,
          obj.objfun.workspace.ViolInfo.tint.data,
          obj.objfun.workspace.ViolInfo.tint.size, xk_data, xk_size);
      evalOK = ((!std::isinf(distNear_tmp)) && (!std::isnan(distNear_tmp)));
      if (evalOK) {
        xk_data[idx] = temp;
      }
      obj.f_1 = distNear_tmp;
      obj.numEvals++;
      if (!evalOK) {
        if ((formulaType == 0) &&
            ((!obj.hasBounds) ||
             (obj.hasUB.data[idx] &&
              (xk_data[idx] + 2.0 * distNear <= ub_data[idx])))) {
          formulaType = 1;
          delta1 = distNear;
          distFar = 2.0 * distNear;
        } else {
          exitg2 = 1;
        }
      } else {
        temp = xk_data[idx];
        xk_data[idx] += distFar;
        distNear_tmp = OptEntTimeSearch_v2_anonFcn1(
            obj.objfun.workspace.si_arr.data, obj.objfun.workspace.si_arr.size,
            obj.objfun.workspace.t0s, obj.objfun.workspace.tf0s,
            obj.objfun.workspace.vfs, obj.objfun.workspace.v0s,
            obj.objfun.workspace.ViolInfo.idxTLStp.data,
            obj.objfun.workspace.ViolInfo.idxTLStp.size,
            obj.objfun.workspace.ViolInfo.tint.data,
            obj.objfun.workspace.ViolInfo.tint.size, xk_data, xk_size);
        evalOK = ((!std::isinf(distNear_tmp)) && (!std::isnan(distNear_tmp)));
        if (evalOK) {
          xk_data[idx] = temp;
        }
        obj.f_2 = distNear_tmp;
        obj.numEvals++;
        if ((!evalOK) && (formulaType == 0) &&
            ((!obj.hasBounds) ||
             (obj.hasLB.data[idx] &&
              (xk_data[idx] - 2.0 * distNear >= lb_data[idx])))) {
          formulaType = -1;
          delta1 = -2.0 * distNear;
          distFar = -distNear;
        } else {
          exitg2 = 1;
        }
      }
    } while (exitg2 == 0);
    if (!evalOK) {
      exitg1 = true;
    } else {
      delta1 = obj.f_1;
      distNear_tmp = obj.f_2;
      switch (formulaType) {
      case 0:
        gradf_data[idx] = (-delta1 + distNear_tmp) / (2.0 * distNear);
        break;
      case 1:
        gradf_data[idx] = ((-3.0 * fCurrent + 4.0 * delta1) - distNear_tmp) /
                          (2.0 * distNear);
        break;
      default:
        gradf_data[idx] =
            ((delta1 - 4.0 * distNear_tmp) + 3.0 * fCurrent) / (2.0 * distNear);
        break;
      }
      idx++;
    }
  }
  return evalOK;
}

} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeFiniteDifferences.cpp
//
// [EOF]
//
