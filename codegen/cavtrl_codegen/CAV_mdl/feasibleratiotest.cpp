//
// File: feasibleratiotest.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "feasibleratiotest.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xnrm2.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double solution_xstar_data[]
//                const double solution_searchDir_data[]
//                double workspace_data[]
//                const int workspace_size[2]
//                int workingset_nVar
//                int workingset_ldA
//                const double workingset_Aineq_data[]
//                const double workingset_bineq_data[]
//                const double workingset_lb_data[]
//                const double workingset_ub_data[]
//                const int workingset_indexLB_data[]
//                const int workingset_indexUB_data[]
//                const int workingset_sizes[5]
//                const int workingset_isActiveIdx[6]
//                const boolean_T workingset_isActiveConstr_data[]
//                const int workingset_nWConstr[5]
//                boolean_T isPhaseOne
//                boolean_T &newBlocking
//                int &constrType
//                int &constrIdx
// Return Type  : double
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
double feasibleratiotest(
    const double solution_xstar_data[], const double solution_searchDir_data[],
    double workspace_data[], const int workspace_size[2], int workingset_nVar,
    int workingset_ldA, const double workingset_Aineq_data[],
    const double workingset_bineq_data[], const double workingset_lb_data[],
    const double workingset_ub_data[], const int workingset_indexLB_data[],
    const int workingset_indexUB_data[], const int workingset_sizes[5],
    const int workingset_isActiveIdx[6],
    const boolean_T workingset_isActiveConstr_data[],
    const int workingset_nWConstr[5], boolean_T isPhaseOne,
    boolean_T &newBlocking, int &constrType, int &constrIdx)
{
  double alpha;
  double c;
  double denomTol;
  double phaseOneCorrectionP;
  double phaseOneCorrectionX;
  double ratio;
  int i;
  int iyend;
  alpha = 1.0E+30;
  newBlocking = false;
  constrType = 0;
  constrIdx = 0;
  denomTol = 2.2204460492503131E-13 *
             internal::blas::xnrm2(workingset_nVar, solution_searchDir_data);
  if (workingset_nWConstr[2] < workingset_sizes[2]) {
    int iy;
    int ldw_tmp;
    i = static_cast<unsigned char>(workingset_sizes[2]);
    if (i - 1 >= 0) {
      std::copy(&workingset_bineq_data[0], &workingset_bineq_data[i],
                &workspace_data[0]);
    }
    internal::blas::b_xgemv(workingset_nVar, workingset_sizes[2],
                            workingset_Aineq_data, workingset_ldA,
                            solution_xstar_data, workspace_data);
    ldw_tmp = workspace_size[0];
    iy = workspace_size[0] + 1;
    if ((workingset_nVar != 0) && (workingset_sizes[2] != 0)) {
      iyend = workspace_size[0] + workingset_sizes[2];
      if (iy <= iyend) {
        std::memset(&workspace_data[iy + -1], 0,
                    static_cast<unsigned int>((iyend - iy) + 1) *
                        sizeof(double));
      }
      iy = ldw_tmp;
      iyend = workingset_ldA * (workingset_sizes[2] - 1) + 1;
      for (int iac{1}; workingset_ldA < 0 ? iac >= iyend : iac <= iyend;
           iac += workingset_ldA) {
        int i1;
        c = 0.0;
        i1 = (iac + workingset_nVar) - 1;
        for (int ia{iac}; ia <= i1; ia++) {
          c +=
              workingset_Aineq_data[ia - 1] * solution_searchDir_data[ia - iac];
        }
        workspace_data[iy] += c;
        iy++;
      }
    }
    for (iyend = 0; iyend < i; iyend++) {
      c = workspace_data[ldw_tmp + iyend];
      if ((c > denomTol) &&
          (!workingset_isActiveConstr_data[(workingset_isActiveIdx[2] + iyend) -
                                           1])) {
        c = std::fmin(std::abs(workspace_data[iyend]),
                      1.0E-6 - workspace_data[iyend]) /
            c;
        if (c < alpha) {
          alpha = c;
          constrType = 3;
          constrIdx = iyend + 1;
          newBlocking = true;
        }
      }
    }
  }
  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    phaseOneCorrectionX = static_cast<double>(isPhaseOne) *
                          solution_xstar_data[workingset_nVar - 1];
    phaseOneCorrectionP = static_cast<double>(isPhaseOne) *
                          solution_searchDir_data[workingset_nVar - 1];
    i = workingset_sizes[3];
    for (iyend = 0; iyend <= i - 2; iyend++) {
      c = -solution_searchDir_data[workingset_indexLB_data[iyend] - 1] -
          phaseOneCorrectionP;
      if ((c > denomTol) &&
          (!workingset_isActiveConstr_data[(workingset_isActiveIdx[3] + iyend) -
                                           1])) {
        ratio = (-solution_xstar_data[workingset_indexLB_data[iyend] - 1] -
                 workingset_lb_data[workingset_indexLB_data[iyend] - 1]) -
                phaseOneCorrectionX;
        c = std::fmin(std::abs(ratio), 1.0E-6 - ratio) / c;
        if (c < alpha) {
          alpha = c;
          constrType = 4;
          constrIdx = iyend + 1;
          newBlocking = true;
        }
      }
    }
    i = workingset_indexLB_data[workingset_sizes[3] - 1] - 1;
    c = -solution_searchDir_data[i];
    if ((c > denomTol) &&
        (!workingset_isActiveConstr_data
             [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio = -solution_xstar_data[i] - workingset_lb_data[i];
      c = std::fmin(std::abs(ratio), 1.0E-6 - ratio) / c;
      if (c < alpha) {
        alpha = c;
        constrType = 4;
        constrIdx = workingset_sizes[3];
        newBlocking = true;
      }
    }
  }
  if (workingset_nWConstr[4] < workingset_sizes[4]) {
    phaseOneCorrectionX = static_cast<double>(isPhaseOne) *
                          solution_xstar_data[workingset_nVar - 1];
    phaseOneCorrectionP = static_cast<double>(isPhaseOne) *
                          solution_searchDir_data[workingset_nVar - 1];
    i = static_cast<unsigned char>(workingset_sizes[4]);
    for (iyend = 0; iyend < i; iyend++) {
      c = solution_searchDir_data[workingset_indexUB_data[iyend] - 1] -
          phaseOneCorrectionP;
      if ((c > denomTol) &&
          (!workingset_isActiveConstr_data[(workingset_isActiveIdx[4] + iyend) -
                                           1])) {
        ratio = (solution_xstar_data[workingset_indexUB_data[iyend] - 1] -
                 workingset_ub_data[workingset_indexUB_data[iyend] - 1]) -
                phaseOneCorrectionX;
        c = std::fmin(std::abs(ratio), 1.0E-6 - ratio) / c;
        if (c < alpha) {
          alpha = c;
          constrType = 5;
          constrIdx = iyend + 1;
          newBlocking = true;
        }
      }
    }
  }
  if (!isPhaseOne) {
    if (newBlocking && (alpha > 1.0)) {
      newBlocking = false;
    }
    alpha = std::fmin(alpha, 1.0);
  }
  return alpha;
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for feasibleratiotest.cpp
//
// [EOF]
//
