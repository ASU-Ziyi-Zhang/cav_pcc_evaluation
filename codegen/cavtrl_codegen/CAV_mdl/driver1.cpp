//
// File: driver1.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "driver1.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "PresolveWorkingSet.h"
#include "computeFval.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// Arguments    : const double H_data[]
//                const int H_size[2]
//                const double f_data[]
//                i_struct_T &solution
//                h_struct_T &memspace
//                j_struct_T &workingset
//                e_struct_T &qrmanager
//                f_struct_T &cholmanager
//                g_struct_T &objective
//                l_struct_T &options
//                int runTimeOptions_MaxIterations
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void driver(const double H_data[], const int H_size[2], const double f_data[],
            i_struct_T &solution, h_struct_T &memspace, j_struct_T &workingset,
            e_struct_T &qrmanager, f_struct_T &cholmanager,
            g_struct_T &objective, l_struct_T &options,
            int runTimeOptions_MaxIterations)
{
  double y_data[49];
  int idxEndIneq_tmp_tmp;
  int mConstr;
  int nVar_tmp;
  boolean_T guard1;
  solution.iterations = 0;
  nVar_tmp = workingset.nVar;
  guard1 = false;
  if (workingset.probType == 3) {
    idxEndIneq_tmp_tmp = static_cast<unsigned char>(workingset.sizes[0]);
    for (mConstr = 0; mConstr < idxEndIneq_tmp_tmp; mConstr++) {
      solution.xstar.data[workingset.indexFixed.data[mConstr] - 1] =
          workingset.ub.data[workingset.indexFixed.data[mConstr] - 1];
    }
    idxEndIneq_tmp_tmp = static_cast<unsigned char>(workingset.sizes[3]);
    for (mConstr = 0; mConstr < idxEndIneq_tmp_tmp; mConstr++) {
      if (workingset.isActiveConstr
              .data[(workingset.isActiveIdx[3] + mConstr) - 1]) {
        solution.xstar.data[workingset.indexLB.data[mConstr] - 1] =
            -workingset.lb.data[workingset.indexLB.data[mConstr] - 1];
      }
    }
    idxEndIneq_tmp_tmp = static_cast<unsigned char>(workingset.sizes[4]);
    for (mConstr = 0; mConstr < idxEndIneq_tmp_tmp; mConstr++) {
      if (workingset.isActiveConstr
              .data[(workingset.isActiveIdx[4] + mConstr) - 1]) {
        solution.xstar.data[workingset.indexUB.data[mConstr] - 1] =
            workingset.ub.data[workingset.indexUB.data[mConstr] - 1];
      }
    }
    initialize::PresolveWorkingSet(solution, memspace, workingset, qrmanager);
    if (solution.state >= 0) {
      guard1 = true;
    }
  } else {
    solution.state = 82;
    guard1 = true;
  }
  if (guard1) {
    solution.iterations = 0;
    solution.maxConstr =
        WorkingSet::maxConstraintViolation(workingset, solution.xstar.data);
    if (solution.maxConstr > 1.0E-6) {
      int PROBTYPE_ORIG;
      int idxStartIneq;
      PROBTYPE_ORIG = workingset.probType;
      solution.xstar.data[workingset.nVar] = solution.maxConstr + 1.0;
      if (workingset.probType == 3) {
        idxEndIneq_tmp_tmp = 1;
      } else {
        idxEndIneq_tmp_tmp = 4;
      }
      WorkingSet::setProblemType(workingset, idxEndIneq_tmp_tmp);
      mConstr = workingset.nWConstr[0] + workingset.nWConstr[1];
      idxStartIneq = mConstr + 1;
      idxEndIneq_tmp_tmp = workingset.nActiveConstr;
      for (int idx_global{idxStartIneq}; idx_global <= idxEndIneq_tmp_tmp;
           idx_global++) {
        workingset.isActiveConstr.data
            [(workingset.isActiveIdx[workingset.Wid.data[idx_global - 1] - 1] +
              workingset.Wlocalidx.data[idx_global - 1]) -
             2] = false;
      }
      workingset.nWConstr[2] = 0;
      workingset.nWConstr[3] = 0;
      workingset.nWConstr[4] = 0;
      workingset.nActiveConstr = mConstr;
      objective.prev_objtype = objective.objtype;
      objective.prev_nvar = objective.nvar;
      objective.prev_hasLinear = objective.hasLinear;
      objective.objtype = 5;
      objective.nvar = nVar_tmp + 1;
      objective.gammaScalar = 1.0;
      objective.hasLinear = true;
      solution.fstar =
          Objective::computeFval(objective, memspace.workspace_float.data,
                                 H_data, f_data, solution.xstar.data);
      solution.state = 5;
      iterate(H_data, H_size, f_data, solution, memspace, workingset, qrmanager,
              cholmanager, objective, options.SolverName,
              1.4901161193847657E-10, 1.0E-6, runTimeOptions_MaxIterations);
      if (workingset.isActiveConstr
              .data[(workingset.isActiveIdx[3] + workingset.sizes[3]) - 2]) {
        boolean_T exitg1;
        mConstr = workingset.sizes[0];
        exitg1 = false;
        while ((!exitg1) && (mConstr + 1 <= workingset.nActiveConstr)) {
          if ((workingset.Wid.data[mConstr] == 4) &&
              (workingset.Wlocalidx.data[mConstr] == workingset.sizes[3])) {
            WorkingSet::removeConstr(workingset, mConstr + 1);
            exitg1 = true;
          } else {
            mConstr++;
          }
        }
      }
      mConstr = workingset.nActiveConstr;
      idxStartIneq = workingset.sizes[0];
      while ((mConstr > idxStartIneq) && (mConstr > nVar_tmp)) {
        WorkingSet::removeConstr(workingset, mConstr);
        mConstr--;
      }
      solution.maxConstr = solution.xstar.data[nVar_tmp];
      WorkingSet::setProblemType(workingset, PROBTYPE_ORIG);
      objective.objtype = objective.prev_objtype;
      objective.nvar = objective.prev_nvar;
      objective.hasLinear = objective.prev_hasLinear;
      options.ObjectiveLimit = rtMinusInf;
      options.StepTolerance = 1.0E-6;
      if (solution.state != 0) {
        solution.maxConstr =
            WorkingSet::maxConstraintViolation(workingset, solution.xstar.data);
        if (solution.maxConstr > 1.0E-6) {
          mConstr = workingset.mConstrMax;
          if (mConstr - 1 >= 0) {
            std::memset(&solution.lambda.data[0], 0,
                        static_cast<unsigned int>(mConstr) * sizeof(double));
          }
          solution.fstar =
              Objective::computeFval(objective, memspace.workspace_float.data,
                                     H_data, f_data, solution.xstar.data);
          solution.state = -2;
        } else {
          if (solution.maxConstr > 0.0) {
            double maxConstr_new;
            mConstr = solution.searchDir.size[0];
            idxStartIneq = solution.searchDir.size[0];
            if (idxStartIneq - 1 >= 0) {
              std::copy(&solution.searchDir.data[0],
                        &solution.searchDir.data[idxStartIneq], &y_data[0]);
            }
            idxEndIneq_tmp_tmp = static_cast<unsigned char>(nVar_tmp);
            if (idxEndIneq_tmp_tmp - 1 >= 0) {
              std::copy(&solution.xstar.data[0],
                        &solution.xstar.data[idxEndIneq_tmp_tmp], &y_data[0]);
            }
            if (mConstr - 1 >= 0) {
              std::copy(&y_data[0], &y_data[mConstr],
                        &solution.searchDir.data[0]);
            }
            initialize::PresolveWorkingSet(solution, memspace, workingset,
                                           qrmanager);
            maxConstr_new = WorkingSet::maxConstraintViolation(
                workingset, solution.xstar.data);
            if (maxConstr_new >= solution.maxConstr) {
              solution.maxConstr = maxConstr_new;
              mConstr = solution.xstar.size[0];
              idxStartIneq = solution.xstar.size[0];
              if (idxStartIneq - 1 >= 0) {
                std::copy(&solution.xstar.data[0],
                          &solution.xstar.data[idxStartIneq], &y_data[0]);
              }
              if (idxEndIneq_tmp_tmp - 1 >= 0) {
                std::copy(&solution.searchDir.data[0],
                          &solution.searchDir.data[idxEndIneq_tmp_tmp],
                          &y_data[0]);
              }
              if (mConstr - 1 >= 0) {
                std::copy(&y_data[0], &y_data[mConstr],
                          &solution.xstar.data[0]);
              }
            }
          }
          iterate(H_data, H_size, f_data, solution, memspace, workingset,
                  qrmanager, cholmanager, objective, options.SolverName,
                  options.StepTolerance, options.ObjectiveLimit,
                  runTimeOptions_MaxIterations);
        }
      }
    } else {
      iterate(H_data, H_size, f_data, solution, memspace, workingset, qrmanager,
              cholmanager, objective, options.SolverName, options.StepTolerance,
              options.ObjectiveLimit, runTimeOptions_MaxIterations);
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for driver1.cpp
//
// [EOF]
//
