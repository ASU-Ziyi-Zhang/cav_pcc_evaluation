//
// File: iterate.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "iterate.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "addBoundToActiveSetMatrix_.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "computeQ_.h"
#include "compute_deltax.h"
#include "compute_lambda.h"
#include "deleteColMoveEnd.h"
#include "factorQR.h"
#include "feasibleX0ForWorkingSet.h"
#include "feasibleratiotest.h"
#include "maxConstraintViolation.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xrotg.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cstring>
#include <emmintrin.h>

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
//                const char options_SolverName[7]
//                double options_StepTolerance
//                double options_ObjectiveLimit
//                int runTimeOptions_MaxIterations
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void iterate(const double H_data[], const int H_size[2], const double f_data[],
             i_struct_T &solution, h_struct_T &memspace, j_struct_T &workingset,
             e_struct_T &qrmanager, f_struct_T &cholmanager,
             g_struct_T &objective, const char options_SolverName[7],
             double options_StepTolerance, double options_ObjectiveLimit,
             int runTimeOptions_MaxIterations)
{
  static const char b[7]{'f', 'm', 'i', 'n', 'c', 'o', 'n'};
  double y_data[49];
  double d;
  double s;
  int TYPE;
  int activeSetChangeID;
  int globalActiveConstrIdx;
  int i;
  int iyend;
  int nVar;
  int temp_tmp;
  boolean_T subProblemChanged;
  boolean_T updateFval;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective.objtype;
  nVar = workingset.nVar;
  globalActiveConstrIdx = 0;
  Objective::computeGrad_StoreHx(objective, H_data, f_data,
                                 solution.xstar.data);
  solution.fstar = Objective::computeFval_ReuseHx(
      objective, memspace.workspace_float.data, f_data, solution.xstar.data);
  if (solution.iterations < runTimeOptions_MaxIterations) {
    solution.state = -5;
  } else {
    solution.state = 0;
  }
  iyend = workingset.mConstrMax;
  if (iyend - 1 >= 0) {
    std::memset(&solution.lambda.data[0], 0,
                static_cast<unsigned int>(iyend) * sizeof(double));
  }
  int exitg1;
  do {
    exitg1 = 0;
    if (solution.state == -5) {
      double temp;
      int Qk0;
      boolean_T guard1;
      boolean_T guard2;
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
        case 1: {
          double c;
          int ix0;
          int iy;
          ix0 = workingset.ldA * (workingset.nActiveConstr - 1);
          iyend = qrmanager.mrows;
          Qk0 = qrmanager.ncols + 1;
          if (iyend <= Qk0) {
            Qk0 = iyend;
          }
          qrmanager.minRowCol = Qk0;
          iy = qrmanager.ldq * qrmanager.ncols;
          Qk0 = qrmanager.ldq;
          if (qrmanager.mrows != 0) {
            iyend = iy + qrmanager.mrows;
            if (iy + 1 <= iyend) {
              std::memset(&qrmanager.QR.data[iy], 0,
                          static_cast<unsigned int>(iyend - iy) *
                              sizeof(double));
            }
            i = qrmanager.ldq * (qrmanager.mrows - 1) + 1;
            for (iyend = 1; Qk0 < 0 ? iyend >= i : iyend <= i; iyend += Qk0) {
              c = 0.0;
              temp_tmp = (iyend + qrmanager.mrows) - 1;
              for (int ia{iyend}; ia <= temp_tmp; ia++) {
                c += qrmanager.Q.data[ia - 1] *
                     workingset.ATwset.data[(ix0 + ia) - iyend];
              }
              qrmanager.QR.data[iy] += c;
              iy++;
            }
          }
          qrmanager.ncols++;
          i = qrmanager.ncols - 1;
          qrmanager.jpvt.data[i] = qrmanager.ncols;
          for (int idx{qrmanager.mrows - 2}; idx + 2 > qrmanager.ncols; idx--) {
            temp_tmp = idx + qrmanager.ldq * i;
            d = qrmanager.QR.data[temp_tmp + 1];
            c = internal::blas::xrotg(qrmanager.QR.data[temp_tmp], d, s);
            qrmanager.QR.data[temp_tmp + 1] = d;
            Qk0 = qrmanager.ldq * idx;
            iyend = qrmanager.mrows;
            if (qrmanager.mrows >= 1) {
              iy = qrmanager.ldq + Qk0;
              for (int ia{0}; ia < iyend; ia++) {
                temp_tmp = iy + ia;
                ix0 = Qk0 + ia;
                temp =
                    c * qrmanager.Q.data[ix0] + s * qrmanager.Q.data[temp_tmp];
                qrmanager.Q.data[temp_tmp] =
                    c * qrmanager.Q.data[temp_tmp] - s * qrmanager.Q.data[ix0];
                qrmanager.Q.data[ix0] = temp;
              }
            }
          }
        } break;
        case -1:
          QRManager::deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;
        default:
          QRManager::factorQR(qrmanager, workingset.ATwset.data, nVar,
                              workingset.nActiveConstr, workingset.ldA);
          QRManager::computeQ_(qrmanager, qrmanager.mrows);
          break;
        }
        iyend = std::memcmp(&options_SolverName[0], &b[0], 7);
        compute_deltax(H_data, H_size, solution, memspace, qrmanager,
                       cholmanager, objective, iyend == 0);
        if (solution.state != -5) {
          exitg1 = 1;
        } else if ((internal::blas::xnrm2(nVar, solution.searchDir.data) <
                    options_StepTolerance) ||
                   (workingset.nActiveConstr >= nVar)) {
          guard2 = true;
        } else {
          temp = feasibleratiotest(
              solution.xstar.data, solution.searchDir.data,
              memspace.workspace_float.data, memspace.workspace_float.size,
              workingset.nVar, workingset.ldA, workingset.Aineq.data,
              workingset.bineq.data, workingset.lb.data, workingset.ub.data,
              workingset.indexLB.data, workingset.indexUB.data,
              workingset.sizes, workingset.isActiveIdx,
              workingset.isActiveConstr.data, workingset.nWConstr, TYPE == 5,
              updateFval, i, temp_tmp);
          if (updateFval) {
            switch (i) {
            case 3:
              workingset.nWConstr[2]++;
              workingset.isActiveConstr
                  .data[(workingset.isActiveIdx[2] + temp_tmp) - 2] = true;
              workingset.nActiveConstr++;
              workingset.Wid.data[workingset.nActiveConstr - 1] = 3;
              workingset.Wlocalidx.data[workingset.nActiveConstr - 1] =
                  temp_tmp;
              Qk0 = workingset.ldA * (temp_tmp - 1);
              iyend = workingset.ldA * (workingset.nActiveConstr - 1);
              i = workingset.nVar - 1;
              for (int idx{0}; idx <= i; idx++) {
                workingset.ATwset.data[iyend + idx] =
                    workingset.Aineq.data[Qk0 + idx];
              }
              workingset.bwset.data[workingset.nActiveConstr - 1] =
                  workingset.bineq.data[temp_tmp - 1];
              break;
            case 4:
              WorkingSet::addBoundToActiveSetMatrix_(workingset, 4, temp_tmp);
              break;
            default:
              WorkingSet::addBoundToActiveSetMatrix_(workingset, 5, temp_tmp);
              break;
            }
            activeSetChangeID = 1;
          } else {
            if (objective.objtype == 5) {
              if (internal::blas::xnrm2(objective.nvar,
                                        solution.searchDir.data) >
                  100.0 * static_cast<double>(objective.nvar) *
                      1.4901161193847656E-8) {
                solution.state = 3;
              } else {
                solution.state = 4;
              }
            }
            subProblemChanged = false;
            if (workingset.nActiveConstr == 0) {
              solution.state = 1;
            }
          }
          if ((nVar >= 1) && (!(temp == 0.0))) {
            iyend = nVar - 1;
            Qk0 = (nVar / 2) << 1;
            temp_tmp = Qk0 - 2;
            for (int ia{0}; ia <= temp_tmp; ia += 2) {
              __m128d r;
              __m128d r1;
              r = _mm_loadu_pd(&solution.searchDir.data[ia]);
              r1 = _mm_loadu_pd(&solution.xstar.data[ia]);
              _mm_storeu_pd(&solution.xstar.data[ia],
                            _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(temp), r)));
            }
            for (int ia{Qk0}; ia <= iyend; ia++) {
              solution.xstar.data[ia] += temp * solution.searchDir.data[ia];
            }
          }
          Objective::computeGrad_StoreHx(objective, H_data, f_data,
                                         solution.xstar.data);
          updateFval = true;
          guard1 = true;
        }
      } else {
        Qk0 = solution.searchDir.size[0];
        iyend = solution.searchDir.size[0];
        if (iyend - 1 >= 0) {
          std::copy(&solution.searchDir.data[0],
                    &solution.searchDir.data[iyend], &y_data[0]);
        }
        if (nVar - 1 >= 0) {
          std::memset(&y_data[0], 0,
                      static_cast<unsigned int>(nVar) * sizeof(double));
        }
        if (Qk0 - 1 >= 0) {
          std::copy(&y_data[0], &y_data[Qk0], &solution.searchDir.data[0]);
        }
        guard2 = true;
      }
      if (guard2) {
        compute_lambda(memspace.workspace_float.data, solution, objective,
                       qrmanager);
        if ((solution.state != -7) || (workingset.nActiveConstr > nVar)) {
          iyend = 0;
          temp = 0.0;
          i = (workingset.nWConstr[0] + workingset.nWConstr[1]) + 1;
          temp_tmp = workingset.nActiveConstr;
          for (int idx{i}; idx <= temp_tmp; idx++) {
            d = solution.lambda.data[idx - 1];
            if (d < temp) {
              temp = d;
              iyend = idx;
            }
          }
          if (iyend == 0) {
            solution.state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = iyend;
            subProblemChanged = true;
            WorkingSet::removeConstr(workingset, iyend);
            if (iyend < workingset.nActiveConstr + 1) {
              solution.lambda.data[iyend - 1] =
                  solution.lambda.data[workingset.nActiveConstr];
            }
            solution.lambda.data[workingset.nActiveConstr] = 0.0;
          }
        } else {
          iyend = workingset.nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset.nActiveConstr;
          subProblemChanged = true;
          WorkingSet::removeConstr(workingset, workingset.nActiveConstr);
          solution.lambda.data[iyend - 1] = 0.0;
        }
        updateFval = false;
        guard1 = true;
      }
      if (guard1) {
        solution.iterations++;
        if ((solution.iterations >= runTimeOptions_MaxIterations) &&
            ((solution.state != 1) || (objective.objtype == 5))) {
          solution.state = 0;
        }
        if (solution.iterations - solution.iterations / 50 * 50 == 0) {
          solution.maxConstr = WorkingSet::maxConstraintViolation(
              workingset, solution.xstar.data);
          temp = solution.maxConstr;
          if (objective.objtype == 5) {
            temp = solution.maxConstr - solution.xstar.data[objective.nvar - 1];
          }
          if (temp > 1.0E-6) {
            boolean_T nonDegenerateWset;
            Qk0 = solution.searchDir.size[0];
            iyend = solution.searchDir.size[0];
            if (iyend - 1 >= 0) {
              std::copy(&solution.searchDir.data[0],
                        &solution.searchDir.data[iyend], &y_data[0]);
            }
            i = static_cast<unsigned char>(objective.nvar);
            if (i - 1 >= 0) {
              std::copy(&solution.xstar.data[0], &solution.xstar.data[i],
                        &y_data[0]);
            }
            if (Qk0 - 1 >= 0) {
              std::copy(&y_data[0], &y_data[Qk0], &solution.searchDir.data[0]);
            }
            nonDegenerateWset = initialize::feasibleX0ForWorkingSet(
                memspace.workspace_float.data, memspace.workspace_float.size,
                solution.searchDir.data, workingset, qrmanager);
            if ((!nonDegenerateWset) && (solution.state != 0)) {
              solution.state = -2;
            }
            activeSetChangeID = 0;
            temp = WorkingSet::maxConstraintViolation(workingset,
                                                      solution.searchDir.data);
            if (temp < solution.maxConstr) {
              if (i - 1 >= 0) {
                std::copy(&solution.searchDir.data[0],
                          &solution.searchDir.data[i], &solution.xstar.data[0]);
              }
              solution.maxConstr = temp;
            }
          }
        }
        if (updateFval) {
          if (options_ObjectiveLimit > rtMinusInf) {
            solution.fstar = Objective::computeFval_ReuseHx(
                objective, memspace.workspace_float.data, f_data,
                solution.xstar.data);
            if ((solution.fstar < options_ObjectiveLimit) &&
                ((solution.state != 0) || (objective.objtype != 5))) {
              solution.state = 2;
            }
          } else {
            updateFval = false;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution.fstar = Objective::computeFval_ReuseHx(
            objective, memspace.workspace_float.data, f_data,
            solution.xstar.data);
      }
      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for iterate.cpp
//
// [EOF]
//
