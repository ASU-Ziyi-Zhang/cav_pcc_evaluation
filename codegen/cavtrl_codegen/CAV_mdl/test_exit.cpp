//
// File: test_exit.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "test_exit.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "computeComplError.h"
#include "computeDualFeasError.h"
#include "computeGradLag.h"
#include "computePrimalFeasError.h"
#include "computeQ_.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xgeqp3.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : b_struct_T &Flags
//                h_struct_T &memspace
//                struct_T &MeritFunction
//                int fscales_lineq_constraint_size
//                const j_struct_T &WorkingSet
//                i_struct_T &b_TrialState
//                e_struct_T &b_QRManager
//                const double lb_data[]
//                const double ub_data[]
//                int runTimeOptions_MaxFunctionEvaluations
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void b_test_exit(b_struct_T &Flags, h_struct_T &memspace,
                 struct_T &MeritFunction, int fscales_lineq_constraint_size,
                 const j_struct_T &WorkingSet, i_struct_T &b_TrialState,
                 e_struct_T &b_QRManager, const double lb_data[],
                 const double ub_data[],
                 int runTimeOptions_MaxFunctionEvaluations)
{
  double optimRelativeFactor;
  double s;
  double smax;
  int fullRank_R;
  int i;
  int idx_max;
  int nVar;
  boolean_T isFeasible;
  nVar = WorkingSet.nVar;
  i = static_cast<unsigned char>(
      ((WorkingSet.sizes[0] + WorkingSet.sizes[2]) + WorkingSet.sizes[3]) +
      WorkingSet.sizes[4]);
  if (i - 1 >= 0) {
    std::copy(&b_TrialState.lambdasqp.data[0], &b_TrialState.lambdasqp.data[i],
              &b_TrialState.lambdaStopTest.data[0]);
  }
  stopping::computeGradLag(
      b_TrialState.gradLag.data, WorkingSet.ldA, WorkingSet.nVar,
      b_TrialState.grad.data, WorkingSet.sizes[2], WorkingSet.Aineq.data,
      WorkingSet.indexFixed.data, WorkingSet.sizes[0], WorkingSet.indexLB.data,
      WorkingSet.sizes[3], WorkingSet.indexUB.data, WorkingSet.sizes[4],
      b_TrialState.lambdaStopTest.data);
  if (WorkingSet.nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet.nVar > 1) {
      smax = std::abs(b_TrialState.grad.data[0]);
      for (fullRank_R = 2; fullRank_R <= nVar; fullRank_R++) {
        s = std::abs(b_TrialState.grad.data[fullRank_R - 1]);
        if (s > smax) {
          idx_max = fullRank_R;
          smax = s;
        }
      }
    }
  }
  optimRelativeFactor =
      std::fmax(1.0, std::abs(b_TrialState.grad.data[idx_max - 1]));
  if (std::isinf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }
  MeritFunction.nlpPrimalFeasError = stopping::computePrimalFeasError(
      b_TrialState.xstarsqp.data, WorkingSet.sizes[2], b_TrialState.cIneq.data,
      WorkingSet.indexLB.data, WorkingSet.sizes[3], lb_data,
      WorkingSet.indexUB.data, WorkingSet.sizes[4], ub_data);
  if (b_TrialState.sqpIterations == 0) {
    MeritFunction.feasRelativeFactor =
        std::fmax(1.0, MeritFunction.nlpPrimalFeasError);
  }
  isFeasible = (MeritFunction.nlpPrimalFeasError <=
                1.0E-6 * MeritFunction.feasRelativeFactor);
  Flags.gradOK =
      stopping::computeDualFeasError(WorkingSet.nVar, b_TrialState.gradLag.data,
                                     MeritFunction.nlpDualFeasError);
  if (!Flags.gradOK) {
    Flags.done = true;
    if (isFeasible) {
      b_TrialState.sqpExitFlag = 2;
    } else {
      b_TrialState.sqpExitFlag = -2;
    }
  } else {
    MeritFunction.nlpComplError = stopping::computeComplError(
        fscales_lineq_constraint_size, b_TrialState.xstarsqp.data,
        WorkingSet.sizes[2], b_TrialState.cIneq.data, WorkingSet.indexLB.data,
        WorkingSet.sizes[3], lb_data, WorkingSet.indexUB.data,
        WorkingSet.sizes[4], ub_data, b_TrialState.lambdaStopTest.data,
        WorkingSet.sizes[0] + 1);
    MeritFunction.firstOrderOpt =
        std::fmax(MeritFunction.nlpDualFeasError, MeritFunction.nlpComplError);
    if (b_TrialState.sqpIterations > 1) {
      stopping::computeGradLag(
          memspace.workspace_float.data, WorkingSet.ldA, WorkingSet.nVar,
          b_TrialState.grad.data, WorkingSet.sizes[2], WorkingSet.Aineq.data,
          WorkingSet.indexFixed.data, WorkingSet.sizes[0],
          WorkingSet.indexLB.data, WorkingSet.sizes[3], WorkingSet.indexUB.data,
          WorkingSet.sizes[4], b_TrialState.lambdaStopTestPrev.data);
      stopping::computeDualFeasError(WorkingSet.nVar,
                                     memspace.workspace_float.data, smax);
      s = stopping::computeComplError(
          fscales_lineq_constraint_size, b_TrialState.xstarsqp.data,
          WorkingSet.sizes[2], b_TrialState.cIneq.data, WorkingSet.indexLB.data,
          WorkingSet.sizes[3], lb_data, WorkingSet.indexUB.data,
          WorkingSet.sizes[4], ub_data, b_TrialState.lambdaStopTestPrev.data,
          WorkingSet.sizes[0] + 1);
      if ((smax < MeritFunction.nlpDualFeasError) &&
          (s < MeritFunction.nlpComplError)) {
        MeritFunction.nlpDualFeasError = smax;
        MeritFunction.nlpComplError = s;
        MeritFunction.firstOrderOpt = std::fmax(smax, s);
        if (i - 1 >= 0) {
          std::copy(&b_TrialState.lambdaStopTestPrev.data[0],
                    &b_TrialState.lambdaStopTestPrev.data[i],
                    &b_TrialState.lambdaStopTest.data[0]);
        }
      } else if (i - 1 >= 0) {
        std::copy(&b_TrialState.lambdaStopTest.data[0],
                  &b_TrialState.lambdaStopTest.data[i],
                  &b_TrialState.lambdaStopTestPrev.data[0]);
      }
    } else if (i - 1 >= 0) {
      std::copy(&b_TrialState.lambdaStopTest.data[0],
                &b_TrialState.lambdaStopTest.data[i],
                &b_TrialState.lambdaStopTestPrev.data[0]);
    }
    if (isFeasible &&
        (MeritFunction.nlpDualFeasError <= 1.0E-6 * optimRelativeFactor) &&
        (MeritFunction.nlpComplError <= 1.0E-6 * optimRelativeFactor)) {
      Flags.done = true;
      b_TrialState.sqpExitFlag = 1;
    } else {
      Flags.done = false;
      if (isFeasible && (b_TrialState.sqpFval < -1.0E+20)) {
        Flags.done = true;
        b_TrialState.sqpExitFlag = -3;
      } else {
        boolean_T guard1;
        guard1 = false;
        if (b_TrialState.sqpIterations > 0) {
          int idx;
          int j;
          boolean_T dxTooSmall;
          boolean_T exitg1;
          dxTooSmall = true;
          j = static_cast<unsigned char>(WorkingSet.nVar);
          idx = 0;
          exitg1 = false;
          while ((!exitg1) && (idx <= j - 1)) {
            if (1.0E-6 *
                    std::fmax(1.0, std::abs(b_TrialState.xstarsqp.data[idx])) <=
                std::abs(b_TrialState.delta_x.data[idx])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              idx++;
            }
          }
          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags.stepType != 2) {
                Flags.stepType = 2;
                Flags.failedLineSearch = false;
                Flags.stepAccepted = false;
                guard1 = true;
              } else {
                Flags.done = true;
                b_TrialState.sqpExitFlag = -2;
              }
            } else {
              int nActiveConstr;
              nActiveConstr = WorkingSet.nActiveConstr - 1;
              if (WorkingSet.nActiveConstr == 0) {
                Flags.done = true;
                b_TrialState.sqpExitFlag = 2;
              } else {
                double d;
                int ix;
                boolean_T guard2;
                if (nActiveConstr >= 0) {
                  std::memset(&b_TrialState.lambda.data[0], 0,
                              static_cast<unsigned int>(nActiveConstr + 1) *
                                  sizeof(double));
                }
                fullRank_R = WorkingSet.nVar * WorkingSet.nActiveConstr;
                guard2 = false;
                if (fullRank_R > 0) {
                  for (idx = 0; idx <= nActiveConstr; idx++) {
                    idx_max = WorkingSet.ldA * idx;
                    ix = b_QRManager.ldq * idx;
                    for (fullRank_R = 0; fullRank_R < j; fullRank_R++) {
                      b_QRManager.QR.data[ix + fullRank_R] =
                          WorkingSet.ATwset.data[idx_max + fullRank_R];
                    }
                  }
                  guard2 = true;
                } else if (fullRank_R == 0) {
                  b_QRManager.mrows = WorkingSet.nVar;
                  b_QRManager.ncols = WorkingSet.nActiveConstr;
                  b_QRManager.minRowCol = 0;
                } else {
                  guard2 = true;
                }
                if (guard2) {
                  b_QRManager.usedPivoting = true;
                  b_QRManager.mrows = WorkingSet.nVar;
                  b_QRManager.ncols = WorkingSet.nActiveConstr;
                  ix = WorkingSet.nVar;
                  idx_max = WorkingSet.nActiveConstr;
                  if (ix <= idx_max) {
                    idx_max = ix;
                  }
                  b_QRManager.minRowCol = idx_max;
                  b_QRManager.tau.size[0] = ::coder::internal::lapack::xgeqp3(
                      b_QRManager.QR.data, b_QRManager.QR.size, WorkingSet.nVar,
                      WorkingSet.nActiveConstr, b_QRManager.jpvt.data,
                      b_QRManager.tau.data);
                }
                QRManager::computeQ_(b_QRManager, b_QRManager.mrows);
                idx_max = b_QRManager.ldq;
                if (WorkingSet.nVar != 0) {
                  std::memset(&memspace.workspace_float.data[0], 0,
                              static_cast<unsigned int>(j) * sizeof(double));
                  ix = 0;
                  j = b_QRManager.ldq * (WorkingSet.nVar - 1) + 1;
                  for (nActiveConstr = 1;
                       idx_max < 0 ? nActiveConstr >= j : nActiveConstr <= j;
                       nActiveConstr += idx_max) {
                    smax = 0.0;
                    fullRank_R = (nActiveConstr + nVar) - 1;
                    for (idx = nActiveConstr; idx <= fullRank_R; idx++) {
                      smax += b_QRManager.Q.data[idx - 1] *
                              b_TrialState.grad.data[idx - nActiveConstr];
                    }
                    memspace.workspace_float.data[ix] -= smax;
                    ix++;
                  }
                }
                ix = WorkingSet.nVar;
                idx_max = WorkingSet.nActiveConstr;
                if (ix >= idx_max) {
                  idx_max = ix;
                }
                smax = std::abs(b_QRManager.QR.data[0]) *
                       std::fmin(1.4901161193847656E-8,
                                 static_cast<double>(idx_max) *
                                     2.2204460492503131E-16);
                ix = WorkingSet.nVar;
                fullRank_R = WorkingSet.nActiveConstr;
                if (ix <= fullRank_R) {
                  fullRank_R = ix;
                }
                nActiveConstr = 0;
                idx_max = 0;
                while ((nActiveConstr < fullRank_R) &&
                       (std::abs(b_QRManager.QR.data[idx_max]) > smax)) {
                  nActiveConstr++;
                  idx_max = (idx_max + b_QRManager.ldq) + 1;
                }
                if ((b_QRManager.QR.size[0] != 0) &&
                    (b_QRManager.QR.size[1] != 0) && (nActiveConstr != 0)) {
                  for (j = nActiveConstr; j >= 1; j--) {
                    idx_max = (j + (j - 1) * b_QRManager.ldq) - 1;
                    memspace.workspace_float.data[j - 1] /=
                        b_QRManager.QR.data[idx_max];
                    for (nVar = 0; nVar <= j - 2; nVar++) {
                      ix = (j - nVar) - 2;
                      memspace.workspace_float.data[ix] -=
                          memspace.workspace_float.data[j - 1] *
                          b_QRManager.QR.data[(idx_max - nVar) - 1];
                    }
                  }
                }
                ix = WorkingSet.nActiveConstr;
                if (ix <= fullRank_R) {
                  fullRank_R = ix;
                }
                for (idx = 0; idx < fullRank_R; idx++) {
                  b_TrialState.lambda.data[b_QRManager.jpvt.data[idx] - 1] =
                      memspace.workspace_float.data[idx];
                }
                qpactiveset::parseoutput::sortLambdaQP(
                    b_TrialState.lambda.data, WorkingSet.nActiveConstr,
                    WorkingSet.sizes, WorkingSet.isActiveIdx,
                    WorkingSet.Wid.data, WorkingSet.Wlocalidx.data,
                    memspace.workspace_float.data);
                stopping::computeGradLag(
                    memspace.workspace_float.data, WorkingSet.ldA,
                    WorkingSet.nVar, b_TrialState.grad.data,
                    WorkingSet.sizes[2], WorkingSet.Aineq.data,
                    WorkingSet.indexFixed.data, WorkingSet.sizes[0],
                    WorkingSet.indexLB.data, WorkingSet.sizes[3],
                    WorkingSet.indexUB.data, WorkingSet.sizes[4],
                    b_TrialState.lambda.data);
                stopping::computeDualFeasError(
                    WorkingSet.nVar, memspace.workspace_float.data, smax);
                s = stopping::computeComplError(
                    fscales_lineq_constraint_size, b_TrialState.xstarsqp.data,
                    WorkingSet.sizes[2], b_TrialState.cIneq.data,
                    WorkingSet.indexLB.data, WorkingSet.sizes[3], lb_data,
                    WorkingSet.indexUB.data, WorkingSet.sizes[4], ub_data,
                    b_TrialState.lambda.data, WorkingSet.sizes[0] + 1);
                d = std::fmax(smax, s);
                if (d <= std::fmax(MeritFunction.nlpDualFeasError,
                                   MeritFunction.nlpComplError)) {
                  MeritFunction.nlpDualFeasError = smax;
                  MeritFunction.nlpComplError = s;
                  MeritFunction.firstOrderOpt = d;
                  if (i - 1 >= 0) {
                    std::copy(&b_TrialState.lambda.data[0],
                              &b_TrialState.lambda.data[i],
                              &b_TrialState.lambdaStopTest.data[0]);
                  }
                }
                if ((MeritFunction.nlpDualFeasError <=
                     1.0E-6 * optimRelativeFactor) &&
                    (MeritFunction.nlpComplError <=
                     1.0E-6 * optimRelativeFactor)) {
                  b_TrialState.sqpExitFlag = 1;
                } else {
                  b_TrialState.sqpExitFlag = 2;
                }
                Flags.done = true;
                guard1 = true;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          if (b_TrialState.sqpIterations >= 400) {
            Flags.done = true;
            b_TrialState.sqpExitFlag = 0;
          } else if (b_TrialState.FunctionEvaluations >=
                     runTimeOptions_MaxFunctionEvaluations) {
            Flags.done = true;
            b_TrialState.sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

//
// Arguments    : struct_T &MeritFunction
//                const j_struct_T &WorkingSet
//                i_struct_T &b_TrialState
//                const double lb_data[]
//                const double ub_data[]
//                int runTimeOptions_MaxFunctionEvaluations
//                boolean_T &Flags_fevalOK
//                boolean_T &Flags_done
//                boolean_T &Flags_stepAccepted
//                boolean_T &Flags_failedLineSearch
//                int &Flags_stepType
// Return Type  : boolean_T
//
boolean_T test_exit(struct_T &MeritFunction, const j_struct_T &WorkingSet,
                    i_struct_T &b_TrialState, const double lb_data[],
                    const double ub_data[],
                    int runTimeOptions_MaxFunctionEvaluations,
                    boolean_T &Flags_fevalOK, boolean_T &Flags_done,
                    boolean_T &Flags_stepAccepted,
                    boolean_T &Flags_failedLineSearch, int &Flags_stepType)
{
  double smax;
  int i;
  int idx_max;
  int nVar;
  boolean_T Flags_gradOK;
  boolean_T isFeasible;
  Flags_fevalOK = true;
  Flags_done = false;
  Flags_stepAccepted = false;
  Flags_failedLineSearch = false;
  Flags_stepType = 1;
  nVar = WorkingSet.nVar;
  i = static_cast<unsigned char>(
      ((WorkingSet.sizes[0] + WorkingSet.sizes[2]) + WorkingSet.sizes[3]) +
      WorkingSet.sizes[4]);
  if (i - 1 >= 0) {
    std::copy(&b_TrialState.lambdasqp.data[0], &b_TrialState.lambdasqp.data[i],
              &b_TrialState.lambdaStopTest.data[0]);
  }
  stopping::computeGradLag(
      b_TrialState.gradLag.data, WorkingSet.ldA, WorkingSet.nVar,
      b_TrialState.grad.data, WorkingSet.sizes[2], WorkingSet.Aineq.data,
      WorkingSet.indexFixed.data, WorkingSet.sizes[0], WorkingSet.indexLB.data,
      WorkingSet.sizes[3], WorkingSet.indexUB.data, WorkingSet.sizes[4],
      b_TrialState.lambdaStopTest.data);
  if (WorkingSet.nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet.nVar > 1) {
      smax = std::abs(b_TrialState.grad.data[0]);
      for (int k{2}; k <= nVar; k++) {
        double s;
        s = std::abs(b_TrialState.grad.data[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }
  smax = std::fmax(1.0, std::abs(b_TrialState.grad.data[idx_max - 1]));
  if (std::isinf(smax)) {
    smax = 1.0;
  }
  MeritFunction.nlpPrimalFeasError = stopping::computePrimalFeasError(
      b_TrialState.xstarsqp.data, WorkingSet.sizes[2], b_TrialState.cIneq.data,
      WorkingSet.indexLB.data, WorkingSet.sizes[3], lb_data,
      WorkingSet.indexUB.data, WorkingSet.sizes[4], ub_data);
  MeritFunction.feasRelativeFactor =
      std::fmax(1.0, MeritFunction.nlpPrimalFeasError);
  isFeasible = (MeritFunction.nlpPrimalFeasError <=
                1.0E-6 * MeritFunction.feasRelativeFactor);
  Flags_gradOK =
      stopping::computeDualFeasError(WorkingSet.nVar, b_TrialState.gradLag.data,
                                     MeritFunction.nlpDualFeasError);
  if (!Flags_gradOK) {
    Flags_done = true;
    if (isFeasible) {
      b_TrialState.sqpExitFlag = 2;
    } else {
      b_TrialState.sqpExitFlag = -2;
    }
  } else {
    MeritFunction.nlpComplError = 0.0;
    MeritFunction.firstOrderOpt =
        std::fmax(MeritFunction.nlpDualFeasError, 0.0);
    if (i - 1 >= 0) {
      std::copy(&b_TrialState.lambdaStopTest.data[0],
                &b_TrialState.lambdaStopTest.data[i],
                &b_TrialState.lambdaStopTestPrev.data[0]);
    }
    if (isFeasible && (MeritFunction.nlpDualFeasError <= 1.0E-6 * smax)) {
      Flags_done = true;
      b_TrialState.sqpExitFlag = 1;
    } else if (isFeasible && (b_TrialState.sqpFval < -1.0E+20)) {
      Flags_done = true;
      b_TrialState.sqpExitFlag = -3;
    } else if (b_TrialState.FunctionEvaluations >=
               runTimeOptions_MaxFunctionEvaluations) {
      Flags_done = true;
      b_TrialState.sqpExitFlag = 0;
    }
  }
  return Flags_gradOK;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for test_exit.cpp
//
// [EOF]
//
