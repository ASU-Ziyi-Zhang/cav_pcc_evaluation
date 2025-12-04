//
// File: driver.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "driver.h"
#include "BFGSUpdate.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "computeFiniteDifferences.h"
#include "evalObjAndConstr.h"
#include "rt_nonfinite.h"
#include "saveState.h"
#include "step.h"
#include "stickyStruct.h"
#include "test_exit.h"
#include "xgemv.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : double Hessian_data[]
//                const int Hessian_size[2]
//                const double bineq_data[]
//                const double lb_data[]
//                const int lb_size[2]
//                const double ub_data[]
//                const int ub_size[2]
//                i_struct_T &b_TrialState
//                struct_T &MeritFunction
//                const ::coder::internal::i_stickyStruct &FcnEvaluator
//                m_struct_T &FiniteDifferences
//                h_struct_T &memspace
//                j_struct_T &WorkingSet
//                e_struct_T &b_QRManager
//                f_struct_T &b_CholManager
//                g_struct_T &QPObjective
//                int fscales_lineq_constraint_size
//                const k_struct_T &runTimeOptions
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void driver(double Hessian_data[], const int Hessian_size[2],
            const double bineq_data[], const double lb_data[],
            const int lb_size[2], const double ub_data[], const int ub_size[2],
            i_struct_T &b_TrialState, struct_T &MeritFunction,
            const ::coder::internal::i_stickyStruct &FcnEvaluator,
            m_struct_T &FiniteDifferences, h_struct_T &memspace,
            j_struct_T &WorkingSet, e_struct_T &b_QRManager,
            f_struct_T &b_CholManager, g_struct_T &QPObjective,
            int fscales_lineq_constraint_size, const k_struct_T &runTimeOptions)
{
  static const char qpoptions_SolverName[7]{'f', 'm', 'i', 'n', 'c', 'o', 'n'};
  b_struct_T Flags;
  l_struct_T b_expl_temp;
  l_struct_T expl_temp;
  double y_data[49];
  double b_y_data[25];
  int ineqStart;
  int ixlast;
  int mConstr;
  int mFixed;
  int mIneq;
  int mLB;
  int mUB;
  int nVar_tmp_tmp;
  int qpoptions_MaxIterations;
  int vectorUB;
  nVar_tmp_tmp = WorkingSet.nVar;
  mFixed = WorkingSet.sizes[0];
  mIneq = WorkingSet.sizes[2];
  mLB = WorkingSet.sizes[3];
  mUB = WorkingSet.sizes[4];
  mConstr =
      ((WorkingSet.sizes[0] + WorkingSet.sizes[2]) + WorkingSet.sizes[3]) +
      WorkingSet.sizes[4];
  ineqStart = WorkingSet.nVar;
  ixlast = ((WorkingSet.sizes[2] + WorkingSet.sizes[3]) + WorkingSet.sizes[4]) +
           (WorkingSet.sizes[0] << 1);
  if (ineqStart >= ixlast) {
    ixlast = ineqStart;
  }
  qpoptions_MaxIterations = 10 * ixlast;
  b_TrialState.steplength = 1.0;
  Flags.gradOK = test_exit(MeritFunction, WorkingSet, b_TrialState, lb_data,
                           ub_data, runTimeOptions.MaxFunctionEvaluations,
                           Flags.fevalOK, Flags.done, Flags.stepAccepted,
                           Flags.failedLineSearch, Flags.stepType);
  TrialState::saveState(b_TrialState);
  if (!Flags.done) {
    b_TrialState.sqpIterations = 1;
  }
  while (!Flags.done) {
    __m128d r;
    __m128d r1;
    double phi_alpha;
    int i;
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      double constrViolationIneq;
      boolean_T hasLB;
      boolean_T hasUB;
      if (Flags.stepType != 3) {
        i = static_cast<unsigned char>(mIneq);
        ineqStart = (static_cast<unsigned char>(mIneq) >> 1) << 1;
        vectorUB = ineqStart - 2;
        for (ixlast = 0; ixlast <= vectorUB; ixlast += 2) {
          r = _mm_loadu_pd(&b_TrialState.cIneq.data[ixlast]);
          _mm_storeu_pd(&WorkingSet.bineq.data[ixlast],
                        _mm_mul_pd(r, _mm_set1_pd(-1.0)));
        }
        for (ixlast = ineqStart; ixlast < i; ixlast++) {
          WorkingSet.bineq.data[ixlast] = -b_TrialState.cIneq.data[ixlast];
        }
        hasLB = (lb_size[1] != 0);
        hasUB = (ub_size[1] != 0);
        if (hasLB) {
          i = static_cast<unsigned char>(mLB);
          for (ixlast = 0; ixlast < i; ixlast++) {
            WorkingSet.lb.data[WorkingSet.indexLB.data[ixlast] - 1] =
                -lb_data[WorkingSet.indexLB.data[ixlast] - 1] +
                b_TrialState.xstarsqp.data[WorkingSet.indexLB.data[ixlast] - 1];
          }
        }
        if (hasUB) {
          i = static_cast<unsigned char>(mUB);
          for (ixlast = 0; ixlast < i; ixlast++) {
            WorkingSet.ub.data[WorkingSet.indexUB.data[ixlast] - 1] =
                ub_data[WorkingSet.indexUB.data[ixlast] - 1] -
                b_TrialState.xstarsqp.data[WorkingSet.indexUB.data[ixlast] - 1];
          }
        }
        if (hasLB && hasUB) {
          i = static_cast<unsigned char>(mFixed);
          for (ixlast = 0; ixlast < i; ixlast++) {
            phi_alpha = ub_data[WorkingSet.indexFixed.data[ixlast] - 1] -
                        b_TrialState.xstarsqp
                            .data[WorkingSet.indexFixed.data[ixlast] - 1];
            WorkingSet.ub.data[WorkingSet.indexFixed.data[ixlast] - 1] =
                phi_alpha;
            WorkingSet.bwset.data[ixlast] = phi_alpha;
          }
        }
        if (WorkingSet.nActiveConstr > mFixed) {
          ineqStart = mFixed + 1;
          if (ineqStart < 1) {
            ineqStart = 1;
          }
          i = WorkingSet.nActiveConstr;
          for (ixlast = ineqStart; ixlast <= i; ixlast++) {
            switch (WorkingSet.Wid.data[ixlast - 1]) {
            case 4:
              WorkingSet.bwset.data[ixlast - 1] =
                  WorkingSet.lb
                      .data[WorkingSet.indexLB.data
                                [WorkingSet.Wlocalidx.data[ixlast - 1] - 1] -
                            1];
              break;
            case 5:
              WorkingSet.bwset.data[ixlast - 1] =
                  WorkingSet.ub
                      .data[WorkingSet.indexUB.data
                                [WorkingSet.Wlocalidx.data[ixlast - 1] - 1] -
                            1];
              break;
            default:
              WorkingSet.bwset.data[ixlast - 1] =
                  WorkingSet.bineq
                      .data[WorkingSet.Wlocalidx.data[ixlast - 1] - 1];
              break;
            }
          }
        }
      }
      expl_temp.ObjectiveLimit = rtMinusInf;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (i = 0; i < 7; i++) {
        expl_temp.SolverName[i] = qpoptions_SolverName[i];
      }
      b_expl_temp = expl_temp;
      Flags.stepAccepted = b_step(
          Flags.stepType, Hessian_data, Hessian_size, lb_data, lb_size, ub_data,
          ub_size, b_TrialState, MeritFunction, memspace, WorkingSet,
          b_QRManager, b_CholManager, QPObjective, b_expl_temp);
      if (Flags.stepAccepted) {
        i = static_cast<unsigned char>(nVar_tmp_tmp);
        for (ineqStart = 0; ineqStart < i; ineqStart++) {
          b_TrialState.xstarsqp.data[ineqStart] +=
              b_TrialState.delta_x.data[ineqStart];
        }
        b_TrialState.sqpFval = utils::ObjNonlinEvaluator::evalObjAndConstr(
            FcnEvaluator, b_TrialState.xstarsqp.data,
            b_TrialState.xstarsqp.size, ineqStart);
        Flags.fevalOK = (ineqStart == 1);
        b_TrialState.FunctionEvaluations++;
        if (mIneq > 0) {
          ineqStart = b_TrialState.cIneq.size[0];
          ixlast = b_TrialState.cIneq.size[0];
          if (ixlast - 1 >= 0) {
            std::copy(&b_TrialState.cIneq.data[0],
                      &b_TrialState.cIneq.data[ixlast], &b_y_data[0]);
          }
          i = static_cast<unsigned char>(mIneq);
          if (i - 1 >= 0) {
            std::copy(&bineq_data[0], &bineq_data[i], &b_y_data[0]);
          }
          if (ineqStart - 1 >= 0) {
            std::copy(&b_y_data[0], &b_y_data[ineqStart],
                      &b_TrialState.cIneq.data[0]);
          }
          ::coder::internal::blas::xgemv(
              nVar_tmp_tmp, mIneq, WorkingSet.Aineq.data, WorkingSet.ldA,
              b_TrialState.xstarsqp.data, b_TrialState.cIneq.data);
        }
        if (Flags.fevalOK) {
          constrViolationIneq = 0.0;
          i = static_cast<unsigned char>(mIneq);
          for (ixlast = 0; ixlast < i; ixlast++) {
            phi_alpha = b_TrialState.cIneq.data[ixlast];
            if (phi_alpha > 0.0) {
              constrViolationIneq += phi_alpha;
            }
          }
          MeritFunction.phiFullStep =
              b_TrialState.sqpFval +
              MeritFunction.penaltyParam * constrViolationIneq;
        } else {
          MeritFunction.phiFullStep = rtInf;
        }
      }
      if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
          (MeritFunction.phi < MeritFunction.phiFullStep) &&
          (b_TrialState.sqpFval < b_TrialState.sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        double alpha;
        int b_mIneq;
        int exitflagLnSrch;
        int i1;
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          hasLB = true;
        } else {
          hasLB = false;
        }
        hasUB = Flags.fevalOK;
        i = WorkingSet.nVar;
        b_mIneq = b_TrialState.mIneq;
        alpha = 1.0;
        exitflagLnSrch = 1;
        phi_alpha = MeritFunction.phiFullStep;
        ineqStart = b_TrialState.searchDir.size[0];
        ixlast = b_TrialState.searchDir.size[0];
        if (ixlast - 1 >= 0) {
          std::copy(&b_TrialState.searchDir.data[0],
                    &b_TrialState.searchDir.data[ixlast], &y_data[0]);
        }
        i1 = static_cast<unsigned char>(WorkingSet.nVar);
        if (i1 - 1 >= 0) {
          std::copy(&b_TrialState.delta_x.data[0],
                    &b_TrialState.delta_x.data[i1], &y_data[0]);
        }
        if (ineqStart - 1 >= 0) {
          std::copy(&y_data[0], &y_data[ineqStart],
                    &b_TrialState.searchDir.data[0]);
        }
        int exitg1;
        do {
          exitg1 = 0;
          if (b_TrialState.FunctionEvaluations <
              runTimeOptions.MaxFunctionEvaluations) {
            if (hasUB && (phi_alpha <=
                          MeritFunction.phi +
                              alpha * 0.0001 * MeritFunction.phiPrimePlus)) {
              exitg1 = 1;
            } else {
              boolean_T exitg2;
              boolean_T tooSmallX;
              alpha *= 0.7;
              i1 = static_cast<unsigned char>(i);
              ineqStart = (static_cast<unsigned char>(i) >> 1) << 1;
              vectorUB = ineqStart - 2;
              for (ixlast = 0; ixlast <= vectorUB; ixlast += 2) {
                r = _mm_loadu_pd(&b_TrialState.xstar.data[ixlast]);
                _mm_storeu_pd(&b_TrialState.delta_x.data[ixlast],
                              _mm_mul_pd(_mm_set1_pd(alpha), r));
              }
              for (ixlast = ineqStart; ixlast < i1; ixlast++) {
                b_TrialState.delta_x.data[ixlast] =
                    alpha * b_TrialState.xstar.data[ixlast];
              }
              if (hasLB) {
                phi_alpha = alpha * alpha;
                if ((i >= 1) && (!(phi_alpha == 0.0))) {
                  ixlast = i - 1;
                  ineqStart = (i / 2) << 1;
                  vectorUB = ineqStart - 2;
                  for (int k{0}; k <= vectorUB; k += 2) {
                    r = _mm_loadu_pd(&b_TrialState.socDirection.data[k]);
                    r1 = _mm_loadu_pd(&b_TrialState.delta_x.data[k]);
                    _mm_storeu_pd(
                        &b_TrialState.delta_x.data[k],
                        _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(phi_alpha), r)));
                  }
                  for (int k{ineqStart}; k <= ixlast; k++) {
                    b_TrialState.delta_x.data[k] +=
                        phi_alpha * b_TrialState.socDirection.data[k];
                  }
                }
              }
              tooSmallX = true;
              ixlast = 0;
              exitg2 = false;
              while ((!exitg2) &&
                     (ixlast <= static_cast<unsigned char>(i) - 1)) {
                if (1.0E-6 *
                        std::fmax(
                            1.0,
                            std::abs(b_TrialState.xstarsqp.data[ixlast])) <=
                    std::abs(b_TrialState.delta_x.data[ixlast])) {
                  tooSmallX = false;
                  exitg2 = true;
                } else {
                  ixlast++;
                }
              }
              if (tooSmallX) {
                exitflagLnSrch = -2;
                exitg1 = 1;
              } else {
                for (ixlast = 0; ixlast < i1; ixlast++) {
                  b_TrialState.xstarsqp.data[ixlast] =
                      b_TrialState.xstarsqp_old.data[ixlast] +
                      b_TrialState.delta_x.data[ixlast];
                }
                b_TrialState.sqpFval =
                    utils::ObjNonlinEvaluator::evalObjAndConstr(
                        FcnEvaluator, b_TrialState.xstarsqp.data,
                        b_TrialState.xstarsqp.size, vectorUB);
                if (b_mIneq > 0) {
                  ineqStart = b_TrialState.cIneq.size[0];
                  ixlast = b_TrialState.cIneq.size[0];
                  if (ixlast - 1 >= 0) {
                    std::copy(&b_TrialState.cIneq.data[0],
                              &b_TrialState.cIneq.data[ixlast], &b_y_data[0]);
                  }
                  i1 = static_cast<unsigned char>(b_mIneq);
                  if (i1 - 1 >= 0) {
                    std::copy(&bineq_data[0], &bineq_data[i1], &b_y_data[0]);
                  }
                  if (ineqStart - 1 >= 0) {
                    std::copy(&b_y_data[0], &b_y_data[ineqStart],
                              &b_TrialState.cIneq.data[0]);
                  }
                  ::coder::internal::blas::xgemv(
                      i, b_mIneq, WorkingSet.Aineq.data, WorkingSet.ldA,
                      b_TrialState.xstarsqp.data, b_TrialState.cIneq.data);
                }
                b_TrialState.FunctionEvaluations++;
                hasUB = (vectorUB == 1);
                if (hasUB) {
                  constrViolationIneq = 0.0;
                  i1 = static_cast<unsigned char>(b_mIneq);
                  for (ixlast = 0; ixlast < i1; ixlast++) {
                    phi_alpha = b_TrialState.cIneq.data[ixlast];
                    if (phi_alpha > 0.0) {
                      constrViolationIneq += phi_alpha;
                    }
                  }
                  phi_alpha = b_TrialState.sqpFval +
                              MeritFunction.penaltyParam * constrViolationIneq;
                } else {
                  phi_alpha = rtInf;
                }
              }
            }
          } else {
            exitflagLnSrch = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        Flags.fevalOK = hasUB;
        b_TrialState.steplength = alpha;
        if (exitflagLnSrch > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }
    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      i = static_cast<unsigned char>(nVar_tmp_tmp);
      for (ixlast = 0; ixlast < i; ixlast++) {
        b_TrialState.xstarsqp.data[ixlast] =
            b_TrialState.xstarsqp_old.data[ixlast] +
            b_TrialState.delta_x.data[ixlast];
      }
      i = static_cast<unsigned char>(mConstr);
      ineqStart = (static_cast<unsigned char>(mConstr) >> 1) << 1;
      vectorUB = ineqStart - 2;
      for (ixlast = 0; ixlast <= vectorUB; ixlast += 2) {
        r = _mm_loadu_pd(&b_TrialState.lambda.data[ixlast]);
        r1 = _mm_loadu_pd(&b_TrialState.lambdasqp.data[ixlast]);
        _mm_storeu_pd(
            &b_TrialState.lambdasqp.data[ixlast],
            _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(b_TrialState.steplength),
                                      _mm_sub_pd(r, r1))));
      }
      for (ixlast = ineqStart; ixlast < i; ixlast++) {
        phi_alpha = b_TrialState.lambdasqp.data[ixlast];
        phi_alpha += b_TrialState.steplength *
                     (b_TrialState.lambda.data[ixlast] - phi_alpha);
        b_TrialState.lambdasqp.data[ixlast] = phi_alpha;
      }
      TrialState::saveState(b_TrialState);
      Flags.gradOK = utils::FiniteDifferences::computeFiniteDifferences(
          FiniteDifferences, b_TrialState.sqpFval, b_TrialState.xstarsqp.data,
          b_TrialState.xstarsqp.size, b_TrialState.grad.data, lb_data, ub_data);
      b_TrialState.FunctionEvaluations += FiniteDifferences.numEvals;
    } else {
      b_TrialState.sqpFval = b_TrialState.sqpFval_old;
      i = b_TrialState.xstarsqp.size[1];
      if (i - 1 >= 0) {
        std::copy(&b_TrialState.xstarsqp_old.data[0],
                  &b_TrialState.xstarsqp_old.data[i],
                  &b_TrialState.xstarsqp.data[0]);
      }
      ineqStart = b_TrialState.cIneq.size[0];
      ixlast = b_TrialState.cIneq.size[0];
      if (ixlast - 1 >= 0) {
        std::copy(&b_TrialState.cIneq.data[0], &b_TrialState.cIneq.data[ixlast],
                  &y_data[0]);
      }
      i = static_cast<unsigned char>(b_TrialState.mIneq);
      if (i - 1 >= 0) {
        std::copy(&b_TrialState.cIneq_old.data[0],
                  &b_TrialState.cIneq_old.data[i], &y_data[0]);
      }
      if (ineqStart - 1 >= 0) {
        std::copy(&y_data[0], &y_data[ineqStart], &b_TrialState.cIneq.data[0]);
      }
    }
    b_test_exit(Flags, memspace, MeritFunction, fscales_lineq_constraint_size,
                WorkingSet, b_TrialState, b_QRManager, lb_data, ub_data,
                runTimeOptions.MaxFunctionEvaluations);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      i = static_cast<unsigned char>(nVar_tmp_tmp);
      if (i - 1 >= 0) {
        std::copy(&b_TrialState.grad.data[0], &b_TrialState.grad.data[i],
                  &b_TrialState.delta_gradLag.data[0]);
      }
      if (nVar_tmp_tmp >= 1) {
        ixlast = nVar_tmp_tmp - 1;
        ineqStart = (nVar_tmp_tmp / 2) << 1;
        vectorUB = ineqStart - 2;
        for (int k{0}; k <= vectorUB; k += 2) {
          r = _mm_loadu_pd(&b_TrialState.delta_gradLag.data[k]);
          r1 = _mm_loadu_pd(&b_TrialState.grad_old.data[k]);
          _mm_storeu_pd(&b_TrialState.delta_gradLag.data[k], _mm_sub_pd(r, r1));
        }
        for (int k{ineqStart}; k <= ixlast; k++) {
          b_TrialState.delta_gradLag.data[k] -= b_TrialState.grad_old.data[k];
        }
      }
      BFGSUpdate(nVar_tmp_tmp, Hessian_data, Hessian_size,
                 b_TrialState.delta_x.data, b_TrialState.delta_gradLag.data,
                 memspace.workspace_float.data);
      b_TrialState.sqpIterations++;
    }
  }
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for driver.cpp
//
// [EOF]
//
