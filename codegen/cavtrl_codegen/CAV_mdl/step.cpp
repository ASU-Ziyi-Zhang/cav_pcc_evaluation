//
// File: step.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "step.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "driver1.h"
#include "relaxed.h"
#include "rt_nonfinite.h"
#include "soc.h"
#include "sortLambdaQP.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : int &STEP_TYPE
//                double Hessian_data[]
//                const int Hessian_size[2]
//                const double lb_data[]
//                const int lb_size[2]
//                const double ub_data[]
//                const int ub_size[2]
//                i_struct_T &b_TrialState
//                struct_T &MeritFunction
//                h_struct_T &memspace
//                j_struct_T &WorkingSet
//                e_struct_T &b_QRManager
//                f_struct_T &b_CholManager
//                g_struct_T &QPObjective
//                l_struct_T &qpoptions
// Return Type  : boolean_T
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
boolean_T b_step(int &STEP_TYPE, double Hessian_data[],
                 const int Hessian_size[2], const double lb_data[],
                 const int lb_size[2], const double ub_data[],
                 const int ub_size[2], i_struct_T &b_TrialState,
                 struct_T &MeritFunction, h_struct_T &memspace,
                 j_struct_T &WorkingSet, e_struct_T &b_QRManager,
                 f_struct_T &b_CholManager, g_struct_T &QPObjective,
                 l_struct_T &qpoptions)
{
  l_struct_T b_qpoptions;
  double y_data[49];
  double tmp_data[25];
  double constrViolDelta;
  double linearizedConstrViolPrev;
  int iH0;
  int idxEndIneq;
  int idxStartIneq;
  int nVar;
  boolean_T checkBoundViolation;
  boolean_T stepSuccess;
  stepSuccess = true;
  checkBoundViolation = true;
  nVar = WorkingSet.nVar;
  if (STEP_TYPE != 3) {
    idxEndIneq = static_cast<unsigned char>(WorkingSet.nVar);
    if (idxEndIneq - 1 >= 0) {
      std::copy(&b_TrialState.xstarsqp.data[0],
                &b_TrialState.xstarsqp.data[idxEndIneq],
                &b_TrialState.xstar.data[0]);
    }
  } else {
    idxStartIneq = b_TrialState.searchDir.size[0];
    iH0 = b_TrialState.searchDir.size[0];
    if (iH0 - 1 >= 0) {
      std::copy(&b_TrialState.searchDir.data[0],
                &b_TrialState.searchDir.data[iH0], &y_data[0]);
    }
    idxEndIneq = static_cast<unsigned char>(WorkingSet.nVar);
    if (idxEndIneq - 1 >= 0) {
      std::copy(&b_TrialState.xstar.data[0],
                &b_TrialState.xstar.data[idxEndIneq], &y_data[0]);
    }
    if (idxStartIneq - 1 >= 0) {
      std::copy(&y_data[0], &y_data[idxStartIneq],
                &b_TrialState.searchDir.data[0]);
    }
  }
  int exitg1;
  boolean_T guard1;
  do {
    int b_nVar;
    exitg1 = 0;
    guard1 = false;
    switch (STEP_TYPE) {
    case 1: {
      b_qpoptions = qpoptions;
      ::coder::optim::coder::qpactiveset::driver(
          Hessian_data, Hessian_size, b_TrialState.grad.data, b_TrialState,
          memspace, WorkingSet, b_QRManager, b_CholManager, QPObjective,
          b_qpoptions, qpoptions.MaxIterations);
      if (b_TrialState.state > 0) {
        double constrViolationIneq;
        double penaltyParamTrial;
        penaltyParamTrial = MeritFunction.penaltyParam;
        constrViolationIneq = 0.0;
        idxEndIneq = static_cast<unsigned char>(WorkingSet.sizes[2]);
        for (int idx{0}; idx < idxEndIneq; idx++) {
          linearizedConstrViolPrev = b_TrialState.cIneq.data[idx];
          if (linearizedConstrViolPrev > 0.0) {
            constrViolationIneq += linearizedConstrViolPrev;
          }
        }
        linearizedConstrViolPrev = MeritFunction.linearizedConstrViol;
        MeritFunction.linearizedConstrViol = 0.0;
        constrViolDelta = constrViolationIneq + linearizedConstrViolPrev;
        if ((constrViolDelta > 2.2204460492503131E-16) &&
            (b_TrialState.fstar > 0.0)) {
          if (b_TrialState.sqpFval == 0.0) {
            penaltyParamTrial = 1.0;
          } else {
            penaltyParamTrial = 1.5;
          }
          penaltyParamTrial =
              penaltyParamTrial * b_TrialState.fstar / constrViolDelta;
        }
        if (penaltyParamTrial < MeritFunction.penaltyParam) {
          MeritFunction.phi =
              b_TrialState.sqpFval + penaltyParamTrial * constrViolationIneq;
          if ((MeritFunction.initFval +
               penaltyParamTrial * MeritFunction.initConstrViolationIneq) -
                  MeritFunction.phi >
              static_cast<double>(MeritFunction.nPenaltyDecreases) *
                  MeritFunction.threshold) {
            MeritFunction.nPenaltyDecreases++;
            if ((MeritFunction.nPenaltyDecreases << 1) >
                b_TrialState.sqpIterations) {
              MeritFunction.threshold *= 10.0;
            }
            MeritFunction.penaltyParam = std::fmax(penaltyParamTrial, 1.0E-10);
          } else {
            MeritFunction.phi =
                b_TrialState.sqpFval +
                MeritFunction.penaltyParam * constrViolationIneq;
          }
        } else {
          MeritFunction.penaltyParam = std::fmax(penaltyParamTrial, 1.0E-10);
          MeritFunction.phi = b_TrialState.sqpFval +
                              MeritFunction.penaltyParam * constrViolationIneq;
        }
        MeritFunction.phiPrimePlus =
            std::fmin(b_TrialState.fstar -
                          MeritFunction.penaltyParam * constrViolationIneq,
                      0.0);
      }
      qpactiveset::parseoutput::sortLambdaQP(
          b_TrialState.lambda.data, WorkingSet.nActiveConstr, WorkingSet.sizes,
          WorkingSet.isActiveIdx, WorkingSet.Wid.data,
          WorkingSet.Wlocalidx.data, memspace.workspace_float.data);
      if ((b_TrialState.state <= 0) && (b_TrialState.state != -6)) {
        STEP_TYPE = 2;
      } else {
        idxStartIneq = b_TrialState.delta_x.size[0];
        iH0 = b_TrialState.delta_x.size[0];
        if (iH0 - 1 >= 0) {
          std::copy(&b_TrialState.delta_x.data[0],
                    &b_TrialState.delta_x.data[iH0], &y_data[0]);
        }
        idxEndIneq = static_cast<unsigned char>(nVar);
        if (idxEndIneq - 1 >= 0) {
          std::copy(&b_TrialState.xstar.data[0],
                    &b_TrialState.xstar.data[idxEndIneq], &y_data[0]);
        }
        if (idxStartIneq - 1 >= 0) {
          std::copy(&y_data[0], &y_data[idxStartIneq],
                    &b_TrialState.delta_x.data[0]);
        }
        guard1 = true;
      }
    } break;
    case 2:
      iH0 = WorkingSet.nWConstr[0] + WorkingSet.nWConstr[1];
      idxStartIneq = iH0 + 1;
      idxEndIneq = WorkingSet.nActiveConstr;
      for (b_nVar = idxStartIneq; b_nVar <= idxEndIneq; b_nVar++) {
        WorkingSet.isActiveConstr
            .data[(WorkingSet.isActiveIdx[WorkingSet.Wid.data[b_nVar - 1] - 1] +
                   WorkingSet.Wlocalidx.data[b_nVar - 1]) -
                  2] = false;
      }
      WorkingSet.nWConstr[2] = 0;
      WorkingSet.nWConstr[3] = 0;
      WorkingSet.nWConstr[4] = 0;
      WorkingSet.nActiveConstr = iH0;
      idxStartIneq = b_TrialState.xstar.size[0];
      iH0 = b_TrialState.xstar.size[0];
      if (iH0 - 1 >= 0) {
        std::copy(&b_TrialState.xstar.data[0], &b_TrialState.xstar.data[iH0],
                  &tmp_data[0]);
      }
      if (lb_size[1] != 0) {
        if (ub_size[1] == 0) {
          idxEndIneq = static_cast<unsigned char>(WorkingSet.sizes[3]);
          for (int idx{0}; idx < idxEndIneq; idx++) {
            linearizedConstrViolPrev =
                WorkingSet.lb.data[WorkingSet.indexLB.data[idx] - 1];
            if (-tmp_data[WorkingSet.indexLB.data[idx] - 1] >
                linearizedConstrViolPrev) {
              tmp_data[WorkingSet.indexLB.data[idx] - 1] =
                  -linearizedConstrViolPrev +
                  std::abs(linearizedConstrViolPrev);
            }
          }
        } else {
          idxEndIneq = static_cast<unsigned char>(WorkingSet.sizes[3]);
          for (int idx{0}; idx < idxEndIneq; idx++) {
            linearizedConstrViolPrev =
                WorkingSet.lb.data[WorkingSet.indexLB.data[idx] - 1];
            if (-tmp_data[WorkingSet.indexLB.data[idx] - 1] >
                linearizedConstrViolPrev) {
              if (std::isinf(ub_data[WorkingSet.indexLB.data[idx] - 1])) {
                tmp_data[WorkingSet.indexLB.data[idx] - 1] =
                    -linearizedConstrViolPrev +
                    std::abs(linearizedConstrViolPrev);
              } else {
                tmp_data[WorkingSet.indexLB.data[idx] - 1] =
                    (WorkingSet.ub.data[WorkingSet.indexLB.data[idx] - 1] -
                     linearizedConstrViolPrev) /
                    2.0;
              }
            }
          }
        }
      }
      if (ub_size[1] != 0) {
        if (lb_size[1] == 0) {
          idxEndIneq = static_cast<unsigned char>(WorkingSet.sizes[4]);
          for (int idx{0}; idx < idxEndIneq; idx++) {
            linearizedConstrViolPrev =
                WorkingSet.ub.data[WorkingSet.indexUB.data[idx] - 1];
            if (tmp_data[WorkingSet.indexUB.data[idx] - 1] >
                linearizedConstrViolPrev) {
              tmp_data[WorkingSet.indexUB.data[idx] - 1] =
                  linearizedConstrViolPrev - std::abs(linearizedConstrViolPrev);
            }
          }
        } else {
          idxEndIneq = static_cast<unsigned char>(WorkingSet.sizes[4]);
          for (int idx{0}; idx < idxEndIneq; idx++) {
            linearizedConstrViolPrev =
                WorkingSet.ub.data[WorkingSet.indexUB.data[idx] - 1];
            if (tmp_data[WorkingSet.indexUB.data[idx] - 1] >
                linearizedConstrViolPrev) {
              if (std::isinf(lb_data[WorkingSet.indexUB.data[idx] - 1])) {
                tmp_data[WorkingSet.indexUB.data[idx] - 1] =
                    linearizedConstrViolPrev -
                    std::abs(linearizedConstrViolPrev);
              } else {
                tmp_data[WorkingSet.indexUB.data[idx] - 1] =
                    (linearizedConstrViolPrev -
                     WorkingSet.lb.data[WorkingSet.indexUB.data[idx] - 1]) /
                    2.0;
              }
            }
          }
        }
      }
      if (idxStartIneq - 1 >= 0) {
        std::copy(&tmp_data[0], &tmp_data[idxStartIneq],
                  &b_TrialState.xstar.data[0]);
      }
      step::relaxed(Hessian_data, Hessian_size, b_TrialState.grad.data,
                    b_TrialState, MeritFunction, memspace, WorkingSet,
                    b_QRManager, b_CholManager, QPObjective, qpoptions);
      idxStartIneq = b_TrialState.delta_x.size[0];
      iH0 = b_TrialState.delta_x.size[0];
      if (iH0 - 1 >= 0) {
        std::copy(&b_TrialState.delta_x.data[0],
                  &b_TrialState.delta_x.data[iH0], &y_data[0]);
      }
      idxEndIneq = static_cast<unsigned char>(nVar);
      if (idxEndIneq - 1 >= 0) {
        std::copy(&b_TrialState.xstar.data[0],
                  &b_TrialState.xstar.data[idxEndIneq], &y_data[0]);
      }
      if (idxStartIneq - 1 >= 0) {
        std::copy(&y_data[0], &y_data[idxStartIneq],
                  &b_TrialState.delta_x.data[0]);
      }
      guard1 = true;
      break;
    default: {
      idxStartIneq = b_TrialState.grad.size[0];
      if (idxStartIneq - 1 >= 0) {
        std::copy(&b_TrialState.grad.data[0],
                  &b_TrialState.grad.data[idxStartIneq], &tmp_data[0]);
      }
      stepSuccess = step::soc(Hessian_data, Hessian_size, tmp_data,
                              b_TrialState, memspace, WorkingSet, b_QRManager,
                              b_CholManager, QPObjective, qpoptions);
      checkBoundViolation = stepSuccess;
      if (stepSuccess && (b_TrialState.state != -6)) {
        idxEndIneq = static_cast<unsigned char>(nVar);
        iH0 = (static_cast<unsigned char>(nVar) >> 1) << 1;
        idxStartIneq = iH0 - 2;
        for (int idx{0}; idx <= idxStartIneq; idx += 2) {
          __m128d r;
          __m128d r1;
          r = _mm_loadu_pd(&b_TrialState.xstar.data[idx]);
          r1 = _mm_loadu_pd(&b_TrialState.socDirection.data[idx]);
          _mm_storeu_pd(&b_TrialState.delta_x.data[idx], _mm_add_pd(r, r1));
        }
        for (int idx{iH0}; idx < idxEndIneq; idx++) {
          b_TrialState.delta_x.data[idx] = b_TrialState.xstar.data[idx] +
                                           b_TrialState.socDirection.data[idx];
        }
      }
      guard1 = true;
    } break;
    }
    if (guard1) {
      if (b_TrialState.state != -6) {
        exitg1 = 1;
      } else {
        b_nVar = Hessian_size[0] - 1;
        linearizedConstrViolPrev = 0.0;
        constrViolDelta = 1.0;
        for (int idx{0}; idx <= b_nVar; idx++) {
          linearizedConstrViolPrev = std::fmax(
              linearizedConstrViolPrev, std::abs(b_TrialState.grad.data[idx]));
          constrViolDelta = std::fmax(constrViolDelta,
                                      std::abs(b_TrialState.xstar.data[idx]));
        }
        linearizedConstrViolPrev = std::fmax(
            2.2204460492503131E-16, linearizedConstrViolPrev / constrViolDelta);
        for (int idx{0}; idx <= b_nVar; idx++) {
          iH0 = (b_nVar + 1) * idx;
          for (idxStartIneq = 0; idxStartIneq < idx; idxStartIneq++) {
            Hessian_data[iH0 + idxStartIneq] = 0.0;
          }
          Hessian_data[idx + Hessian_size[0] * idx] = linearizedConstrViolPrev;
          idxStartIneq = iH0 + idx;
          idxEndIneq = b_nVar - idx;
          if (idxEndIneq - 1 >= 0) {
            std::memset(&Hessian_data[idxStartIneq + 1], 0,
                        static_cast<unsigned int>((idxEndIneq + idxStartIneq) -
                                                  idxStartIneq) *
                            sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);
  if (checkBoundViolation) {
    idxStartIneq = b_TrialState.delta_x.size[0];
    iH0 = b_TrialState.delta_x.size[0];
    if (iH0 - 1 >= 0) {
      std::copy(&b_TrialState.delta_x.data[0], &b_TrialState.delta_x.data[iH0],
                &tmp_data[0]);
    }
    if (lb_size[1] != 0) {
      idxEndIneq = static_cast<unsigned char>(WorkingSet.sizes[3]);
      for (int idx{0}; idx < idxEndIneq; idx++) {
        linearizedConstrViolPrev = tmp_data[WorkingSet.indexLB.data[idx] - 1];
        constrViolDelta =
            (b_TrialState.xstarsqp.data[WorkingSet.indexLB.data[idx] - 1] +
             linearizedConstrViolPrev) -
            lb_data[WorkingSet.indexLB.data[idx] - 1];
        if (constrViolDelta < 0.0) {
          tmp_data[WorkingSet.indexLB.data[idx] - 1] =
              linearizedConstrViolPrev - constrViolDelta;
          b_TrialState.xstar.data[WorkingSet.indexLB.data[idx] - 1] -=
              constrViolDelta;
        }
      }
    }
    if (ub_size[1] != 0) {
      idxEndIneq = static_cast<unsigned char>(WorkingSet.sizes[4]);
      for (int idx{0}; idx < idxEndIneq; idx++) {
        linearizedConstrViolPrev = tmp_data[WorkingSet.indexUB.data[idx] - 1];
        constrViolDelta =
            (ub_data[WorkingSet.indexUB.data[idx] - 1] -
             b_TrialState.xstarsqp.data[WorkingSet.indexUB.data[idx] - 1]) -
            linearizedConstrViolPrev;
        if (constrViolDelta < 0.0) {
          tmp_data[WorkingSet.indexUB.data[idx] - 1] =
              linearizedConstrViolPrev + constrViolDelta;
          b_TrialState.xstar.data[WorkingSet.indexUB.data[idx] - 1] +=
              constrViolDelta;
        }
      }
    }
    if (idxStartIneq - 1 >= 0) {
      std::copy(&tmp_data[0], &tmp_data[idxStartIneq],
                &b_TrialState.delta_x.data[0]);
    }
  }
  return stepSuccess;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for step.cpp
//
// [EOF]
//
