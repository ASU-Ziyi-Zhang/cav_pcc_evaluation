//
// File: relaxed.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "relaxed.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "driver1.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sortLambdaQP.h"
#include "xgemv.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double Hessian_data[]
//                const int Hessian_size[2]
//                const double grad_data[]
//                i_struct_T &b_TrialState
//                struct_T &MeritFunction
//                h_struct_T &memspace
//                j_struct_T &WorkingSet
//                e_struct_T &b_QRManager
//                f_struct_T &b_CholManager
//                g_struct_T &QPObjective
//                l_struct_T &qpoptions
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace step {
void relaxed(const double Hessian_data[], const int Hessian_size[2],
             const double grad_data[], i_struct_T &b_TrialState,
             struct_T &MeritFunction, h_struct_T &memspace,
             j_struct_T &WorkingSet, e_struct_T &b_QRManager,
             f_struct_T &b_CholManager, g_struct_T &QPObjective,
             l_struct_T &qpoptions)
{
  double beta;
  double d;
  double s;
  double smax;
  int idx;
  int idx_max;
  int ix0_tmp;
  int mIneq;
  int nActiveLBArtificial;
  int nVarOrig;
  int temp;
  nVarOrig = WorkingSet.nVar;
  mIneq = WorkingSet.sizes[2];
  beta = 0.0;
  temp = static_cast<unsigned char>(WorkingSet.nVar);
  for (idx = 0; idx < temp; idx++) {
    beta += Hessian_data[idx + Hessian_size[0] * idx];
  }
  beta /= static_cast<double>(WorkingSet.nVar);
  if (b_TrialState.sqpIterations <= 1) {
    temp = QPObjective.nvar;
    if (QPObjective.nvar < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (QPObjective.nvar > 1) {
        smax = std::abs(grad_data[0]);
        for (idx = 2; idx <= temp; idx++) {
          s = std::abs(grad_data[idx - 1]);
          if (s > smax) {
            idx_max = idx;
            smax = s;
          }
        }
      }
    }
    smax = 100.0 * std::fmax(1.0, std::abs(grad_data[idx_max - 1]));
  } else {
    temp = WorkingSet.mConstr;
    if (WorkingSet.mConstr < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (WorkingSet.mConstr > 1) {
        smax = std::abs(b_TrialState.lambdasqp.data[0]);
        for (idx = 2; idx <= temp; idx++) {
          s = std::abs(b_TrialState.lambdasqp.data[idx - 1]);
          if (s > smax) {
            idx_max = idx;
            smax = s;
          }
        }
      }
    }
    smax = std::abs(b_TrialState.lambdasqp.data[idx_max - 1]);
  }
  QPObjective.nvar = WorkingSet.nVar;
  QPObjective.beta = beta;
  QPObjective.rho = smax;
  QPObjective.hasLinear = true;
  QPObjective.objtype = 4;
  qpactiveset::WorkingSet::setProblemType(WorkingSet, 2);
  temp = static_cast<unsigned char>(WorkingSet.sizes[2]);
  if (temp - 1 >= 0) {
    std::copy(&WorkingSet.bineq.data[0], &WorkingSet.bineq.data[temp],
              &memspace.workspace_float.data[0]);
  }
  ::coder::internal::blas::b_xgemv(
      nVarOrig, WorkingSet.sizes[2], WorkingSet.Aineq.data, WorkingSet.ldA,
      b_TrialState.xstar.data, memspace.workspace_float.data);
  for (idx = 0; idx < temp; idx++) {
    d = memspace.workspace_float.data[idx];
    b_TrialState.xstar.data[nVarOrig + idx] = static_cast<double>(d > 0.0) * d;
  }
  l_struct_T b_qpoptions;
  ::coder::internal::blas::b_xgemv(nVarOrig, 0, nullptr, WorkingSet.ldA,
                                   b_TrialState.xstar.data,
                                   memspace.workspace_float.data);
  temp = qpoptions.MaxIterations;
  qpoptions.MaxIterations =
      (qpoptions.MaxIterations + WorkingSet.nVar) - nVarOrig;
  b_qpoptions = qpoptions;
  ::coder::optim::coder::qpactiveset::driver(
      Hessian_data, Hessian_size, grad_data, b_TrialState, memspace, WorkingSet,
      b_QRManager, b_CholManager, QPObjective, b_qpoptions,
      qpoptions.MaxIterations);
  qpoptions.MaxIterations = temp;
  nActiveLBArtificial = 0;
  temp = static_cast<unsigned char>(WorkingSet.sizes[2]);
  for (idx = 0; idx < temp; idx++) {
    boolean_T tf;
    tf = WorkingSet.isActiveConstr
             .data[(((WorkingSet.isActiveIdx[3] + WorkingSet.sizes[3]) -
                     WorkingSet.sizes[2]) +
                    idx) -
                   1];
    memspace.workspace_int.data[idx] = tf;
    nActiveLBArtificial += tf;
  }
  if (b_TrialState.state != -6) {
    double constrViolationIneq;
    double qpfvalQuadExcess;
    idx_max = (WorkingSet.nVarMax - nVarOrig) - 1;
    ix0_tmp = nVarOrig + 1;
    s = 0.0;
    qpfvalQuadExcess = 0.0;
    if (idx_max >= 1) {
      temp = nVarOrig + idx_max;
      for (idx = ix0_tmp; idx <= temp; idx++) {
        s += std::abs(b_TrialState.xstar.data[idx - 1]);
      }
    }
    if (idx_max >= 1) {
      temp = static_cast<unsigned char>(idx_max);
      for (idx = 0; idx < temp; idx++) {
        d = b_TrialState.xstar.data[nVarOrig + idx];
        qpfvalQuadExcess += d * d;
      }
    }
    qpfvalQuadExcess =
        (b_TrialState.fstar - smax * s) - beta / 2.0 * qpfvalQuadExcess;
    beta = MeritFunction.penaltyParam;
    constrViolationIneq = 0.0;
    temp = static_cast<unsigned char>(mIneq);
    for (idx = 0; idx < temp; idx++) {
      d = b_TrialState.cIneq.data[idx];
      if (d > 0.0) {
        constrViolationIneq += d;
      }
    }
    smax = MeritFunction.linearizedConstrViol;
    s = 0.0;
    if (idx_max >= 1) {
      temp = nVarOrig + idx_max;
      for (idx = ix0_tmp; idx <= temp; idx++) {
        s += std::abs(b_TrialState.xstar.data[idx - 1]);
      }
    }
    MeritFunction.linearizedConstrViol = s;
    smax = (constrViolationIneq + smax) - s;
    if ((smax > 2.2204460492503131E-16) && (qpfvalQuadExcess > 0.0)) {
      if (b_TrialState.sqpFval == 0.0) {
        d = 1.0;
      } else {
        d = 1.5;
      }
      beta = d * qpfvalQuadExcess / smax;
    }
    if (beta < MeritFunction.penaltyParam) {
      MeritFunction.phi = b_TrialState.sqpFval + beta * constrViolationIneq;
      if ((MeritFunction.initFval +
           beta * MeritFunction.initConstrViolationIneq) -
              MeritFunction.phi >
          static_cast<double>(MeritFunction.nPenaltyDecreases) *
              MeritFunction.threshold) {
        MeritFunction.nPenaltyDecreases++;
        if ((MeritFunction.nPenaltyDecreases << 1) >
            b_TrialState.sqpIterations) {
          MeritFunction.threshold *= 10.0;
        }
        MeritFunction.penaltyParam = std::fmax(beta, 1.0E-10);
      } else {
        MeritFunction.phi = b_TrialState.sqpFval +
                            MeritFunction.penaltyParam * constrViolationIneq;
      }
    } else {
      MeritFunction.penaltyParam = std::fmax(beta, 1.0E-10);
      MeritFunction.phi = b_TrialState.sqpFval +
                          MeritFunction.penaltyParam * constrViolationIneq;
    }
    MeritFunction.phiPrimePlus = std::fmin(
        qpfvalQuadExcess - MeritFunction.penaltyParam * constrViolationIneq,
        0.0);
    temp = WorkingSet.isActiveIdx[2];
    idx_max = WorkingSet.nActiveConstr;
    for (idx = temp; idx <= idx_max; idx++) {
      if (WorkingSet.Wid.data[idx - 1] == 3) {
        b_TrialState.lambda.data[idx - 1] *= static_cast<double>(
            memspace.workspace_int
                .data[WorkingSet.Wlocalidx.data[idx - 1] - 1]);
      }
    }
  }
  temp = WorkingSet.sizes[0];
  idx_max = WorkingSet.sizes[3] - WorkingSet.sizes[2];
  idx = WorkingSet.nActiveConstr;
  while ((idx > temp) && (nActiveLBArtificial > 0)) {
    if ((WorkingSet.Wid.data[idx - 1] == 4) &&
        (WorkingSet.Wlocalidx.data[idx - 1] > idx_max)) {
      ix0_tmp = WorkingSet.nActiveConstr - 1;
      smax = b_TrialState.lambda.data[ix0_tmp];
      b_TrialState.lambda.data[ix0_tmp] = 0.0;
      b_TrialState.lambda.data[idx - 1] = smax;
      qpactiveset::WorkingSet::removeConstr(WorkingSet, idx);
      nActiveLBArtificial--;
    }
    idx--;
  }
  QPObjective.nvar = nVarOrig;
  QPObjective.hasLinear = true;
  QPObjective.objtype = 3;
  qpactiveset::WorkingSet::setProblemType(WorkingSet, 3);
  qpactiveset::parseoutput::sortLambdaQP(
      b_TrialState.lambda.data, WorkingSet.nActiveConstr, WorkingSet.sizes,
      WorkingSet.isActiveIdx, WorkingSet.Wid.data, WorkingSet.Wlocalidx.data,
      memspace.workspace_float.data);
}

} // namespace step
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for relaxed.cpp
//
// [EOF]
//
