//
// File: fmincon.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "fmincon.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types1.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types2.h"
#include "RefTrjGnrtr_240503.h"
#include "anonymous_function.h"
#include "compressBounds.h"
#include "computeFiniteDifferences.h"
#include "driver.h"
#include "factoryConstruct.h"
#include "factoryConstruct1.h"
#include "factoryConstruct2.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "stickyStruct.h"
#include "xgemv.h"
#include "xnrm2.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const anonymous_function &fun
//                const double x0_data[]
//                const int x0_size[2]
//                const double Aineq_data[]
//                const double bineq_data[]
//                const int bineq_size[2]
//                const double lb_data[]
//                const int lb_size[2]
//                const double ub_data[]
//                const int ub_size[2]
//                double x_data[]
//                int x_size[2]
//                double &exitflag
//                double &output_iterations
//                double &output_funcCount
//                char output_algorithm[3]
//                double &output_constrviolation
//                double &output_stepsize
//                double &output_lssteplength
//                double &output_firstorderopt
// Return Type  : double
//
namespace coder {
double fmincon(const anonymous_function &fun, const double x0_data[],
               const int x0_size[2], const double Aineq_data[],
               const double bineq_data[], const int bineq_size[2],
               const double lb_data[], const int lb_size[2],
               const double ub_data[], const int ub_size[2], double x_data[],
               int x_size[2], double &exitflag, double &output_iterations,
               double &output_funcCount, char output_algorithm[3],
               double &output_constrviolation, double &output_stepsize,
               double &output_lssteplength, double &output_firstorderopt)
{
  static e_struct_T QRManager;
  internal::i_stickyStruct FcnEvaluator;
  f_struct_T CholManager;
  g_struct_T QPObjective;
  h_struct_T memspace;
  i_struct_T TrialState;
  j_struct_T WorkingSet;
  k_struct_T expl_temp;
  m_struct_T FiniteDifferences;
  struct_T MeritFunction;
  double Hessian_data[144];
  double y_data[25];
  double fval;
  int Hessian_size[2];
  int idxFillStart;
  int loop_ub;
  int mConstrMax;
  int mFixed;
  int mLinIneq_tmp;
  int mUB;
  int maxDims;
  int nVar;
  int nVarMax;
  boolean_T hasLB;
  exitflag = rtInf;
  hasLB = (lb_size[1] == 0);
  if ((!hasLB) && (ub_size[1] != 0)) {
    boolean_T exitg1;
    idxFillStart = 0;
    exitg1 = false;
    while ((!exitg1) && (idxFillStart <= x0_size[1] - 1)) {
      if (lb_data[idxFillStart] > ub_data[idxFillStart]) {
        exitflag = -2.0;
        exitg1 = true;
      } else {
        idxFillStart++;
      }
    }
  }
  loop_ub = x0_size[1];
  mLinIneq_tmp = bineq_size[1];
  nVar = x0_size[1] - 1;
  mConstrMax =
      (((bineq_size[1] + lb_size[1]) + ub_size[1]) + bineq_size[1]) + 1;
  nVarMax = (x0_size[1] + bineq_size[1]) + 1;
  if (nVarMax >= mConstrMax) {
    maxDims = nVarMax;
  } else {
    maxDims = mConstrMax;
  }
  Hessian_size[0] = x0_size[1];
  Hessian_size[1] = x0_size[1];
  idxFillStart = x0_size[1] * x0_size[1];
  if (idxFillStart - 1 >= 0) {
    std::memset(&Hessian_data[0], 0,
                static_cast<unsigned int>(idxFillStart) * sizeof(double));
  }
  if (x0_size[1] > 0) {
    for (idxFillStart = 0; idxFillStart <= nVar; idxFillStart++) {
      Hessian_data[idxFillStart + Hessian_size[0] * idxFillStart] = 1.0;
    }
  }
  if (exitflag == -2.0) {
    x_size[0] = 1;
    x_size[1] = x0_size[1];
    if (loop_ub - 1 >= 0) {
      std::copy(&x0_data[0], &x0_data[loop_ub], &x_data[0]);
    }
    fval = rtInf;
    output_iterations = 0.0;
    output_funcCount = 0.0;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    output_constrviolation = rtInf;
    output_stepsize = rtInf;
    output_lssteplength = rtInf;
    output_firstorderopt = rtInf;
  } else {
    double d;
    double normResid;
    int mLB;
    boolean_T hasUB;
    optim::coder::fminconsqp::TrialState::factoryConstruct(
        nVarMax, mConstrMax, bineq_size[1], x0_size, TrialState);
    if (nVar >= 0) {
      std::copy(&x0_data[0], &x0_data[nVar + 1], &TrialState.xstarsqp.data[0]);
    }
    FcnEvaluator.next.next.next.next.next.next.next.next.value = fun;
    optim::coder::utils::FiniteDifferences::factoryConstruct(
        fun, x0_size[1], lb_data, lb_size, ub_data, ub_size, FiniteDifferences);
    QRManager.ldq = maxDims;
    QRManager.QR.size[0] = maxDims;
    QRManager.QR.size[1] = maxDims;
    QRManager.Q.size[0] = maxDims;
    QRManager.Q.size[1] = maxDims;
    loop_ub = maxDims * maxDims;
    std::memset(&QRManager.Q.data[0], 0,
                static_cast<unsigned int>(loop_ub) * sizeof(double));
    QRManager.jpvt.size[0] = maxDims;
    std::memset(&QRManager.jpvt.data[0], 0,
                static_cast<unsigned int>(maxDims) * sizeof(int));
    QRManager.mrows = 0;
    QRManager.ncols = 0;
    QRManager.tau.size[0] = maxDims;
    QRManager.minRowCol = 0;
    QRManager.usedPivoting = false;
    CholManager.FMat.size[0] = maxDims;
    CholManager.FMat.size[1] = maxDims;
    CholManager.ldm = maxDims;
    CholManager.ndims = 0;
    CholManager.info = 0;
    CholManager.scaleFactor = 0.0;
    CholManager.ConvexCheck = true;
    CholManager.regTol_ = rtInf;
    CholManager.workspace_ = rtInf;
    CholManager.workspace2_ = rtInf;
    QPObjective.grad.size[0] = nVarMax;
    QPObjective.Hx.size[0] = nVarMax - 1;
    QPObjective.maxVar = nVarMax;
    QPObjective.beta = 0.0;
    QPObjective.rho = 0.0;
    QPObjective.prev_objtype = 3;
    QPObjective.prev_nvar = 0;
    QPObjective.prev_hasLinear = false;
    QPObjective.gammaScalar = 0.0;
    QPObjective.nvar = x0_size[1];
    QPObjective.hasLinear = true;
    QPObjective.objtype = 3;
    memspace.workspace_float.size[0] = maxDims;
    if (nVarMax >= 2) {
      idxFillStart = nVarMax;
    } else {
      idxFillStart = 2;
    }
    memspace.workspace_float.size[1] = idxFillStart;
    memspace.workspace_int.size[0] = maxDims;
    memspace.workspace_sort.size[0] = maxDims;
    optim::coder::qpactiveset::WorkingSet::factoryConstruct(
        bineq_size[1], x0_size[1], nVarMax, mConstrMax, WorkingSet);
    mLB = optim::coder::qpactiveset::initialize::compressBounds(
        x0_size[1], WorkingSet.indexLB.data, WorkingSet.indexUB.data,
        WorkingSet.indexFixed.data, lb_data, lb_size, ub_data, ub_size, mUB,
        mFixed);
    nVar = bineq_size[1] + mLB;
    idxFillStart = (nVar + mUB) + mFixed;
    WorkingSet.mConstr = idxFillStart;
    WorkingSet.mConstrOrig = idxFillStart;
    WorkingSet.mConstrMax = mConstrMax;
    WorkingSet.sizes[0] = mFixed;
    WorkingSet.sizes[1] = 0;
    WorkingSet.sizes[2] = bineq_size[1];
    WorkingSet.sizes[3] = mLB;
    WorkingSet.sizes[4] = mUB;
    WorkingSet.sizesPhaseOne[0] = mFixed;
    WorkingSet.sizesPhaseOne[1] = 0;
    WorkingSet.sizesPhaseOne[2] = bineq_size[1];
    WorkingSet.sizesPhaseOne[3] = mLB + 1;
    WorkingSet.sizesPhaseOne[4] = mUB;
    WorkingSet.sizesRegularized[0] = mFixed;
    WorkingSet.sizesRegularized[1] = 0;
    WorkingSet.sizesRegularized[2] = bineq_size[1];
    WorkingSet.sizesRegularized[3] = nVar;
    WorkingSet.sizesRegularized[4] = mUB;
    WorkingSet.sizesRegPhaseOne[0] = mFixed;
    WorkingSet.sizesRegPhaseOne[1] = 0;
    WorkingSet.sizesRegPhaseOne[2] = bineq_size[1];
    WorkingSet.sizesRegPhaseOne[3] = nVar + 1;
    WorkingSet.sizesRegPhaseOne[4] = mUB;
    WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
    WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
    WorkingSet.isActiveIdxRegPhaseOne[2] = 0;
    WorkingSet.isActiveIdxRegPhaseOne[3] = bineq_size[1];
    WorkingSet.isActiveIdxRegPhaseOne[4] = mLB;
    WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
    for (idxFillStart = 0; idxFillStart < 5; idxFillStart++) {
      WorkingSet.sizesNormal[idxFillStart] = WorkingSet.sizes[idxFillStart];
      WorkingSet.isActiveIdxRegPhaseOne[idxFillStart + 1] +=
          WorkingSet.isActiveIdxRegPhaseOne[idxFillStart];
    }
    for (mConstrMax = 0; mConstrMax < 6; mConstrMax++) {
      WorkingSet.isActiveIdx[mConstrMax] =
          WorkingSet.isActiveIdxRegPhaseOne[mConstrMax];
      WorkingSet.isActiveIdxNormal[mConstrMax] =
          WorkingSet.isActiveIdxRegPhaseOne[mConstrMax];
    }
    WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
    WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
    WorkingSet.isActiveIdxRegPhaseOne[2] = 0;
    WorkingSet.isActiveIdxRegPhaseOne[3] = bineq_size[1];
    WorkingSet.isActiveIdxRegPhaseOne[4] = mLB + 1;
    WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
    for (idxFillStart = 0; idxFillStart < 5; idxFillStart++) {
      WorkingSet.isActiveIdxRegPhaseOne[idxFillStart + 1] +=
          WorkingSet.isActiveIdxRegPhaseOne[idxFillStart];
    }
    for (mConstrMax = 0; mConstrMax < 6; mConstrMax++) {
      WorkingSet.isActiveIdxPhaseOne[mConstrMax] =
          WorkingSet.isActiveIdxRegPhaseOne[mConstrMax];
    }
    WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
    WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
    WorkingSet.isActiveIdxRegPhaseOne[2] = 0;
    WorkingSet.isActiveIdxRegPhaseOne[3] = bineq_size[1];
    WorkingSet.isActiveIdxRegPhaseOne[4] = nVar;
    WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
    for (idxFillStart = 0; idxFillStart < 5; idxFillStart++) {
      WorkingSet.isActiveIdxRegPhaseOne[idxFillStart + 1] +=
          WorkingSet.isActiveIdxRegPhaseOne[idxFillStart];
    }
    for (mConstrMax = 0; mConstrMax < 6; mConstrMax++) {
      WorkingSet.isActiveIdxRegularized[mConstrMax] =
          WorkingSet.isActiveIdxRegPhaseOne[mConstrMax];
    }
    WorkingSet.isActiveIdxRegPhaseOne[0] = 1;
    WorkingSet.isActiveIdxRegPhaseOne[1] = mFixed;
    WorkingSet.isActiveIdxRegPhaseOne[2] = 0;
    WorkingSet.isActiveIdxRegPhaseOne[3] = bineq_size[1];
    WorkingSet.isActiveIdxRegPhaseOne[4] = nVar + 1;
    WorkingSet.isActiveIdxRegPhaseOne[5] = mUB;
    for (idxFillStart = 0; idxFillStart < 5; idxFillStart++) {
      WorkingSet.isActiveIdxRegPhaseOne[idxFillStart + 1] +=
          WorkingSet.isActiveIdxRegPhaseOne[idxFillStart];
    }
    if (bineq_size[1] > 0) {
      mConstrMax = static_cast<unsigned char>(WorkingSet.nVar);
      for (idxFillStart = 0; idxFillStart < mLinIneq_tmp; idxFillStart++) {
        for (nVar = 0; nVar < mConstrMax; nVar++) {
          WorkingSet.Aineq.data[nVar + WorkingSet.ldA * idxFillStart] =
              Aineq_data[idxFillStart + bineq_size[1] * nVar];
        }
      }
    }
    if (lb_size[1] != 0) {
      mConstrMax = static_cast<unsigned char>(mLB);
      for (maxDims = 0; maxDims < mConstrMax; maxDims++) {
        nVarMax = WorkingSet.indexLB.data[maxDims];
        TrialState.xstarsqp.data[nVarMax - 1] = std::fmax(
            TrialState.xstarsqp.data[nVarMax - 1], lb_data[nVarMax - 1]);
      }
    }
    if (ub_size[1] != 0) {
      mConstrMax = static_cast<unsigned char>(mUB);
      for (maxDims = 0; maxDims < mConstrMax; maxDims++) {
        nVarMax = WorkingSet.indexUB.data[maxDims];
        TrialState.xstarsqp.data[nVarMax - 1] = std::fmin(
            TrialState.xstarsqp.data[nVarMax - 1], ub_data[nVarMax - 1]);
      }
      mConstrMax = static_cast<unsigned char>(mFixed);
      for (maxDims = 0; maxDims < mConstrMax; maxDims++) {
        nVarMax = WorkingSet.indexFixed.data[maxDims];
        TrialState.xstarsqp.data[nVarMax - 1] = ub_data[nVarMax - 1];
      }
    }
    fval = OptEntTimeSearch_v2_anonFcn1(
        fun.workspace.si_arr.data, fun.workspace.si_arr.size, fun.workspace.t0s,
        fun.workspace.tf0s, fun.workspace.vfs, fun.workspace.v0s,
        fun.workspace.ViolInfo.idxTLStp.data,
        fun.workspace.ViolInfo.idxTLStp.size, fun.workspace.ViolInfo.tint.data,
        fun.workspace.ViolInfo.tint.size, TrialState.xstarsqp.data,
        TrialState.xstarsqp.size);
    TrialState.sqpFval = fval;
    optim::coder::utils::FiniteDifferences::computeFiniteDifferences(
        FiniteDifferences, fval, TrialState.xstarsqp.data,
        TrialState.xstarsqp.size, TrialState.grad.data, lb_data, ub_data);
    TrialState.FunctionEvaluations = FiniteDifferences.numEvals + 1;
    if (bineq_size[1] > 0) {
      loop_ub = TrialState.cIneq.size[0];
      if (loop_ub - 1 >= 0) {
        std::copy(&TrialState.cIneq.data[0], &TrialState.cIneq.data[loop_ub],
                  &y_data[0]);
      }
      std::copy(&bineq_data[0], &bineq_data[mLinIneq_tmp], &y_data[0]);
      if (loop_ub - 1 >= 0) {
        std::copy(&y_data[0], &y_data[loop_ub], &TrialState.cIneq.data[0]);
      }
      internal::blas::xgemv(x0_size[1], bineq_size[1], WorkingSet.Aineq.data,
                            WorkingSet.ldA, TrialState.xstarsqp.data,
                            TrialState.cIneq.data);
    }
    idxFillStart = (bineq_size[1] / 2) << 1;
    nVar = idxFillStart - 2;
    for (maxDims = 0; maxDims <= nVar; maxDims += 2) {
      __m128d r;
      r = _mm_loadu_pd(&TrialState.cIneq.data[maxDims]);
      _mm_storeu_pd(&WorkingSet.bineq.data[maxDims],
                    _mm_mul_pd(r, _mm_set1_pd(-1.0)));
    }
    for (maxDims = idxFillStart; maxDims < mLinIneq_tmp; maxDims++) {
      WorkingSet.bineq.data[maxDims] = -TrialState.cIneq.data[maxDims];
    }
    hasLB = (lb_size[1] != 0);
    hasUB = (ub_size[1] != 0);
    if (hasLB) {
      mConstrMax = static_cast<unsigned char>(mLB);
      for (maxDims = 0; maxDims < mConstrMax; maxDims++) {
        WorkingSet.lb.data[WorkingSet.indexLB.data[maxDims] - 1] =
            -lb_data[WorkingSet.indexLB.data[maxDims] - 1] +
            x0_data[WorkingSet.indexLB.data[maxDims] - 1];
      }
    }
    if (hasUB) {
      mConstrMax = static_cast<unsigned char>(mUB);
      for (maxDims = 0; maxDims < mConstrMax; maxDims++) {
        WorkingSet.ub.data[WorkingSet.indexUB.data[maxDims] - 1] =
            ub_data[WorkingSet.indexUB.data[maxDims] - 1] -
            x0_data[WorkingSet.indexUB.data[maxDims] - 1];
      }
    }
    if (hasLB && hasUB) {
      mConstrMax = static_cast<unsigned char>(mFixed);
      for (maxDims = 0; maxDims < mConstrMax; maxDims++) {
        d = ub_data[WorkingSet.indexFixed.data[maxDims] - 1] -
            x0_data[WorkingSet.indexFixed.data[maxDims] - 1];
        WorkingSet.ub.data[WorkingSet.indexFixed.data[maxDims] - 1] = d;
        WorkingSet.bwset.data[maxDims] = d;
      }
    }
    optim::coder::qpactiveset::WorkingSet::setProblemType(WorkingSet, 3);
    idxFillStart = WorkingSet.isActiveIdx[2];
    mConstrMax = WorkingSet.mConstrMax;
    if (idxFillStart <= mConstrMax) {
      std::memset(&WorkingSet.isActiveConstr.data[idxFillStart + -1], 0,
                  static_cast<unsigned int>((mConstrMax - idxFillStart) + 1) *
                      sizeof(boolean_T));
    }
    WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
    WorkingSet.nWConstr[1] = 0;
    WorkingSet.nWConstr[2] = 0;
    WorkingSet.nWConstr[3] = 0;
    WorkingSet.nWConstr[4] = 0;
    WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
    mConstrMax = static_cast<unsigned char>(WorkingSet.sizes[0]);
    for (maxDims = 0; maxDims < mConstrMax; maxDims++) {
      WorkingSet.Wid.data[maxDims] = 1;
      WorkingSet.Wlocalidx.data[maxDims] = maxDims + 1;
      WorkingSet.isActiveConstr.data[maxDims] = true;
      idxFillStart = WorkingSet.ldA * maxDims;
      nVarMax =
          static_cast<unsigned char>(WorkingSet.indexFixed.data[maxDims] - 1);
      if (nVarMax - 1 >= 0) {
        std::memset(&WorkingSet.ATwset.data[idxFillStart], 0,
                    static_cast<unsigned int>(nVarMax) * sizeof(double));
      }
      WorkingSet.ATwset
          .data[(WorkingSet.indexFixed.data[maxDims] + idxFillStart) - 1] = 1.0;
      nVarMax = WorkingSet.indexFixed.data[maxDims] + 1;
      nVar = WorkingSet.nVar;
      if (nVarMax <= nVar) {
        std::memset(
            &WorkingSet.ATwset.data[(nVarMax + idxFillStart) + -1], 0,
            static_cast<unsigned int>(
                (((nVar + idxFillStart) - nVarMax) - idxFillStart) + 1) *
                sizeof(double));
      }
      WorkingSet.bwset.data[maxDims] =
          WorkingSet.ub.data[WorkingSet.indexFixed.data[maxDims] - 1];
    }
    MeritFunction.penaltyParam = 1.0;
    MeritFunction.threshold = 0.0001;
    MeritFunction.nPenaltyDecreases = 0;
    MeritFunction.linearizedConstrViol = 0.0;
    MeritFunction.initFval = fval;
    MeritFunction.initConstrViolationEq = 0.0;
    normResid = 0.0;
    for (maxDims = 0; maxDims < mLinIneq_tmp; maxDims++) {
      d = TrialState.cIneq.data[maxDims];
      if (d > 0.0) {
        normResid += d;
      }
    }
    MeritFunction.initConstrViolationIneq = normResid;
    MeritFunction.phi = 0.0;
    MeritFunction.phiPrimePlus = 0.0;
    MeritFunction.phiFullStep = 0.0;
    MeritFunction.feasRelativeFactor = 0.0;
    MeritFunction.nlpPrimalFeasError = 0.0;
    MeritFunction.nlpDualFeasError = 0.0;
    MeritFunction.nlpComplError = 0.0;
    MeritFunction.firstOrderOpt = 0.0;
    MeritFunction.hasObjective = true;
    expl_temp.MaxFunctionEvaluations = 100 * x0_size[1];
    optim::coder::fminconsqp::driver(
        Hessian_data, Hessian_size, bineq_data, lb_data, lb_size, ub_data,
        ub_size, TrialState, MeritFunction, FcnEvaluator, FiniteDifferences,
        memspace, WorkingSet, QRManager, CholManager, QPObjective,
        bineq_size[1], expl_temp);
    x_size[0] = 1;
    loop_ub = TrialState.xstarsqp.size[1];
    x_size[1] = TrialState.xstarsqp.size[1];
    if (loop_ub - 1 >= 0) {
      std::copy(&TrialState.xstarsqp.data[0],
                &TrialState.xstarsqp.data[loop_ub], &x_data[0]);
    }
    fval = TrialState.sqpFval;
    exitflag = TrialState.sqpExitFlag;
    output_iterations = TrialState.sqpIterations;
    output_funcCount = TrialState.FunctionEvaluations;
    output_algorithm[0] = 's';
    output_algorithm[1] = 'q';
    output_algorithm[2] = 'p';
    output_constrviolation = MeritFunction.nlpPrimalFeasError;
    output_stepsize =
        internal::blas::xnrm2(x0_size[1], TrialState.delta_x.data);
    output_lssteplength = TrialState.steplength;
    output_firstorderopt = MeritFunction.firstOrderOpt;
  }
  return fval;
}

} // namespace coder

//
// File trailer for fmincon.cpp
//
// [EOF]
//
