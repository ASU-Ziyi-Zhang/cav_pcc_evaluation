//
// File: soc.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "soc.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "addBoundToActiveSetMatrix_.h"
#include "driver1.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xnrm2.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const double Hessian_data[]
//                const int Hessian_size[2]
//                const double grad_data[]
//                i_struct_T &b_TrialState
//                h_struct_T &memspace
//                j_struct_T &WorkingSet
//                e_struct_T &b_QRManager
//                f_struct_T &b_CholManager
//                g_struct_T &QPObjective
//                const l_struct_T &qpoptions
// Return Type  : boolean_T
//
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace step {
boolean_T soc(const double Hessian_data[], const int Hessian_size[2],
              const double grad_data[], i_struct_T &b_TrialState,
              h_struct_T &memspace, j_struct_T &WorkingSet,
              e_struct_T &b_QRManager, f_struct_T &b_CholManager,
              g_struct_T &QPObjective, const l_struct_T &qpoptions)
{
  __m128d r;
  l_struct_T b_qpoptions;
  double c;
  int i;
  int i1;
  int i2;
  int idxIneqOffset;
  int idx_Aineq;
  int idx_Partition;
  int idx_lower;
  int iy;
  int nVar;
  int nWIneq_old;
  int nWLower_old;
  int nWUpper_old;
  boolean_T success;
  nWIneq_old = WorkingSet.nWConstr[2];
  nWLower_old = WorkingSet.nWConstr[3];
  nWUpper_old = WorkingSet.nWConstr[4];
  nVar = WorkingSet.nVar;
  i = static_cast<unsigned char>(WorkingSet.nVar);
  for (iy = 0; iy < i; iy++) {
    b_TrialState.xstarsqp.data[iy] = b_TrialState.xstarsqp_old.data[iy];
    b_TrialState.socDirection.data[iy] = b_TrialState.xstar.data[iy];
  }
  i1 = static_cast<unsigned char>(WorkingSet.mConstrMax);
  if (i1 - 1 >= 0) {
    std::copy(&b_TrialState.lambda.data[0], &b_TrialState.lambda.data[i1],
              &b_TrialState.lambdaStopTest.data[0]);
  }
  idxIneqOffset = WorkingSet.isActiveIdx[2];
  if (WorkingSet.sizes[2] > 0) {
    i2 = static_cast<unsigned char>(WorkingSet.sizes[2]);
    idx_lower = (i2 / 2) << 1;
    idx_Aineq = idx_lower - 2;
    for (int idx{0}; idx <= idx_Aineq; idx += 2) {
      r = _mm_loadu_pd(&b_TrialState.cIneq.data[idx]);
      _mm_storeu_pd(&WorkingSet.bineq.data[idx],
                    _mm_mul_pd(r, _mm_set1_pd(-1.0)));
    }
    for (int idx{idx_lower}; idx < i2; idx++) {
      WorkingSet.bineq.data[idx] = -b_TrialState.cIneq.data[idx];
    }
    idx_Aineq = WorkingSet.ldA;
    if (WorkingSet.nVar != 0) {
      iy = 0;
      i2 = WorkingSet.ldA * (WorkingSet.sizes[2] - 1) + 1;
      for (idx_Partition = 1;
           idx_Aineq < 0 ? idx_Partition >= i2 : idx_Partition <= i2;
           idx_Partition += idx_Aineq) {
        c = 0.0;
        idx_lower = (idx_Partition + WorkingSet.nVar) - 1;
        for (int idx{idx_Partition}; idx <= idx_lower; idx++) {
          c += WorkingSet.Aineq.data[idx - 1] *
               b_TrialState.searchDir.data[idx - idx_Partition];
        }
        WorkingSet.bineq.data[iy] += c;
        iy++;
      }
    }
    idx_Aineq = 1;
    idx_lower = WorkingSet.sizes[2] + 1;
    iy = (WorkingSet.sizes[2] + WorkingSet.sizes[3]) + 1;
    i2 = WorkingSet.nActiveConstr;
    for (int idx{idxIneqOffset}; idx <= i2; idx++) {
      switch (WorkingSet.Wid.data[idx - 1]) {
      case 3:
        idx_Partition = idx_Aineq;
        idx_Aineq++;
        WorkingSet.bwset.data[idx - 1] =
            WorkingSet.bineq.data[WorkingSet.Wlocalidx.data[idx - 1] - 1];
        break;
      case 4:
        idx_Partition = idx_lower;
        idx_lower++;
        break;
      default:
        idx_Partition = iy;
        iy++;
        break;
      }
      b_TrialState.workingset_old.data[idx_Partition - 1] =
          WorkingSet.Wlocalidx.data[idx - 1];
    }
  }
  if (i - 1 >= 0) {
    std::copy(&b_TrialState.xstarsqp.data[0], &b_TrialState.xstarsqp.data[i],
              &b_TrialState.xstar.data[0]);
  }
  b_qpoptions = qpoptions;
  ::coder::optim::coder::qpactiveset::driver(
      Hessian_data, Hessian_size, grad_data, b_TrialState, memspace, WorkingSet,
      b_QRManager, b_CholManager, QPObjective, b_qpoptions,
      qpoptions.MaxIterations);
  i = static_cast<unsigned char>(nVar);
  idx_lower = (static_cast<unsigned char>(nVar) >> 1) << 1;
  idx_Aineq = idx_lower - 2;
  for (int idx{0}; idx <= idx_Aineq; idx += 2) {
    __m128d r1;
    r = _mm_loadu_pd(&b_TrialState.socDirection.data[idx]);
    r1 = _mm_loadu_pd(&b_TrialState.xstar.data[idx]);
    _mm_storeu_pd(&b_TrialState.socDirection.data[idx], _mm_sub_pd(r1, r));
    _mm_storeu_pd(&b_TrialState.xstar.data[idx], r);
  }
  for (int idx{idx_lower}; idx < i; idx++) {
    double oldDirIdx;
    c = b_TrialState.socDirection.data[idx];
    oldDirIdx = c;
    c = b_TrialState.xstar.data[idx] - c;
    b_TrialState.socDirection.data[idx] = c;
    b_TrialState.xstar.data[idx] = oldDirIdx;
  }
  success =
      (::coder::internal::blas::xnrm2(nVar, b_TrialState.socDirection.data) <=
       2.0 * ::coder::internal::blas::xnrm2(nVar, b_TrialState.xstar.data));
  idx_Partition = WorkingSet.sizes[2];
  idxIneqOffset = WorkingSet.sizes[3];
  if (WorkingSet.sizes[2] > 0) {
    i = static_cast<unsigned char>(WorkingSet.sizes[2]);
    idx_lower = (i / 2) << 1;
    idx_Aineq = idx_lower - 2;
    for (int idx{0}; idx <= idx_Aineq; idx += 2) {
      r = _mm_loadu_pd(&b_TrialState.cIneq.data[idx]);
      _mm_storeu_pd(&WorkingSet.bineq.data[idx],
                    _mm_mul_pd(r, _mm_set1_pd(-1.0)));
    }
    for (int idx{idx_lower}; idx < i; idx++) {
      WorkingSet.bineq.data[idx] = -b_TrialState.cIneq.data[idx];
    }
    if (!success) {
      idx_lower = WorkingSet.nWConstr[0] + WorkingSet.nWConstr[1];
      idx_Aineq = idx_lower + 1;
      iy = WorkingSet.nActiveConstr;
      for (nVar = idx_Aineq; nVar <= iy; nVar++) {
        WorkingSet.isActiveConstr
            .data[(WorkingSet.isActiveIdx[WorkingSet.Wid.data[nVar - 1] - 1] +
                   WorkingSet.Wlocalidx.data[nVar - 1]) -
                  2] = false;
      }
      WorkingSet.nWConstr[2] = 0;
      WorkingSet.nWConstr[3] = 0;
      WorkingSet.nWConstr[4] = 0;
      WorkingSet.nActiveConstr = idx_lower;
      for (int idx{0}; idx < nWIneq_old; idx++) {
        iy = b_TrialState.workingset_old.data[idx];
        WorkingSet.nWConstr[2]++;
        WorkingSet.isActiveConstr.data[(WorkingSet.isActiveIdx[2] + iy) - 2] =
            true;
        WorkingSet.nActiveConstr++;
        i = WorkingSet.nActiveConstr - 1;
        WorkingSet.Wid.data[i] = 3;
        WorkingSet.Wlocalidx.data[i] = iy;
        idx_Aineq = WorkingSet.ldA * (iy - 1);
        idx_lower = WorkingSet.ldA * i;
        i2 = WorkingSet.nVar - 1;
        for (nVar = 0; nVar <= i2; nVar++) {
          WorkingSet.ATwset.data[idx_lower + nVar] =
              WorkingSet.Aineq.data[idx_Aineq + nVar];
        }
        WorkingSet.bwset.data[i] = WorkingSet.bineq.data[iy - 1];
      }
      for (int idx{0}; idx < nWLower_old; idx++) {
        qpactiveset::WorkingSet::addBoundToActiveSetMatrix_(
            WorkingSet, 4,
            b_TrialState.workingset_old.data[idx + idx_Partition]);
      }
      for (int idx{0}; idx < nWUpper_old; idx++) {
        qpactiveset::WorkingSet::addBoundToActiveSetMatrix_(
            WorkingSet, 5,
            b_TrialState.workingset_old
                .data[(idx + idx_Partition) + idxIneqOffset]);
      }
    }
  }
  if (!success) {
    if (i1 - 1 >= 0) {
      std::copy(&b_TrialState.lambdaStopTest.data[0],
                &b_TrialState.lambdaStopTest.data[i1],
                &b_TrialState.lambda.data[0]);
    }
  } else {
    qpactiveset::parseoutput::sortLambdaQP(
        b_TrialState.lambda.data, WorkingSet.nActiveConstr, WorkingSet.sizes,
        WorkingSet.isActiveIdx, WorkingSet.Wid.data, WorkingSet.Wlocalidx.data,
        memspace.workspace_float.data);
  }
  return success;
}

} // namespace step
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for soc.cpp
//
// [EOF]
//
