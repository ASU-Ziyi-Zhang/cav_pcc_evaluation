//
// File: PresolveWorkingSet.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "PresolveWorkingSet.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "computeQ_.h"
#include "countsort.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include "coder_bounded_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : i_struct_T &solution
//                h_struct_T &memspace
//                j_struct_T &workingset
//                e_struct_T &qrmanager
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void PresolveWorkingSet(i_struct_T &solution, h_struct_T &memspace,
                        j_struct_T &workingset, e_struct_T &qrmanager)
{
  double tol;
  int i;
  int i1;
  int idx;
  int idxDiag;
  int ix;
  int mTotalWorkingEq_tmp_tmp;
  int mWorkingFixed;
  int nDepInd;
  int nVar_tmp;
  int u0;
  solution.state = 82;
  nVar_tmp = workingset.nVar;
  mWorkingFixed = workingset.nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset.nWConstr[0] + workingset.nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    i = static_cast<unsigned char>(nVar_tmp);
    for (ix = 0; ix < mTotalWorkingEq_tmp_tmp; ix++) {
      for (int idx_col{0}; idx_col < i; idx_col++) {
        qrmanager.QR.data[ix + qrmanager.ldq * idx_col] =
            workingset.ATwset.data[idx_col + workingset.ldA * ix];
      }
    }
    nDepInd = mTotalWorkingEq_tmp_tmp - workingset.nVar;
    if (nDepInd <= 0) {
      nDepInd = 0;
    }
    i1 = static_cast<unsigned char>(workingset.nVar);
    if (i1 - 1 >= 0) {
      std::memset(&qrmanager.jpvt.data[0], 0,
                  static_cast<unsigned int>(i1) * sizeof(int));
    }
    i1 = mTotalWorkingEq_tmp_tmp * workingset.nVar;
    if (i1 == 0) {
      qrmanager.mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager.ncols = workingset.nVar;
      qrmanager.minRowCol = 0;
    } else {
      qrmanager.usedPivoting = true;
      qrmanager.mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager.ncols = workingset.nVar;
      ix = workingset.nVar;
      if (mTotalWorkingEq_tmp_tmp <= ix) {
        ix = mTotalWorkingEq_tmp_tmp;
      }
      qrmanager.minRowCol = ix;
      qrmanager.tau.size[0] = internal::lapack::xgeqp3(
          qrmanager.QR.data, qrmanager.QR.size, mTotalWorkingEq_tmp_tmp,
          workingset.nVar, qrmanager.jpvt.data, qrmanager.tau.data);
    }
    tol = 100.0 * static_cast<double>(workingset.nVar) * 2.2204460492503131E-16;
    u0 = workingset.nVar;
    if (u0 > mTotalWorkingEq_tmp_tmp) {
      u0 = mTotalWorkingEq_tmp_tmp;
    }
    idxDiag = u0 + qrmanager.ldq * (u0 - 1);
    while ((idxDiag > 0) && (std::abs(qrmanager.QR.data[idxDiag - 1]) < tol)) {
      idxDiag = (idxDiag - qrmanager.ldq) - 1;
      nDepInd++;
    }
    if (nDepInd > 0) {
      boolean_T exitg1;
      QRManager::computeQ_(qrmanager, qrmanager.mrows);
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx <= nDepInd - 1)) {
        double qtb;
        ix = qrmanager.ldq * ((mTotalWorkingEq_tmp_tmp - idx) - 1);
        qtb = 0.0;
        for (int k{0}; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager.Q.data[ix + k] * workingset.bwset.data[k];
        }
        if (std::abs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idx++;
        }
      }
    }
    if (nDepInd > 0) {
      for (int idx_col{0}; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        idx = qrmanager.ldq * idx_col;
        idxDiag = workingset.ldA * idx_col;
        for (int k{0}; k < i; k++) {
          qrmanager.QR.data[idx + k] = workingset.ATwset.data[idxDiag + k];
        }
      }
      for (idx = 0; idx < mWorkingFixed; idx++) {
        qrmanager.jpvt.data[idx] = 1;
      }
      i = workingset.nWConstr[0] + 1;
      if (i <= mTotalWorkingEq_tmp_tmp) {
        std::memset(
            &qrmanager.jpvt.data[i + -1], 0,
            static_cast<unsigned int>((mTotalWorkingEq_tmp_tmp - i) + 1) *
                sizeof(int));
      }
      if (i1 == 0) {
        qrmanager.mrows = workingset.nVar;
        qrmanager.ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager.minRowCol = 0;
      } else {
        qrmanager.usedPivoting = true;
        qrmanager.mrows = workingset.nVar;
        qrmanager.ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager.minRowCol = u0;
        qrmanager.tau.size[0] = internal::lapack::xgeqp3(
            qrmanager.QR.data, qrmanager.QR.size, workingset.nVar,
            mTotalWorkingEq_tmp_tmp, qrmanager.jpvt.data, qrmanager.tau.data);
      }
      for (idx = 0; idx < nDepInd; idx++) {
        memspace.workspace_int.data[idx] =
            qrmanager.jpvt.data[(mTotalWorkingEq_tmp_tmp - nDepInd) + idx];
      }
      utils::countsort(memspace.workspace_int.data, nDepInd,
                       memspace.workspace_sort.data, 1,
                       mTotalWorkingEq_tmp_tmp);
      for (idx = nDepInd; idx >= 1; idx--) {
        i = memspace.workspace_int.data[idx - 1];
        if (i <= mTotalWorkingEq_tmp_tmp) {
          if ((workingset.nActiveConstr == mTotalWorkingEq_tmp_tmp) ||
              (i == mTotalWorkingEq_tmp_tmp)) {
            workingset.mEqRemoved++;
            // A check that is always false is detected at compile-time.
            // Eliminating code that follows.
          } else {
            workingset.mEqRemoved++;
            // A check that is always false is detected at compile-time.
            // Eliminating code that follows.
          }
        }
      }
    }
  }
  if ((nDepInd != -1) && (workingset.nActiveConstr <= qrmanager.ldq)) {
    boolean_T guard1;
    boolean_T okWorkingSet;
    ix = workingset.nActiveConstr;
    if ((workingset.nWConstr[2] + workingset.nWConstr[3]) +
            workingset.nWConstr[4] >
        0) {
      tol =
          100.0 * static_cast<double>(workingset.nVar) * 2.2204460492503131E-16;
      for (idx = 0; idx < mTotalWorkingEq_tmp_tmp; idx++) {
        qrmanager.jpvt.data[idx] = 1;
      }
      i = mTotalWorkingEq_tmp_tmp + 1;
      if (i <= ix) {
        std::memset(&qrmanager.jpvt.data[i + -1], 0,
                    static_cast<unsigned int>((ix - i) + 1) * sizeof(int));
      }
      i = workingset.nActiveConstr;
      for (int idx_col{0}; idx_col < i; idx_col++) {
        idx = qrmanager.ldq * idx_col;
        idxDiag = workingset.ldA * idx_col;
        i1 = static_cast<unsigned char>(nVar_tmp);
        for (int k{0}; k < i1; k++) {
          qrmanager.QR.data[idx + k] = workingset.ATwset.data[idxDiag + k];
        }
      }
      if (workingset.nVar * workingset.nActiveConstr == 0) {
        qrmanager.mrows = workingset.nVar;
        qrmanager.ncols = workingset.nActiveConstr;
        qrmanager.minRowCol = 0;
      } else {
        qrmanager.usedPivoting = true;
        qrmanager.mrows = workingset.nVar;
        qrmanager.ncols = workingset.nActiveConstr;
        u0 = workingset.nVar;
        ix = workingset.nActiveConstr;
        if (u0 <= ix) {
          ix = u0;
        }
        qrmanager.minRowCol = ix;
        qrmanager.tau.size[0] = internal::lapack::xgeqp3(
            qrmanager.QR.data, qrmanager.QR.size, workingset.nVar,
            workingset.nActiveConstr, qrmanager.jpvt.data, qrmanager.tau.data);
      }
      ix = 0;
      for (idx = workingset.nActiveConstr - 1; idx + 1 > nVar_tmp; idx--) {
        ix++;
        memspace.workspace_int.data[ix - 1] = qrmanager.jpvt.data[idx];
      }
      if (idx + 1 <= workingset.nVar) {
        idxDiag = idx + qrmanager.ldq * idx;
        while ((idx + 1 > mTotalWorkingEq_tmp_tmp) &&
               (std::abs(qrmanager.QR.data[idxDiag]) < tol)) {
          ix++;
          memspace.workspace_int.data[ix - 1] = qrmanager.jpvt.data[idx];
          idx--;
          idxDiag = (idxDiag - qrmanager.ldq) - 1;
        }
      }
      utils::countsort(memspace.workspace_int.data, ix,
                       memspace.workspace_sort.data,
                       mTotalWorkingEq_tmp_tmp + 1, workingset.nActiveConstr);
      for (idx = ix; idx >= 1; idx--) {
        WorkingSet::removeConstr(workingset,
                                 memspace.workspace_int.data[idx - 1]);
      }
    }
    okWorkingSet = feasibleX0ForWorkingSet(
        memspace.workspace_float.data, memspace.workspace_float.size,
        solution.xstar.data, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      ix = workingset.nActiveConstr;
      i = workingset.nWConstr[0] + workingset.nWConstr[1];
      if ((workingset.nWConstr[2] + workingset.nWConstr[3]) +
              workingset.nWConstr[4] >
          0) {
        tol = 1000.0 * static_cast<double>(workingset.nVar) *
              2.2204460492503131E-16;
        for (idx = 0; idx < i; idx++) {
          qrmanager.jpvt.data[idx] = 1;
        }
        i1 = i + 1;
        if (i1 <= ix) {
          std::memset(&qrmanager.jpvt.data[i1 + -1], 0,
                      static_cast<unsigned int>((ix - i1) + 1) * sizeof(int));
        }
        i1 = workingset.nActiveConstr;
        for (int idx_col{0}; idx_col < i1; idx_col++) {
          idx = qrmanager.ldq * idx_col;
          idxDiag = workingset.ldA * idx_col;
          ix = static_cast<unsigned char>(nVar_tmp);
          for (int k{0}; k < ix; k++) {
            qrmanager.QR.data[idx + k] = workingset.ATwset.data[idxDiag + k];
          }
        }
        if (workingset.nVar * workingset.nActiveConstr == 0) {
          qrmanager.mrows = workingset.nVar;
          qrmanager.ncols = workingset.nActiveConstr;
          qrmanager.minRowCol = 0;
        } else {
          qrmanager.usedPivoting = true;
          qrmanager.mrows = workingset.nVar;
          qrmanager.ncols = workingset.nActiveConstr;
          u0 = workingset.nVar;
          ix = workingset.nActiveConstr;
          if (u0 <= ix) {
            ix = u0;
          }
          qrmanager.minRowCol = ix;
          qrmanager.tau.size[0] = internal::lapack::xgeqp3(
              qrmanager.QR.data, qrmanager.QR.size, workingset.nVar,
              workingset.nActiveConstr, qrmanager.jpvt.data,
              qrmanager.tau.data);
        }
        ix = 0;
        for (idx = workingset.nActiveConstr - 1; idx + 1 > nVar_tmp; idx--) {
          ix++;
          memspace.workspace_int.data[ix - 1] = qrmanager.jpvt.data[idx];
        }
        if (idx + 1 <= workingset.nVar) {
          idxDiag = idx + qrmanager.ldq * idx;
          while ((idx + 1 > i) &&
                 (std::abs(qrmanager.QR.data[idxDiag]) < tol)) {
            ix++;
            memspace.workspace_int.data[ix - 1] = qrmanager.jpvt.data[idx];
            idx--;
            idxDiag = (idxDiag - qrmanager.ldq) - 1;
          }
        }
        utils::countsort(memspace.workspace_int.data, ix,
                         memspace.workspace_sort.data, i + 1,
                         workingset.nActiveConstr);
        for (idx = ix; idx >= 1; idx--) {
          WorkingSet::removeConstr(workingset,
                                   memspace.workspace_int.data[idx - 1]);
        }
      }
      okWorkingSet = feasibleX0ForWorkingSet(
          memspace.workspace_float.data, memspace.workspace_float.size,
          solution.xstar.data, workingset, qrmanager);
      if (!okWorkingSet) {
        solution.state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 &&
        (workingset.nWConstr[0] + workingset.nWConstr[1] == workingset.nVar)) {
      tol = WorkingSet::maxConstraintViolation(workingset, solution.xstar.data);
      if (tol > 1.0E-6) {
        solution.state = -2;
      }
    }
  } else {
    solution.state = -3;
    ix = mTotalWorkingEq_tmp_tmp + 1;
    idxDiag = workingset.nActiveConstr;
    for (idx = ix; idx <= idxDiag; idx++) {
      workingset.isActiveConstr
          .data[(workingset.isActiveIdx[workingset.Wid.data[idx - 1] - 1] +
                 workingset.Wlocalidx.data[idx - 1]) -
                2] = false;
    }
    workingset.nWConstr[2] = 0;
    workingset.nWConstr[3] = 0;
    workingset.nWConstr[4] = 0;
    workingset.nActiveConstr = mTotalWorkingEq_tmp_tmp;
  }
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for PresolveWorkingSet.cpp
//
// [EOF]
//
