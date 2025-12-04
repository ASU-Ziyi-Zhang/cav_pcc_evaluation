//
// File: feasibleX0ForWorkingSet.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "feasibleX0ForWorkingSet.h"
#include "CAV_ctrl_mdl_wTraJ_241219_data.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xzgeqp3.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : double workspace_data[]
//                const int workspace_size[2]
//                double xCurrent_data[]
//                j_struct_T &workingset
//                e_struct_T &qrmanager
// Return Type  : boolean_T
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
boolean_T feasibleX0ForWorkingSet(double workspace_data[],
                                  const int workspace_size[2],
                                  double xCurrent_data[],
                                  j_struct_T &workingset, e_struct_T &qrmanager)
{
  double B_data[1225];
  int mWConstr;
  int nVar;
  boolean_T nonDegenerateWset;
  mWConstr = workingset.nActiveConstr;
  nVar = workingset.nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    __m128d r;
    __m128d r1;
    double c;
    int ar;
    int b_i;
    int i;
    int i1;
    int iAcol;
    int iy;
    int jBcol;
    for (iy = 0; iy < mWConstr; iy++) {
      c = workingset.bwset.data[iy];
      workspace_data[iy] = c;
      workspace_data[iy + workspace_size[0]] = c;
    }
    iAcol = workingset.ldA;
    if ((nVar != 0) && (mWConstr != 0)) {
      iy = 0;
      i = workingset.ldA * (mWConstr - 1) + 1;
      for (jBcol = 1; iAcol < 0 ? jBcol >= i : jBcol <= i; jBcol += iAcol) {
        c = 0.0;
        i1 = (jBcol + nVar) - 1;
        for (ar = jBcol; ar <= i1; ar++) {
          c += workingset.ATwset.data[ar - 1] * xCurrent_data[ar - jBcol];
        }
        workspace_data[iy] -= c;
        iy++;
      }
    }
    if (mWConstr >= nVar) {
      int ldq;
      int ldw_tmp;
      i = static_cast<unsigned char>(nVar);
      for (iAcol = 0; iAcol < i; iAcol++) {
        iy = qrmanager.ldq * iAcol;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager.QR.data[jBcol + iy] =
              workingset.ATwset.data[iAcol + workingset.ldA * jBcol];
        }
      }
      if (mWConstr * nVar == 0) {
        qrmanager.mrows = mWConstr;
        qrmanager.ncols = nVar;
        qrmanager.minRowCol = 0;
      } else {
        qrmanager.usedPivoting = false;
        qrmanager.mrows = mWConstr;
        qrmanager.ncols = nVar;
        jBcol = (static_cast<unsigned char>(nVar) >> 2) << 2;
        b_i = jBcol - 4;
        for (iy = 0; iy <= b_i; iy += 4) {
          _mm_storeu_si128(
              (__m128i *)&qrmanager.jpvt.data[iy],
              _mm_add_epi32(
                  _mm_add_epi32(_mm_set1_epi32(iy),
                                _mm_loadu_si128((const __m128i *)&iv[0])),
                  _mm_set1_epi32(1)));
        }
        for (iy = jBcol; iy < i; iy++) {
          qrmanager.jpvt.data[iy] = iy + 1;
        }
        if (mWConstr <= nVar) {
          i = mWConstr;
        } else {
          i = nVar;
        }
        qrmanager.minRowCol = i;
        iAcol = qrmanager.QR.size[0];
        iy = qrmanager.QR.size[1];
        if (iAcol <= iy) {
          iy = iAcol;
        }
        qrmanager.tau.size[0] = iy;
        if (iy - 1 >= 0) {
          std::memset(&qrmanager.tau.data[0], 0,
                      static_cast<unsigned int>(iy) * sizeof(double));
        }
        if ((qrmanager.QR.size[0] != 0) && (qrmanager.QR.size[1] != 0) &&
            (i >= 1)) {
          internal::reflapack::qrf(qrmanager.QR.data, qrmanager.QR.size,
                                   mWConstr, nVar, i, qrmanager.tau.data);
        }
      }
      QRManager::computeQ_(qrmanager, qrmanager.mrows);
      ldq = qrmanager.ldq;
      ldw_tmp = workspace_size[0];
      iAcol = workspace_size[0] * workspace_size[1];
      if (iAcol - 1 >= 0) {
        std::copy(&workspace_data[0], &workspace_data[iAcol], &B_data[0]);
      }
      if (nVar != 0) {
        for (int cr{0}; ldw_tmp < 0 ? cr >= ldw_tmp : cr <= ldw_tmp;
             cr += ldw_tmp) {
          i = cr + 1;
          i1 = cr + nVar;
          if (i <= i1) {
            std::memset(&workspace_data[i + -1], 0,
                        static_cast<unsigned int>((i1 - i) + 1) *
                            sizeof(double));
          }
        }
        iAcol = -1;
        for (int cr{0}; ldw_tmp < 0 ? cr >= ldw_tmp : cr <= ldw_tmp;
             cr += ldw_tmp) {
          ar = -1;
          i = cr + 1;
          i1 = cr + nVar;
          for (int ic{i}; ic <= i1; ic++) {
            c = 0.0;
            for (iy = 0; iy < mWConstr; iy++) {
              c += qrmanager.Q.data[(iy + ar) + 1] * B_data[(iy + iAcol) + 1];
            }
            workspace_data[ic - 1] += c;
            ar += ldq;
          }
          iAcol += ldw_tmp;
        }
      }
      for (ar = 0; ar < 2; ar++) {
        jBcol = ldw_tmp * ar - 1;
        for (int k{nVar}; k >= 1; k--) {
          iy = ldq * (k - 1) - 1;
          i = k + jBcol;
          c = workspace_data[i];
          if (c != 0.0) {
            workspace_data[i] = c / qrmanager.QR.data[k + iy];
            i1 = static_cast<unsigned char>(k - 1);
            for (b_i = 0; b_i < i1; b_i++) {
              int i2;
              i2 = (b_i + jBcol) + 1;
              workspace_data[i2] -=
                  workspace_data[i] * qrmanager.QR.data[(b_i + iy) + 1];
            }
          }
        }
      }
    } else {
      int ldq;
      int ldw_tmp;
      QRManager::factorQR(qrmanager, workingset.ATwset.data, nVar, mWConstr,
                          workingset.ldA);
      QRManager::computeQ_(qrmanager, qrmanager.minRowCol);
      ldq = qrmanager.ldq;
      ldw_tmp = workspace_size[0];
      for (ar = 0; ar < 2; ar++) {
        jBcol = ldw_tmp * ar;
        for (b_i = 0; b_i < mWConstr; b_i++) {
          iAcol = ldq * b_i;
          iy = b_i + jBcol;
          c = workspace_data[iy];
          i = static_cast<unsigned char>(b_i);
          for (int k{0}; k < i; k++) {
            c -= qrmanager.QR.data[k + iAcol] * workspace_data[k + jBcol];
          }
          workspace_data[iy] = c / qrmanager.QR.data[b_i + iAcol];
        }
      }
      iAcol = workspace_size[0] * workspace_size[1];
      if (iAcol - 1 >= 0) {
        std::copy(&workspace_data[0], &workspace_data[iAcol], &B_data[0]);
      }
      if (nVar != 0) {
        for (int cr{0}; ldw_tmp < 0 ? cr >= ldw_tmp : cr <= ldw_tmp;
             cr += ldw_tmp) {
          i = cr + 1;
          i1 = cr + nVar;
          if (i <= i1) {
            std::memset(&workspace_data[i + -1], 0,
                        static_cast<unsigned int>((i1 - i) + 1) *
                            sizeof(double));
          }
        }
        iAcol = 0;
        for (int cr{0}; ldw_tmp < 0 ? cr >= ldw_tmp : cr <= ldw_tmp;
             cr += ldw_tmp) {
          ar = -1;
          i = iAcol + 1;
          i1 = iAcol + mWConstr;
          for (int k{i}; k <= i1; k++) {
            int i2;
            i2 = cr + 1;
            iy = cr + nVar;
            jBcol = ((((iy - cr) / 2) << 1) + cr) + 1;
            b_i = jBcol - 2;
            for (int ic{i2}; ic <= b_i; ic += 2) {
              r = _mm_loadu_pd(&qrmanager.Q.data[(ar + ic) - cr]);
              r1 = _mm_loadu_pd(&workspace_data[ic - 1]);
              _mm_storeu_pd(
                  &workspace_data[ic - 1],
                  _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(B_data[k - 1]), r)));
            }
            for (int ic{jBcol}; ic <= iy; ic++) {
              workspace_data[ic - 1] +=
                  B_data[k - 1] * qrmanager.Q.data[(ar + ic) - cr];
            }
            ar += ldq;
          }
          iAcol += ldw_tmp;
        }
      }
    }
    iy = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (iy <= static_cast<unsigned char>(nVar) - 1) {
        c = workspace_data[iy];
        if (std::isinf(c) || std::isnan(c)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace_data[iy + workspace_size[0]];
          if (std::isinf(c) || std::isnan(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            iy++;
          }
        }
      } else {
        double v;
        if (nVar >= 1) {
          iAcol = nVar - 1;
          jBcol = (nVar / 2) << 1;
          b_i = jBcol - 2;
          for (int k{0}; k <= b_i; k += 2) {
            r = _mm_loadu_pd(&workspace_data[k]);
            r1 = _mm_loadu_pd(&xCurrent_data[k]);
            _mm_storeu_pd(&workspace_data[k], _mm_add_pd(r, r1));
          }
          for (int k{jBcol}; k <= iAcol; k++) {
            workspace_data[k] += xCurrent_data[k];
          }
        }
        if (workingset.probType == 2) {
          c = 0.0;
          if (workingset.Aineq.size[0] != 0) {
            i = static_cast<unsigned char>(workingset.sizes[2]);
            if (i - 1 >= 0) {
              std::copy(&workingset.bineq.data[0], &workingset.bineq.data[i],
                        &workingset.maxConstrWorkspace.data[0]);
            }
            internal::blas::b_xgemv(workingset.nVarOrig, workingset.sizes[2],
                                    workingset.Aineq.data, workingset.ldA,
                                    workspace_data,
                                    workingset.maxConstrWorkspace.data);
            for (iy = 0; iy < i; iy++) {
              workingset.maxConstrWorkspace.data[iy] -=
                  workspace_data[workingset.nVarOrig + iy];
              c = std::fmax(c, workingset.maxConstrWorkspace.data[iy]);
            }
          }
        } else {
          c = 0.0;
          if (workingset.Aineq.size[0] != 0) {
            i = static_cast<unsigned char>(workingset.sizes[2]);
            if (i - 1 >= 0) {
              std::copy(&workingset.bineq.data[0], &workingset.bineq.data[i],
                        &workingset.maxConstrWorkspace.data[0]);
            }
            internal::blas::b_xgemv(workingset.nVar, workingset.sizes[2],
                                    workingset.Aineq.data, workingset.ldA,
                                    workspace_data,
                                    workingset.maxConstrWorkspace.data);
            for (iy = 0; iy < i; iy++) {
              c = std::fmax(c, workingset.maxConstrWorkspace.data[iy]);
            }
          }
        }
        if (workingset.sizes[3] > 0) {
          i = static_cast<unsigned char>(workingset.sizes[3]);
          for (iy = 0; iy < i; iy++) {
            iAcol = workingset.indexLB.data[iy] - 1;
            c = std::fmax(c,
                          -workspace_data[iAcol] - workingset.lb.data[iAcol]);
          }
        }
        if (workingset.sizes[4] > 0) {
          i = static_cast<unsigned char>(workingset.sizes[4]);
          for (iy = 0; iy < i; iy++) {
            iAcol = workingset.indexUB.data[iy] - 1;
            c = std::fmax(c, workspace_data[iAcol] - workingset.ub.data[iAcol]);
          }
        }
        if (workingset.sizes[0] > 0) {
          i = static_cast<unsigned char>(workingset.sizes[0]);
          for (iy = 0; iy < i; iy++) {
            c = std::fmax(
                c, std::abs(
                       workspace_data[workingset.indexFixed.data[iy] - 1] -
                       workingset.ub.data[workingset.indexFixed.data[iy] - 1]));
          }
        }
        iAcol = workspace_size[0] - 1;
        if (workingset.probType == 2) {
          v = 0.0;
          if (workingset.Aineq.size[0] != 0) {
            i = static_cast<unsigned char>(workingset.sizes[2]);
            if (i - 1 >= 0) {
              std::copy(&workingset.bineq.data[0], &workingset.bineq.data[i],
                        &workingset.maxConstrWorkspace.data[0]);
            }
            internal::blas::xgemv(workingset.nVarOrig, workingset.sizes[2],
                                  workingset.Aineq.data, workingset.ldA,
                                  workspace_data, workspace_size[0] + 1,
                                  workingset.maxConstrWorkspace.data);
            for (iy = 0; iy < i; iy++) {
              workingset.maxConstrWorkspace.data[iy] -=
                  workspace_data[((iAcol + workingset.nVarOrig) + iy) + 1];
              v = std::fmax(v, workingset.maxConstrWorkspace.data[iy]);
            }
          }
        } else {
          v = 0.0;
          if (workingset.Aineq.size[0] != 0) {
            i = static_cast<unsigned char>(workingset.sizes[2]);
            if (i - 1 >= 0) {
              std::copy(&workingset.bineq.data[0], &workingset.bineq.data[i],
                        &workingset.maxConstrWorkspace.data[0]);
            }
            internal::blas::xgemv(workingset.nVar, workingset.sizes[2],
                                  workingset.Aineq.data, workingset.ldA,
                                  workspace_data, workspace_size[0] + 1,
                                  workingset.maxConstrWorkspace.data);
            for (iy = 0; iy < i; iy++) {
              v = std::fmax(v, workingset.maxConstrWorkspace.data[iy]);
            }
          }
        }
        if (workingset.sizes[3] > 0) {
          i = static_cast<unsigned char>(workingset.sizes[3]);
          for (iy = 0; iy < i; iy++) {
            v = std::fmax(
                v, -workspace_data[iAcol + workingset.indexLB.data[iy]] -
                       workingset.lb.data[workingset.indexLB.data[iy] - 1]);
          }
        }
        if (workingset.sizes[4] > 0) {
          i = static_cast<unsigned char>(workingset.sizes[4]);
          for (iy = 0; iy < i; iy++) {
            v = std::fmax(
                v, workspace_data[iAcol + workingset.indexUB.data[iy]] -
                       workingset.ub.data[workingset.indexUB.data[iy] - 1]);
          }
        }
        if (workingset.sizes[0] > 0) {
          i = static_cast<unsigned char>(workingset.sizes[0]);
          for (iy = 0; iy < i; iy++) {
            v = std::fmax(
                v, std::abs(
                       workspace_data[iAcol + workingset.indexFixed.data[iy]] -
                       workingset.ub.data[workingset.indexFixed.data[iy] - 1]));
          }
        }
        if ((c <= 2.2204460492503131E-16) || (c < v)) {
          i = static_cast<unsigned char>(nVar);
          if (i - 1 >= 0) {
            std::copy(&workspace_data[0], &workspace_data[i],
                      &xCurrent_data[0]);
          }
        } else {
          i = static_cast<unsigned char>(nVar);
          for (int k{0}; k < i; k++) {
            xCurrent_data[k] = workspace_data[workspace_size[0] + k];
          }
        }
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return nonDegenerateWset;
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for feasibleX0ForWorkingSet.cpp
//
// [EOF]
//
