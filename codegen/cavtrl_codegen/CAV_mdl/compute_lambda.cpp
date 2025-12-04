//
// File: compute_lambda.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "compute_lambda.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : double workspace_data[]
//                i_struct_T &solution
//                const g_struct_T &objective
//                const e_struct_T &qrmanager
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void compute_lambda(double workspace_data[], i_struct_T &solution,
                    const g_struct_T &objective, const e_struct_T &qrmanager)
{
  int nActiveConstr_tmp;
  nActiveConstr_tmp = qrmanager.ncols;
  if (qrmanager.ncols > 0) {
    double c;
    int idx;
    int idxQR;
    boolean_T guard1;
    guard1 = false;
    if (objective.objtype != 4) {
      boolean_T nonDegenerate;
      c = 100.0 * static_cast<double>(qrmanager.mrows) * 2.2204460492503131E-16;
      if ((qrmanager.mrows > 0) && (qrmanager.ncols > 0)) {
        nonDegenerate = true;
      } else {
        nonDegenerate = false;
      }
      if (nonDegenerate) {
        boolean_T guard2;
        idx = nActiveConstr_tmp;
        guard2 = false;
        if (qrmanager.mrows < qrmanager.ncols) {
          idxQR = qrmanager.mrows + qrmanager.ldq * (qrmanager.ncols - 1);
          while ((idx > qrmanager.mrows) &&
                 (std::abs(qrmanager.QR.data[idxQR - 1]) >= c)) {
            idx--;
            idxQR -= qrmanager.ldq;
          }
          nonDegenerate = (idx == qrmanager.mrows);
          if (nonDegenerate) {
            guard2 = true;
          }
        } else {
          guard2 = true;
        }
        if (guard2) {
          idxQR = idx + qrmanager.ldq * (idx - 1);
          while ((idx >= 1) && (std::abs(qrmanager.QR.data[idxQR - 1]) >= c)) {
            idx--;
            idxQR = (idxQR - qrmanager.ldq) - 1;
          }
          nonDegenerate = (idx == 0);
        }
      }
      if (!nonDegenerate) {
        solution.state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1) {
      int ix;
      int j;
      idxQR = qrmanager.ldq;
      if (qrmanager.mrows != 0) {
        std::memset(&workspace_data[0], 0,
                    static_cast<unsigned int>(nActiveConstr_tmp) *
                        sizeof(double));
        ix = 0;
        idx = qrmanager.ldq * (qrmanager.ncols - 1) + 1;
        for (int iac{1}; idxQR < 0 ? iac >= idx : iac <= idx; iac += idxQR) {
          c = 0.0;
          j = (iac + qrmanager.mrows) - 1;
          for (int ia{iac}; ia <= j; ia++) {
            c += qrmanager.Q.data[ia - 1] * objective.grad.data[ia - iac];
          }
          workspace_data[ix] += c;
          ix++;
        }
      }
      if ((qrmanager.QR.size[0] != 0) && (qrmanager.QR.size[1] != 0) &&
          (qrmanager.ncols != 0)) {
        for (j = nActiveConstr_tmp; j >= 1; j--) {
          idxQR = (j + (j - 1) * qrmanager.ldq) - 1;
          workspace_data[j - 1] /= qrmanager.QR.data[idxQR];
          for (idx = 0; idx <= j - 2; idx++) {
            ix = (j - idx) - 2;
            workspace_data[ix] -=
                workspace_data[j - 1] * qrmanager.QR.data[(idxQR - idx) - 1];
          }
        }
      }
      idxQR = (nActiveConstr_tmp / 2) << 1;
      ix = idxQR - 2;
      for (idx = 0; idx <= ix; idx += 2) {
        __m128d r;
        r = _mm_loadu_pd(&workspace_data[idx]);
        _mm_storeu_pd(&solution.lambda.data[idx],
                      _mm_mul_pd(r, _mm_set1_pd(-1.0)));
      }
      for (idx = idxQR; idx < nActiveConstr_tmp; idx++) {
        solution.lambda.data[idx] = -workspace_data[idx];
      }
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for compute_lambda.cpp
//
// [EOF]
//
