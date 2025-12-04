//
// File: compute_deltax.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "compute_deltax.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "fullColLDL2_.h"
#include "rt_nonfinite.h"
#include "solve.h"
#include "xgemm.h"
#include "xpotrf.h"
#include "coder_bounded_array.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : const double H_data[]
//                const int H_size[2]
//                i_struct_T &solution
//                h_struct_T &memspace
//                const e_struct_T &qrmanager
//                f_struct_T &cholmanager
//                const g_struct_T &objective
//                boolean_T alwaysPositiveDef
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void compute_deltax(const double H_data[], const int H_size[2],
                    i_struct_T &solution, h_struct_T &memspace,
                    const e_struct_T &qrmanager, f_struct_T &cholmanager,
                    const g_struct_T &objective, boolean_T alwaysPositiveDef)
{
  int mNull_tmp;
  int nVar_tmp;
  nVar_tmp = qrmanager.mrows - 1;
  mNull_tmp = qrmanager.mrows - qrmanager.ncols;
  if (mNull_tmp <= 0) {
    if (nVar_tmp >= 0) {
      std::memset(&solution.searchDir.data[0], 0,
                  static_cast<unsigned int>(nVar_tmp + 1) * sizeof(double));
    }
  } else {
    __m128d r;
    int idx;
    int ix;
    int scalarLB;
    scalarLB = ((nVar_tmp + 1) / 2) << 1;
    ix = scalarLB - 2;
    for (idx = 0; idx <= ix; idx += 2) {
      r = _mm_loadu_pd(&objective.grad.data[idx]);
      _mm_storeu_pd(&solution.searchDir.data[idx],
                    _mm_mul_pd(r, _mm_set1_pd(-1.0)));
    }
    for (idx = scalarLB; idx <= nVar_tmp; idx++) {
      solution.searchDir.data[idx] = -objective.grad.data[idx];
    }
    if (qrmanager.ncols <= 0) {
      switch (objective.objtype) {
      case 5:
        break;
      case 3: {
        double smax;
        int idx_col;
        int nVars;
        if (alwaysPositiveDef) {
          cholmanager.ndims = qrmanager.mrows;
          if ((H_size[0] != 0) && (H_size[1] != 0)) {
            for (idx = 0; idx <= nVar_tmp; idx++) {
              idx_col = (nVar_tmp + 1) * idx;
              scalarLB = cholmanager.ldm * idx;
              for (int idx_row{0}; idx_row <= nVar_tmp; idx_row++) {
                cholmanager.FMat.data[scalarLB + idx_row] =
                    H_data[idx_col + idx_row];
              }
            }
          }
          cholmanager.info = internal::lapack::xpotrf(
              qrmanager.mrows, cholmanager.FMat.data, cholmanager.ldm);
        } else {
          cholmanager.ndims = qrmanager.mrows;
          if ((H_size[0] != 0) && (H_size[1] != 0)) {
            for (idx = 0; idx <= nVar_tmp; idx++) {
              idx_col = qrmanager.mrows * idx;
              scalarLB = cholmanager.ldm * idx;
              for (int idx_row{0}; idx_row <= nVar_tmp; idx_row++) {
                cholmanager.FMat.data[scalarLB + idx_row] =
                    H_data[idx_col + idx_row];
              }
            }
          }
          if (qrmanager.mrows < 1) {
            nVars = -1;
          } else {
            nVars = 0;
            if (qrmanager.mrows > 1) {
              ix = 0;
              smax = std::abs(cholmanager.FMat.data[0]);
              for (int idx_row{2}; idx_row <= nVar_tmp + 1; idx_row++) {
                double s;
                ix = (ix + cholmanager.ldm) + 1;
                s = std::abs(cholmanager.FMat.data[ix]);
                if (s > smax) {
                  nVars = idx_row - 1;
                  smax = s;
                }
              }
            }
          }
          cholmanager.regTol_ = std::fmax(
              std::abs(cholmanager.FMat.data[nVars + cholmanager.ldm * nVars]) *
                  2.2204460492503131E-16,
              0.0);
          DynamicRegCholManager::fullColLDL2_(cholmanager, qrmanager.mrows);
          if (cholmanager.ConvexCheck) {
            idx = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (idx <= nVar_tmp) {
                if (cholmanager.FMat.data[idx + cholmanager.ldm * idx] <= 0.0) {
                  cholmanager.info = -idx - 1;
                  exitg1 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager.ConvexCheck = false;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }
        }
        if (cholmanager.info != 0) {
          solution.state = -6;
        } else if (alwaysPositiveDef) {
          CholManager::solve(cholmanager, solution.searchDir.data);
        } else {
          int i;
          scalarLB = cholmanager.ndims - 2;
          if ((cholmanager.FMat.size[0] != 0) &&
              (cholmanager.FMat.size[1] != 0) && (cholmanager.ndims != 0)) {
            for (idx_col = 0; idx_col <= scalarLB + 1; idx_col++) {
              nVars = idx_col + idx_col * cholmanager.ldm;
              i = scalarLB - idx_col;
              for (idx = 0; idx <= i; idx++) {
                ix = (idx_col + idx) + 1;
                solution.searchDir.data[ix] -=
                    solution.searchDir.data[idx_col] *
                    cholmanager.FMat.data[(nVars + idx) + 1];
              }
            }
          }
          i = cholmanager.ndims;
          for (idx = 0; idx < i; idx++) {
            solution.searchDir.data[idx] /=
                cholmanager.FMat.data[idx + cholmanager.ldm * idx];
          }
          scalarLB = cholmanager.ndims;
          if ((cholmanager.FMat.size[0] != 0) &&
              (cholmanager.FMat.size[1] != 0) && (cholmanager.ndims != 0)) {
            for (idx_col = scalarLB; idx_col >= 1; idx_col--) {
              nVars = (idx_col - 1) * cholmanager.ldm;
              smax = solution.searchDir.data[idx_col - 1];
              i = idx_col + 1;
              for (idx = scalarLB; idx >= i; idx--) {
                smax -= cholmanager.FMat.data[(nVars + idx) - 1] *
                        solution.searchDir.data[idx - 1];
              }
              solution.searchDir.data[idx_col - 1] = smax;
            }
          }
        }
      } break;
      default: {
        if (alwaysPositiveDef) {
          int idx_col;
          int nVars;
          nVars = objective.nvar;
          cholmanager.ndims = objective.nvar;
          if ((H_size[0] != 0) && (H_size[1] != 0)) {
            for (idx = 0; idx < nVars; idx++) {
              idx_col = nVars * idx;
              scalarLB = cholmanager.ldm * idx;
              for (int idx_row{0}; idx_row < nVars; idx_row++) {
                cholmanager.FMat.data[scalarLB + idx_row] =
                    H_data[idx_col + idx_row];
              }
            }
          }
          cholmanager.info = internal::lapack::xpotrf(
              objective.nvar, cholmanager.FMat.data, cholmanager.ldm);
          if (cholmanager.info != 0) {
            solution.state = -6;
          } else {
            double smax;
            int i;
            CholManager::solve(cholmanager, solution.searchDir.data);
            smax = 1.0 / objective.beta;
            idx_col = objective.nvar + 1;
            i = qrmanager.mrows;
            scalarLB = ((((i - idx_col) + 1) / 2) << 1) + idx_col;
            ix = scalarLB - 2;
            for (int idx_row{idx_col}; idx_row <= ix; idx_row += 2) {
              r = _mm_loadu_pd(&solution.searchDir.data[idx_row - 1]);
              _mm_storeu_pd(&solution.searchDir.data[idx_row - 1],
                            _mm_mul_pd(_mm_set1_pd(smax), r));
            }
            for (int idx_row{scalarLB}; idx_row <= i; idx_row++) {
              solution.searchDir.data[idx_row - 1] *= smax;
            }
          }
        }
      } break;
      }
    } else {
      int nullStartIdx_tmp;
      nullStartIdx_tmp = qrmanager.ldq * qrmanager.ncols + 1;
      if (objective.objtype == 5) {
        for (idx = 0; idx < mNull_tmp; idx++) {
          memspace.workspace_float.data[idx] =
              -qrmanager.Q
                   .data[nVar_tmp + qrmanager.ldq * (qrmanager.ncols + idx)];
        }
        scalarLB = qrmanager.ldq;
        if (qrmanager.mrows != 0) {
          int i;
          std::memset(&solution.searchDir.data[0], 0,
                      static_cast<unsigned int>(nVar_tmp + 1) * sizeof(double));
          ix = 0;
          i = nullStartIdx_tmp + qrmanager.ldq * (mNull_tmp - 1);
          for (idx = nullStartIdx_tmp; scalarLB < 0 ? idx >= i : idx <= i;
               idx += scalarLB) {
            int idx_col;
            idx_col = idx + nVar_tmp;
            for (int ia{idx}; ia <= idx_col; ia++) {
              int nVars;
              nVars = ia - idx;
              solution.searchDir.data[nVars] +=
                  qrmanager.Q.data[ia - 1] * memspace.workspace_float.data[ix];
            }
            ix++;
          }
        }
      } else {
        double smax;
        int i;
        int idx_col;
        int idx_row;
        int nVars;
        if (objective.objtype == 3) {
          internal::blas::xgemm(qrmanager.mrows, mNull_tmp, qrmanager.mrows,
                                H_data, qrmanager.mrows, qrmanager.Q.data,
                                nullStartIdx_tmp, qrmanager.ldq,
                                memspace.workspace_float.data,
                                memspace.workspace_float.size[0]);
          internal::blas::xgemm(mNull_tmp, mNull_tmp, qrmanager.mrows,
                                qrmanager.Q.data, nullStartIdx_tmp,
                                qrmanager.ldq, memspace.workspace_float.data,
                                memspace.workspace_float.size[0],
                                cholmanager.FMat.data, cholmanager.ldm);
        } else if (alwaysPositiveDef) {
          nVars = qrmanager.mrows;
          internal::blas::xgemm(
              objective.nvar, mNull_tmp, objective.nvar, H_data, objective.nvar,
              qrmanager.Q.data, nullStartIdx_tmp, qrmanager.ldq,
              memspace.workspace_float.data, memspace.workspace_float.size[0]);
          i = objective.nvar + 1;
          scalarLB = ((((nVars - i) + 1) / 2) << 1) + i;
          ix = scalarLB - 2;
          for (idx_col = 0; idx_col < mNull_tmp; idx_col++) {
            for (idx_row = i; idx_row <= ix; idx_row += 2) {
              r = _mm_loadu_pd(
                  &qrmanager.Q
                       .data[(idx_row + qrmanager.Q.size[0] *
                                            (idx_col + qrmanager.ncols)) -
                             1]);
              _mm_storeu_pd(
                  &memspace.workspace_float
                       .data[(idx_row +
                              memspace.workspace_float.size[0] * idx_col) -
                             1],
                  _mm_mul_pd(_mm_set1_pd(objective.beta), r));
            }
            for (idx_row = scalarLB; idx_row <= nVars; idx_row++) {
              memspace.workspace_float
                  .data[(idx_row + memspace.workspace_float.size[0] * idx_col) -
                        1] =
                  objective.beta *
                  qrmanager.Q.data[(idx_row + qrmanager.Q.size[0] *
                                                  (idx_col + qrmanager.ncols)) -
                                   1];
            }
          }
          internal::blas::xgemm(mNull_tmp, mNull_tmp, qrmanager.mrows,
                                qrmanager.Q.data, nullStartIdx_tmp,
                                qrmanager.ldq, memspace.workspace_float.data,
                                memspace.workspace_float.size[0],
                                cholmanager.FMat.data, cholmanager.ldm);
        }
        if (alwaysPositiveDef) {
          cholmanager.ndims = mNull_tmp;
          cholmanager.info = internal::lapack::xpotrf(
              mNull_tmp, cholmanager.FMat.data, cholmanager.ldm);
        } else {
          cholmanager.ndims = mNull_tmp;
          nVars = 0;
          if (mNull_tmp > 1) {
            ix = 0;
            smax = std::abs(cholmanager.FMat.data[0]);
            for (idx_row = 2; idx_row <= mNull_tmp; idx_row++) {
              double s;
              ix = (ix + cholmanager.ldm) + 1;
              s = std::abs(cholmanager.FMat.data[ix]);
              if (s > smax) {
                nVars = idx_row - 1;
                smax = s;
              }
            }
          }
          cholmanager.regTol_ = std::fmax(
              std::abs(cholmanager.FMat.data[nVars + cholmanager.ldm * nVars]) *
                  2.2204460492503131E-16,
              0.0);
          DynamicRegCholManager::fullColLDL2_(cholmanager, mNull_tmp);
          if (cholmanager.ConvexCheck) {
            idx = 0;
            int exitg1;
            do {
              exitg1 = 0;
              if (idx <= mNull_tmp - 1) {
                if (cholmanager.FMat.data[idx + cholmanager.ldm * idx] <= 0.0) {
                  cholmanager.info = -idx - 1;
                  exitg1 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager.ConvexCheck = false;
                exitg1 = 1;
              }
            } while (exitg1 == 0);
          }
        }
        if (cholmanager.info != 0) {
          solution.state = -6;
        } else {
          idx_row = qrmanager.ldq;
          if (qrmanager.mrows != 0) {
            std::memset(&memspace.workspace_float.data[0], 0,
                        static_cast<unsigned int>(mNull_tmp) * sizeof(double));
            scalarLB = 0;
            i = nullStartIdx_tmp + qrmanager.ldq * (mNull_tmp - 1);
            for (idx = nullStartIdx_tmp; idx_row < 0 ? idx >= i : idx <= i;
                 idx += idx_row) {
              smax = 0.0;
              idx_col = idx + nVar_tmp;
              for (int ia{idx}; ia <= idx_col; ia++) {
                smax +=
                    qrmanager.Q.data[ia - 1] * objective.grad.data[ia - idx];
              }
              memspace.workspace_float.data[scalarLB] -= smax;
              scalarLB++;
            }
          }
          if (alwaysPositiveDef) {
            scalarLB = cholmanager.ndims;
            if ((cholmanager.FMat.size[0] != 0) &&
                (cholmanager.FMat.size[1] != 0) && (cholmanager.ndims != 0)) {
              for (idx_col = 0; idx_col < scalarLB; idx_col++) {
                nVars = idx_col * cholmanager.ldm;
                smax = memspace.workspace_float.data[idx_col];
                for (idx = 0; idx < idx_col; idx++) {
                  smax -= cholmanager.FMat.data[nVars + idx] *
                          memspace.workspace_float.data[idx];
                }
                memspace.workspace_float.data[idx_col] =
                    smax / cholmanager.FMat.data[nVars + idx_col];
              }
            }
            scalarLB = cholmanager.ndims;
            if ((cholmanager.FMat.size[0] != 0) &&
                (cholmanager.FMat.size[1] != 0) && (cholmanager.ndims != 0)) {
              for (idx_col = scalarLB; idx_col >= 1; idx_col--) {
                nVars = (idx_col + (idx_col - 1) * cholmanager.ldm) - 1;
                memspace.workspace_float.data[idx_col - 1] /=
                    cholmanager.FMat.data[nVars];
                for (idx = 0; idx <= idx_col - 2; idx++) {
                  ix = (idx_col - idx) - 2;
                  memspace.workspace_float.data[ix] -=
                      memspace.workspace_float.data[idx_col - 1] *
                      cholmanager.FMat.data[(nVars - idx) - 1];
                }
              }
            }
          } else {
            scalarLB = cholmanager.ndims - 2;
            if ((cholmanager.FMat.size[0] != 0) &&
                (cholmanager.FMat.size[1] != 0) && (cholmanager.ndims != 0)) {
              for (idx_col = 0; idx_col <= scalarLB + 1; idx_col++) {
                nVars = idx_col + idx_col * cholmanager.ldm;
                i = scalarLB - idx_col;
                for (idx = 0; idx <= i; idx++) {
                  ix = (idx_col + idx) + 1;
                  memspace.workspace_float.data[ix] -=
                      memspace.workspace_float.data[idx_col] *
                      cholmanager.FMat.data[(nVars + idx) + 1];
                }
              }
            }
            i = cholmanager.ndims;
            for (idx = 0; idx < i; idx++) {
              memspace.workspace_float.data[idx] /=
                  cholmanager.FMat.data[idx + cholmanager.ldm * idx];
            }
            scalarLB = cholmanager.ndims;
            if ((cholmanager.FMat.size[0] != 0) &&
                (cholmanager.FMat.size[1] != 0) && (cholmanager.ndims != 0)) {
              for (idx_col = scalarLB; idx_col >= 1; idx_col--) {
                nVars = (idx_col - 1) * cholmanager.ldm;
                smax = memspace.workspace_float.data[idx_col - 1];
                i = idx_col + 1;
                for (idx = scalarLB; idx >= i; idx--) {
                  smax -= cholmanager.FMat.data[(nVars + idx) - 1] *
                          memspace.workspace_float.data[idx - 1];
                }
                memspace.workspace_float.data[idx_col - 1] = smax;
              }
            }
          }
          if (qrmanager.mrows != 0) {
            std::memset(&solution.searchDir.data[0], 0,
                        static_cast<unsigned int>(nVar_tmp + 1) *
                            sizeof(double));
            ix = 0;
            i = nullStartIdx_tmp + qrmanager.ldq * (mNull_tmp - 1);
            for (idx = nullStartIdx_tmp; idx_row < 0 ? idx >= i : idx <= i;
                 idx += idx_row) {
              idx_col = idx + nVar_tmp;
              for (int ia{idx}; ia <= idx_col; ia++) {
                nVars = ia - idx;
                solution.searchDir.data[nVars] +=
                    qrmanager.Q.data[ia - 1] *
                    memspace.workspace_float.data[ix];
              }
              ix++;
            }
          }
        }
      }
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for compute_deltax.cpp
//
// [EOF]
//
