//
// File: fullColLDL2_.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "fullColLDL2_.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : f_struct_T &obj
//                int NColsRemain
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace DynamicRegCholManager {
void fullColLDL2_(f_struct_T &obj, int NColsRemain)
{
  int LDimSizeP1;
  LDimSizeP1 = obj.ldm;
  for (int k{0}; k < NColsRemain; k++) {
    __m128d r;
    double alpha1;
    double y;
    int LD_diagOffset;
    int i;
    int jA;
    int offset1;
    int scalarLB;
    int subMatrixDim;
    int vectorUB;
    LD_diagOffset = (LDimSizeP1 + 1) * k;
    alpha1 = -1.0 / obj.FMat.data[LD_diagOffset];
    subMatrixDim = NColsRemain - k;
    offset1 = LD_diagOffset + 2;
    y = obj.workspace_;
    for (jA = 0; jA <= subMatrixDim - 2; jA++) {
      y = obj.FMat.data[(LD_diagOffset + jA) + 1];
    }
    obj.workspace_ = y;
    if (!(alpha1 == 0.0)) {
      jA = LD_diagOffset + LDimSizeP1;
      for (int j{0}; j <= subMatrixDim - 2; j++) {
        if (y != 0.0) {
          double temp;
          int i1;
          temp = y * alpha1;
          i = jA + 2;
          i1 = subMatrixDim + jA;
          scalarLB = (((((i1 - jA) - 1) / 2) << 1) + jA) + 2;
          vectorUB = scalarLB - 2;
          for (int ijA{i}; ijA <= vectorUB; ijA += 2) {
            r = _mm_loadu_pd(&obj.FMat.data[ijA - 1]);
            _mm_storeu_pd(&obj.FMat.data[ijA - 1],
                          _mm_add_pd(r, _mm_set1_pd(y * temp)));
          }
          for (int ijA{scalarLB}; ijA <= i1; ijA++) {
            obj.FMat.data[ijA - 1] += y * temp;
          }
        }
        jA += obj.ldm;
      }
    }
    alpha1 = 1.0 / obj.FMat.data[LD_diagOffset];
    i = LD_diagOffset + subMatrixDim;
    scalarLB = (((((i - LD_diagOffset) - 1) / 2) << 1) + LD_diagOffset) + 2;
    vectorUB = scalarLB - 2;
    for (jA = offset1; jA <= vectorUB; jA += 2) {
      r = _mm_loadu_pd(&obj.FMat.data[jA - 1]);
      _mm_storeu_pd(&obj.FMat.data[jA - 1], _mm_mul_pd(_mm_set1_pd(alpha1), r));
    }
    for (jA = scalarLB; jA <= i; jA++) {
      obj.FMat.data[jA - 1] *= alpha1;
    }
  }
}

} // namespace DynamicRegCholManager
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for fullColLDL2_.cpp
//
// [EOF]
//
