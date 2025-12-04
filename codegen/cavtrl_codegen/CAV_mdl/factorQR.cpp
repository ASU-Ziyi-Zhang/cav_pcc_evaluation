//
// File: factorQR.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "factorQR.h"
#include "CAV_ctrl_mdl_wTraJ_241219_data.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include "coder_bounded_array.h"
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// Arguments    : e_struct_T &obj
//                const double A_data[]
//                int mrows
//                int ncols
//                int ldA
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void factorQR(e_struct_T &obj, const double A_data[], int mrows, int ncols,
              int ldA)
{
  int i;
  int ix0;
  int minmana;
  boolean_T guard1;
  i = mrows * ncols;
  guard1 = false;
  if (i > 0) {
    for (int idx{0}; idx < ncols; idx++) {
      ix0 = ldA * idx;
      minmana = obj.ldq * idx;
      i = static_cast<unsigned char>(mrows);
      for (int k{0}; k < i; k++) {
        obj.QR.data[minmana + k] = A_data[ix0 + k];
      }
    }
    guard1 = true;
  } else if (i == 0) {
    obj.mrows = mrows;
    obj.ncols = ncols;
    obj.minRowCol = 0;
  } else {
    guard1 = true;
  }
  if (guard1) {
    obj.usedPivoting = false;
    obj.mrows = mrows;
    obj.ncols = ncols;
    ix0 = (ncols / 4) << 2;
    minmana = ix0 - 4;
    for (int idx{0}; idx <= minmana; idx += 4) {
      _mm_storeu_si128(
          (__m128i *)&obj.jpvt.data[idx],
          _mm_add_epi32(_mm_add_epi32(_mm_set1_epi32(idx),
                                      _mm_loadu_si128((const __m128i *)&iv[0])),
                        _mm_set1_epi32(1)));
    }
    for (int idx{ix0}; idx < ncols; idx++) {
      obj.jpvt.data[idx] = idx + 1;
    }
    if (mrows <= ncols) {
      i = mrows;
    } else {
      i = ncols;
    }
    obj.minRowCol = i;
    ix0 = obj.QR.size[0];
    minmana = obj.QR.size[1];
    if (ix0 <= minmana) {
      minmana = ix0;
    }
    obj.tau.size[0] = minmana;
    if (minmana - 1 >= 0) {
      std::memset(&obj.tau.data[0], 0,
                  static_cast<unsigned int>(minmana) * sizeof(double));
    }
    if ((obj.QR.size[0] != 0) && (obj.QR.size[1] != 0) && (i >= 1)) {
      internal::reflapack::qrf(obj.QR.data, obj.QR.size, mrows, ncols, i,
                               obj.tau.data);
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for factorQR.cpp
//
// [EOF]
//
