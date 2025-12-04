//
// File: computeQ_.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "computeQ_.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "xzlarf.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// Arguments    : e_struct_T &obj
//                int nrows
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void computeQ_(e_struct_T &obj, int nrows)
{
  double work_data[49];
  int i;
  int iQR0;
  int iaii;
  int k;
  int lda;
  int m;
  k = obj.minRowCol;
  for (iaii = 0; iaii < k; iaii++) {
    iQR0 = obj.ldq * iaii + iaii;
    i = obj.mrows - iaii;
    if (i - 2 >= 0) {
      std::copy(&obj.QR.data[iQR0 + 1],
                &obj.QR.data[iQR0 + ((i + iQR0) - iQR0)],
                &obj.Q.data[iQR0 + 1]);
    }
  }
  m = obj.mrows;
  lda = obj.ldq;
  if (nrows >= 1) {
    int b_i;
    int itau;
    b_i = nrows - 1;
    for (int j{k}; j <= b_i; j++) {
      iQR0 = j * lda;
      i = m - 1;
      if (i >= 0) {
        std::memset(&obj.Q.data[iQR0], 0,
                    static_cast<unsigned int>(((i + iQR0) - iQR0) + 1) *
                        sizeof(double));
      }
      obj.Q.data[iQR0 + j] = 1.0;
    }
    itau = obj.minRowCol - 1;
    iQR0 = obj.Q.size[1];
    if (iQR0 - 1 >= 0) {
      std::memset(&work_data[0], 0,
                  static_cast<unsigned int>(iQR0) * sizeof(double));
    }
    for (i = obj.minRowCol; i >= 1; i--) {
      iaii = i + (i - 1) * lda;
      if (i < nrows) {
        obj.Q.data[iaii - 1] = 1.0;
        internal::reflapack::xzlarf((m - i) + 1, nrows - i, iaii,
                                    obj.tau.data[itau], obj.Q.data, iaii + lda,
                                    lda, work_data);
      }
      if (i < m) {
        iQR0 = iaii + 1;
        b_i = (iaii + m) - i;
        for (k = iQR0; k <= b_i; k++) {
          obj.Q.data[k - 1] *= -obj.tau.data[itau];
        }
      }
      obj.Q.data[iaii - 1] = 1.0 - obj.tau.data[itau];
      b_i = static_cast<unsigned char>(i - 1);
      for (int j{0}; j < b_i; j++) {
        obj.Q.data[(iaii - j) - 2] = 0.0;
      }
      itau--;
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for computeQ_.cpp
//
// [EOF]
//
