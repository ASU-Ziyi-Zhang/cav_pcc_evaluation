//
// File: solve.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "solve.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_bounded_array.h"
#include <cstring>

// Function Definitions
//
// Arguments    : const f_struct_T &obj
//                double rhs_data[]
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace CholManager {
void solve(const f_struct_T &obj, double rhs_data[])
{
  int jA;
  int n_tmp;
  boolean_T b;
  n_tmp = obj.ndims;
  b = ((obj.FMat.size[0] == 0) || (obj.FMat.size[1] == 0));
  if ((!b) && (obj.ndims != 0)) {
    for (int j{0}; j < n_tmp; j++) {
      double temp;
      jA = j * obj.ldm;
      temp = rhs_data[j];
      for (int i{0}; i < j; i++) {
        temp -= obj.FMat.data[jA + i] * rhs_data[i];
      }
      rhs_data[j] = temp / obj.FMat.data[jA + j];
    }
  }
  if ((!b) && (obj.ndims != 0)) {
    for (int j{n_tmp}; j >= 1; j--) {
      jA = (j + (j - 1) * obj.ldm) - 1;
      rhs_data[j - 1] /= obj.FMat.data[jA];
      for (int i{0}; i <= j - 2; i++) {
        int ix;
        ix = (j - i) - 2;
        rhs_data[ix] -= rhs_data[j - 1] * obj.FMat.data[(jA - i) - 1];
      }
    }
  }
}

} // namespace CholManager
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for solve.cpp
//
// [EOF]
//
