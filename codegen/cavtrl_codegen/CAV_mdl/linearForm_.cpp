//
// File: linearForm_.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "linearForm_.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// Arguments    : boolean_T obj_hasLinear
//                int obj_nvar
//                double workspace_data[]
//                const double H_data[]
//                const double f_data[]
//                const double x_data[]
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
void linearForm_(boolean_T obj_hasLinear, int obj_nvar, double workspace_data[],
                 const double H_data[], const double f_data[],
                 const double x_data[])
{
  int beta1;
  beta1 = 0;
  if (obj_hasLinear) {
    beta1 = static_cast<unsigned char>(obj_nvar);
    if (beta1 - 1 >= 0) {
      std::copy(&f_data[0], &f_data[beta1], &workspace_data[0]);
    }
    beta1 = 1;
  }
  if (obj_nvar != 0) {
    int ix;
    if (beta1 != 1) {
      beta1 = static_cast<unsigned char>(obj_nvar);
      std::memset(&workspace_data[0], 0,
                  static_cast<unsigned int>(beta1) * sizeof(double));
    }
    ix = 0;
    beta1 = obj_nvar * (obj_nvar - 1) + 1;
    for (int iac{1}; obj_nvar < 0 ? iac >= beta1 : iac <= beta1;
         iac += obj_nvar) {
      double c;
      int i;
      c = 0.5 * x_data[ix];
      i = (iac + obj_nvar) - 1;
      for (int ia{iac}; ia <= i; ia++) {
        int i1;
        i1 = ia - iac;
        workspace_data[i1] += H_data[ia - 1] * c;
      }
      ix++;
    }
  }
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for linearForm_.cpp
//
// [EOF]
//
