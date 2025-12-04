//
// File: sortLambdaQP.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "sortLambdaQP.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
//
// Arguments    : double lambda_data[]
//                int WorkingSet_nActiveConstr
//                const int WorkingSet_sizes[5]
//                const int WorkingSet_isActiveIdx[6]
//                const int WorkingSet_Wid_data[]
//                const int WorkingSet_Wlocalidx_data[]
//                double workspace_data[]
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace parseoutput {
void sortLambdaQP(double lambda_data[], int WorkingSet_nActiveConstr,
                  const int WorkingSet_sizes[5],
                  const int WorkingSet_isActiveIdx[6],
                  const int WorkingSet_Wid_data[],
                  const int WorkingSet_Wlocalidx_data[],
                  double workspace_data[])
{
  if (WorkingSet_nActiveConstr != 0) {
    int idx;
    int idxOffset;
    int mAll;
    mAll = ((WorkingSet_sizes[0] + WorkingSet_sizes[3]) + WorkingSet_sizes[4]) +
           WorkingSet_sizes[2];
    idx = static_cast<unsigned char>(mAll);
    if (idx - 1 >= 0) {
      std::copy(&lambda_data[0], &lambda_data[idx], &workspace_data[0]);
    }
    if (mAll - 1 >= 0) {
      std::memset(&lambda_data[0], 0,
                  static_cast<unsigned int>(mAll) * sizeof(double));
    }
    mAll = 0;
    idx = 0;
    while ((idx + 1 <= WorkingSet_nActiveConstr) &&
           (WorkingSet_Wid_data[idx] <= 2)) {
      if (WorkingSet_Wid_data[idx] == 1) {
        idxOffset = 1;
      } else {
        idxOffset = WorkingSet_isActiveIdx[1];
      }
      lambda_data[(idxOffset + WorkingSet_Wlocalidx_data[idx]) - 2] =
          workspace_data[mAll];
      mAll++;
      idx++;
    }
    while (idx + 1 <= WorkingSet_nActiveConstr) {
      switch (WorkingSet_Wid_data[idx]) {
      case 3:
        idxOffset = WorkingSet_isActiveIdx[2];
        break;
      case 4:
        idxOffset = WorkingSet_isActiveIdx[3];
        break;
      default:
        idxOffset = WorkingSet_isActiveIdx[4];
        break;
      }
      lambda_data[(idxOffset + WorkingSet_Wlocalidx_data[idx]) - 2] =
          workspace_data[mAll];
      mAll++;
      idx++;
    }
  }
}

} // namespace parseoutput
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for sortLambdaQP.cpp
//
// [EOF]
//
