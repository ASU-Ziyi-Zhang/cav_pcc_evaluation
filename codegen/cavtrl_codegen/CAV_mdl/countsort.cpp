//
// File: countsort.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "countsort.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : int x_data[]
//                int xLen
//                int workspace_data[]
//                int xMin
//                int xMax
// Return Type  : void
//
namespace coder {
namespace optim {
namespace coder {
namespace utils {
void countsort(int x_data[], int xLen, int workspace_data[], int xMin, int xMax)
{
  if ((xLen > 1) && (xMax > xMin)) {
    int idxEnd;
    int idxStart;
    int maxOffset;
    idxStart = xMax - xMin;
    if (idxStart >= 0) {
      std::memset(&workspace_data[0], 0,
                  static_cast<unsigned int>(idxStart + 1) * sizeof(int));
    }
    maxOffset = idxStart - 1;
    for (int idx{0}; idx < xLen; idx++) {
      idxStart = x_data[idx] - xMin;
      workspace_data[idxStart]++;
    }
    for (int idx{2}; idx <= maxOffset + 2; idx++) {
      workspace_data[idx - 1] += workspace_data[idx - 2];
    }
    idxStart = 1;
    idxEnd = workspace_data[0];
    for (int idx{0}; idx <= maxOffset; idx++) {
      for (int idxFill{idxStart}; idxFill <= idxEnd; idxFill++) {
        x_data[idxFill - 1] = idx + xMin;
      }
      idxStart = workspace_data[idx] + 1;
      idxEnd = workspace_data[idx + 1];
    }
    for (int idx{idxStart}; idx <= idxEnd; idx++) {
      x_data[idx - 1] = xMax;
    }
  }
}

} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

//
// File trailer for countsort.cpp
//
// [EOF]
//
