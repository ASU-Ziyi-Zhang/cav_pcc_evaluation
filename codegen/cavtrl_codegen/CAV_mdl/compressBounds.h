//
// File: compressBounds.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef COMPRESSBOUNDS_H
#define COMPRESSBOUNDS_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
int compressBounds(int nVar, int indexLB_data[], int indexUB_data[],
                   int indexFixed_data[], const double lb_data[],
                   const int lb_size[2], const double ub_data[],
                   const int ub_size[2], int &mUB, int &mFixed);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for compressBounds.h
//
// [EOF]
//
