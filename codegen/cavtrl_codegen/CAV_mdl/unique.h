//
// File: unique.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef UNIQUE_H
#define UNIQUE_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
int count_nonfinites(const double b_data[], int nb, int &nFinite, int &nPInf,
                     int &nNaN);

int unique_vector(const double a[1000], double b_data[], int b_size[2],
                  int ndx_data[], int pos[1000]);

} // namespace coder

#endif
//
// File trailer for unique.h
//
// [EOF]
//
