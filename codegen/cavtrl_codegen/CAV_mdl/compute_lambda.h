//
// File: compute_lambda.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef COMPUTE_LAMBDA_H
#define COMPUTE_LAMBDA_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct i_struct_T;

struct g_struct_T;

struct e_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void compute_lambda(double workspace_data[], i_struct_T &solution,
                    const g_struct_T &objective, const e_struct_T &qrmanager);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for compute_lambda.h
//
// [EOF]
//
