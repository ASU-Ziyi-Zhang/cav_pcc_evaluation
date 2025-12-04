//
// File: factoryConstruct1.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef FACTORYCONSTRUCT1_H
#define FACTORYCONSTRUCT1_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class anonymous_function;

}
struct m_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
void factoryConstruct(const anonymous_function &objfun, int nVar,
                      const double lb_data[], const int lb_size[2],
                      const double ub_data[], const int ub_size[2],
                      m_struct_T &obj);

}
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for factoryConstruct1.h
//
// [EOF]
//
