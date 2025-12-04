//
// File: stickyStruct.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef STICKYSTRUCT_H
#define STICKYSTRUCT_H

// Include Files
#include "anonymous_function.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace internal {
class stickyStruct {
public:
  anonymous_function value;
};

class b_stickyStruct {
public:
  stickyStruct next;
};

class c_stickyStruct {
public:
  b_stickyStruct next;
};

class d_stickyStruct {
public:
  c_stickyStruct next;
};

class e_stickyStruct {
public:
  d_stickyStruct next;
};

class f_stickyStruct {
public:
  e_stickyStruct next;
};

class g_stickyStruct {
public:
  f_stickyStruct next;
};

class h_stickyStruct {
public:
  g_stickyStruct next;
};

class i_stickyStruct {
public:
  h_stickyStruct next;
};

} // namespace internal
} // namespace coder

#endif
//
// File trailer for stickyStruct.h
//
// [EOF]
//
