//
// File: step.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef STEP_H
#define STEP_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct i_struct_T;

struct struct_T;

struct h_struct_T;

struct j_struct_T;

struct e_struct_T;

struct f_struct_T;

struct g_struct_T;

struct l_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
boolean_T b_step(int &STEP_TYPE, double Hessian_data[],
                 const int Hessian_size[2], const double lb_data[],
                 const int lb_size[2], const double ub_data[],
                 const int ub_size[2], i_struct_T &b_TrialState,
                 struct_T &MeritFunction, h_struct_T &memspace,
                 j_struct_T &WorkingSet, e_struct_T &b_QRManager,
                 f_struct_T &b_CholManager, g_struct_T &QPObjective,
                 l_struct_T &qpoptions);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for step.h
//
// [EOF]
//
