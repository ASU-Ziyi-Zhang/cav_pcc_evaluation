//
// File: test_exit.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef TEST_EXIT_H
#define TEST_EXIT_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct_T;

struct j_struct_T;

struct i_struct_T;

struct b_struct_T;

struct h_struct_T;

struct e_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void b_test_exit(b_struct_T &Flags, h_struct_T &memspace,
                 struct_T &MeritFunction, int fscales_lineq_constraint_size,
                 const j_struct_T &WorkingSet, i_struct_T &b_TrialState,
                 e_struct_T &b_QRManager, const double lb_data[],
                 const double ub_data[],
                 int runTimeOptions_MaxFunctionEvaluations);

boolean_T test_exit(struct_T &MeritFunction, const j_struct_T &WorkingSet,
                    i_struct_T &b_TrialState, const double lb_data[],
                    const double ub_data[],
                    int runTimeOptions_MaxFunctionEvaluations,
                    boolean_T &Flags_fevalOK, boolean_T &Flags_done,
                    boolean_T &Flags_stepAccepted,
                    boolean_T &Flags_failedLineSearch, int &Flags_stepType);

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for test_exit.h
//
// [EOF]
//
