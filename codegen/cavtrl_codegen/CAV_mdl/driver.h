//
// File: driver.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef DRIVER_H
#define DRIVER_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct i_struct_T;

struct struct_T;

namespace coder {
namespace internal {
class i_stickyStruct;

}
} // namespace coder
struct m_struct_T;

struct h_struct_T;

struct j_struct_T;

struct e_struct_T;

struct f_struct_T;

struct g_struct_T;

struct k_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void driver(double Hessian_data[], const int Hessian_size[2],
            const double bineq_data[], const double lb_data[],
            const int lb_size[2], const double ub_data[], const int ub_size[2],
            i_struct_T &b_TrialState, struct_T &MeritFunction,
            const ::coder::internal::i_stickyStruct &FcnEvaluator,
            m_struct_T &FiniteDifferences, h_struct_T &memspace,
            j_struct_T &WorkingSet, e_struct_T &b_QRManager,
            f_struct_T &b_CholManager, g_struct_T &QPObjective,
            int fscales_lineq_constraint_size,
            const k_struct_T &runTimeOptions);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for driver.h
//
// [EOF]
//
