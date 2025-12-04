//
// File: feasibleratiotest.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef FEASIBLERATIOTEST_H
#define FEASIBLERATIOTEST_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
double feasibleratiotest(
    const double solution_xstar_data[], const double solution_searchDir_data[],
    double workspace_data[], const int workspace_size[2], int workingset_nVar,
    int workingset_ldA, const double workingset_Aineq_data[],
    const double workingset_bineq_data[], const double workingset_lb_data[],
    const double workingset_ub_data[], const int workingset_indexLB_data[],
    const int workingset_indexUB_data[], const int workingset_sizes[5],
    const int workingset_isActiveIdx[6],
    const boolean_T workingset_isActiveConstr_data[],
    const int workingset_nWConstr[5], boolean_T isPhaseOne,
    boolean_T &newBlocking, int &constrType, int &constrIdx);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
//
// File trailer for feasibleratiotest.h
//
// [EOF]
//
