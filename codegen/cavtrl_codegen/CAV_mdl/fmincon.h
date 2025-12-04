//
// File: fmincon.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef FMINCON_H
#define FMINCON_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class anonymous_function;

}

// Function Declarations
namespace coder {
double fmincon(const anonymous_function &fun, const double x0_data[],
               const int x0_size[2], const double Aineq_data[],
               const double bineq_data[], const int bineq_size[2],
               const double lb_data[], const int lb_size[2],
               const double ub_data[], const int ub_size[2], double x_data[],
               int x_size[2], double &exitflag, double &output_iterations,
               double &output_funcCount, char output_algorithm[3],
               double &output_constrviolation, double &output_stepsize,
               double &output_lssteplength, double &output_firstorderopt);

}

#endif
//
// File trailer for fmincon.h
//
// [EOF]
//
