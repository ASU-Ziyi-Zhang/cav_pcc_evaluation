//
// File: find.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef FIND_H
#define FIND_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
void binary_expand_op_3(int in1_data[], const double in2_data[],
                        const int in2_size[2], const double in3_data[],
                        const int in3_size[2], const double in4[50], int in6,
                        int in7, int in1_size[2]);

void binary_expand_op_8(int in1_data[], const double in2_data[], int in3,
                        int in4, int in5, const double in6_data[],
                        const int in6_size[2], const double in7_data[],
                        const int in7_size[2], const double in8_data[],
                        const int in8_size[2], int in1_size[2]);

namespace coder {
void b_eml_find(const boolean_T x[12], int i_data[], int i_size[2]);

void c_eml_find(const boolean_T x_data[], const int x_size[2], int i_data[],
                int i_size[2]);

void d_eml_find(const boolean_T x[1000], int i_data[], int i_size[2]);

void eml_find(const boolean_T x[10], int i_data[], int i_size[2]);

} // namespace coder

#endif
//
// File trailer for find.h
//
// [EOF]
//
