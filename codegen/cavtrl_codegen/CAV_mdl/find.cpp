//
// File: find.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "find.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
//
// Arguments    : int in1_data[]
//                const double in2_data[]
//                const int in2_size[2]
//                const double in3_data[]
//                const int in3_size[2]
//                const double in4[50]
//                int in6
//                int in7
//                int in1_size[2]
// Return Type  : void
//
void binary_expand_op_3(int in1_data[], const double in2_data[],
                        const int in2_size[2], const double in3_data[],
                        const int in3_size[2], const double in4[50], int in6,
                        int in7, int in1_size[2])
{
  int b_in2_size[2];
  int b_loop_ub;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  boolean_T b_in2_data[12];
  boolean_T in4_data[11];
  loop_ub = in7 - in6;
  for (int i{0}; i <= loop_ub; i++) {
    in4_data[i] = (in4[5 * (in6 + i) + 3] == 1.0);
  }
  in4_data[loop_ub + 1] = false;
  b_in2_size[0] = 1;
  if (loop_ub + 2 == 1) {
    if (in3_size[1] == 1) {
      b_loop_ub = in2_size[1];
    } else {
      b_loop_ub = in3_size[1];
    }
  } else {
    b_loop_ub = loop_ub + 2;
  }
  b_in2_size[1] = b_loop_ub;
  stride_0_1 = (in2_size[1] != 1);
  stride_1_1 = (in3_size[1] != 1);
  loop_ub = (loop_ub + 2 != 1);
  for (int i{0}; i < b_loop_ub; i++) {
    b_in2_data[i] =
        ((in2_data[i * stride_0_1] / in3_data[i * stride_1_1] < 4.0) &&
         in4_data[i * loop_ub]);
  }
  coder::c_eml_find(b_in2_data, b_in2_size, in1_data, in1_size);
}

//
// Arguments    : int in1_data[]
//                const double in2_data[]
//                int in3
//                int in4
//                int in5
//                const double in6_data[]
//                const int in6_size[2]
//                const double in7_data[]
//                const int in7_size[2]
//                const double in8_data[]
//                const int in8_size[2]
//                int in1_size[2]
// Return Type  : void
//
void binary_expand_op_8(int in1_data[], const double in2_data[], int in3,
                        int in4, int in5, const double in6_data[],
                        const int in6_size[2], const double in7_data[],
                        const int in7_size[2], const double in8_data[],
                        const int in8_size[2], int in1_size[2])
{
  int in2_size[2];
  int i;
  int loop_ub;
  int stride_0_1_tmp;
  int stride_1_1_tmp;
  int stride_2_1_tmp;
  int stride_3_1_tmp;
  int stride_6_1_tmp;
  boolean_T b_in2_data[25];
  in2_size[0] = 1;
  i = (in5 - in4) + 1;
  if (i == 1) {
    loop_ub = in3 + 1;
  } else {
    loop_ub = i;
  }
  if (in8_size[1] == 1) {
    stride_0_1_tmp = loop_ub;
  } else {
    stride_0_1_tmp = in8_size[1];
  }
  if (stride_0_1_tmp == 1) {
    if (in7_size[1] == 1) {
      stride_0_1_tmp = in6_size[1];
    } else {
      stride_0_1_tmp = in7_size[1];
    }
  }
  if (stride_0_1_tmp != 1) {
    loop_ub = stride_0_1_tmp;
  }
  in2_size[1] = loop_ub;
  stride_0_1_tmp = (in3 + 1 != 1);
  stride_1_1_tmp = (i != 1);
  stride_2_1_tmp = (in6_size[1] != 1);
  stride_3_1_tmp = (in7_size[1] != 1);
  stride_6_1_tmp = (in8_size[1] != 1);
  for (i = 0; i < loop_ub; i++) {
    double b_in2_tmp;
    double c_in2_tmp;
    double d_in2_tmp;
    double e_in2_tmp;
    double in2_tmp;
    in2_tmp = in2_data[i * stride_0_1_tmp];
    b_in2_tmp = in2_data[in4 + i * stride_1_1_tmp];
    c_in2_tmp = in6_data[i * stride_2_1_tmp];
    d_in2_tmp = in7_data[i * stride_3_1_tmp];
    e_in2_tmp = in8_data[i * stride_6_1_tmp];
    b_in2_data[i] = (((in2_tmp > 0.0) && (b_in2_tmp == 0.0) &&
                      (-6.0 * c_in2_tmp / d_in2_tmp +
                           2.0 * (in2_tmp + 2.0 * b_in2_tmp) / e_in2_tmp >
                       0.0)) ||
                     ((in2_tmp == 0.0) && (b_in2_tmp > 0.0) &&
                      (6.0 * c_in2_tmp / d_in2_tmp -
                           2.0 * (2.0 * in2_tmp + b_in2_tmp) / e_in2_tmp <
                       0.0)));
  }
  coder::c_eml_find(b_in2_data, in2_size, in1_data, in1_size);
}

//
// Arguments    : const boolean_T x[12]
//                int i_data[]
//                int i_size[2]
// Return Type  : void
//
namespace coder {
void b_eml_find(const boolean_T x[12], int i_data[], int i_size[2])
{
  int idx;
  int ii;
  boolean_T exitg1;
  idx = 0;
  i_size[0] = 1;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 12)) {
    if (x[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= 12) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (idx < 1) {
    i_size[1] = 0;
  } else {
    i_size[1] = idx;
  }
}

//
// Arguments    : const boolean_T x_data[]
//                const int x_size[2]
//                int i_data[]
//                int i_size[2]
// Return Type  : void
//
void c_eml_find(const boolean_T x_data[], const int x_size[2], int i_data[],
                int i_size[2])
{
  int idx;
  int ii;
  int nx_tmp;
  boolean_T exitg1;
  nx_tmp = x_size[1];
  idx = 0;
  i_size[0] = 1;
  i_size[1] = x_size[1];
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii <= nx_tmp - 1)) {
    if (x_data[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= nx_tmp) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (x_size[1] == 1) {
    if (idx == 0) {
      i_size[0] = 1;
      i_size[1] = 0;
    }
  } else if (idx < 1) {
    i_size[1] = 0;
  } else {
    i_size[1] = idx;
  }
}

//
// Arguments    : const boolean_T x[1000]
//                int i_data[]
//                int i_size[2]
// Return Type  : void
//
void d_eml_find(const boolean_T x[1000], int i_data[], int i_size[2])
{
  int idx;
  int ii;
  boolean_T exitg1;
  idx = 0;
  i_size[0] = 1;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 1000)) {
    if (x[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= 1000) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (idx < 1) {
    i_size[1] = 0;
  } else {
    i_size[1] = idx;
  }
}

//
// Arguments    : const boolean_T x[10]
//                int i_data[]
//                int i_size[2]
// Return Type  : void
//
void eml_find(const boolean_T x[10], int i_data[], int i_size[2])
{
  int idx;
  int ii;
  boolean_T exitg1;
  idx = 0;
  i_size[0] = 1;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 10)) {
    if (x[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= 10) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (idx < 1) {
    i_size[1] = 0;
  } else {
    i_size[1] = idx;
  }
}

} // namespace coder

//
// File trailer for find.cpp
//
// [EOF]
//
