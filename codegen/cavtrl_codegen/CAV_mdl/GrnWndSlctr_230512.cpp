//
// File: GrnWndSlctr_230512.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "GrnWndSlctr_230512.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : double in1[48]
//                int in2
//                const double in3_data[]
//                const int in3_size[2]
//                double in4
// Return Type  : void
//
void binary_expand_op(double in1[48], int in2, const double in3_data[],
                      const int in3_size[2], double in4)
{
  int stride_0_1;
  stride_0_1 = (in3_size[1] != 1);
  for (int i{0}; i < in2; i++) {
    in1[(i << 2) + 2] = in3_data[i * stride_0_1] / in4;
  }
}

//
// Arguments    : double v0s
//                double vfs
//                double SpdDes
//                double dist_s
//                double aDes
//                double bDes
// Return Type  : double
//
double tfs_des_cal_fcn(double v0s, double vfs, double SpdDes, double dist_s,
                       double aDes, double bDes)
{
  double out;
  if (vfs != 0.0) {
    out = 2.0 * dist_s / (v0s + vfs);
  } else {
    double tf_brk;
    double z1_des;
    double z3_des;
    //  case including a stop sign
    //          dist_stp_req = max(0, (SpdDes^2)/2/bDes);
    z3_des = v0s * v0s;
    tf_brk = z3_des / 2.0;
    z1_des = tf_brk / bDes;
    if (dist_s > std::fmax(0.0, z1_des)) {
      //  (bres < bDes && bres > 0)
      tf_brk = std::sqrt((dist_s + (tf_brk / aDes + vfs * vfs / 2.0 / bDes)) /
                         (0.5 / aDes + 0.5 / bDes));
      if (v0s > tf_brk) {
        //  hold cruising
        out = 3.0 * dist_s / (2.0 * v0s + vfs);
      } else {
        //  accelerating-(cruising)-braking
        if (tf_brk > SpdDes) {
          //  3 intervals
          z1_des = std::fmax(0.0, (SpdDes - v0s) / aDes);
          tf_brk = SpdDes * SpdDes;
          tf_brk = ((dist_s - (tf_brk - z3_des) / 2.0 / aDes) -
                    tf_brk / 2.0 / bDes) /
                   SpdDes;
          z3_des = -(vfs - SpdDes) / bDes;
        } else {
          //  2 intervals
          z1_des = std::fmax(0.0, (tf_brk - v0s) / aDes);
          //  avoid nan value coming from zero aDes
          tf_brk = -(vfs - tf_brk) / bDes;
          z3_des = 0.0;
        }
        out = (z1_des + tf_brk) + z3_des;
      }
      //              out = 2*dist_s/(SpdDes + vfs);
    } else {
      //  braking regime
      //  calculate the fixed af for braking
      tf_brk = 3.0 * z1_des / (2.0 * v0s + vfs);
      z3_des = v0s + 2.0 * vfs;
      tf_brk = -6.0 * z1_des / (tf_brk * tf_brk) + 2.0 * z3_des / tf_brk;
      //  calculate the shrunken time horizon using af
      if (std::abs(tf_brk) < 0.01) {
        tf_brk = 3.0 * dist_s / z3_des;
      } else {
        tf_brk = (z3_des - std::sqrt(std::fmax(
                               z3_des * z3_des - 6.0 * tf_brk * dist_s, 0.0))) /
                 tf_brk;
      }
      out = std::fmax(1.0, tf_brk);
    }
  }
  return out;
}

//
// File trailer for GrnWndSlctr_230512.cpp
//
// [EOF]
//
