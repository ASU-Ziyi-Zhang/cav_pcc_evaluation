//
// File: RefTrjGnrtr_240503.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "RefTrjGnrtr_240503.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types1.h"
#include "CAV_ctrl_mdl_wTraJ_241219_internal_types2.h"
#include "CAV_ctrl_mdl_wTraJ_241219_rtwutil.h"
#include "anonymous_function.h"
#include "diff.h"
#include "find.h"
#include "fmincon.h"
#include "ifWhileCond.h"
#include "ixfun.h"
#include "minOrMax.h"
#include "mldivide.h"
#include "mrdivide_helper.h"
#include "round.h"
#include "rt_nonfinite.h"
#include "sum.h"
#include "unique.h"
#include "coder_bounded_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Declarations
static double OptEntTimeSearch_v2(
    double t0s, double tf0s, double vfs, double v0s, const double si_arr_data[],
    const int si_arr_size[2], const double tmin0_set_data[],
    const int tmin0_set_size[2], const double tmax0_set_data[],
    const int tmax0_set_size[2], const double ViolInfo_idxTLStp_data[],
    const int ViolInfo_idxTLStp_size[2], const double ViolInfo_tint_data[],
    const int ViolInfo_tint_size[2], double topt_data[], int topt_size[2],
    double topt0_data[], int topt0_size[2], double &exitflag);

static void
WHOLE_TRAJ_GEN_fcn1(const double vi_arr_data[], const int vi_arr_size[2],
                    const double si_arr_data[], const double ti_arr_data[],
                    const int ti_arr_size[2], const double Nti_arr_data[],
                    const int Nti_arr_size[2], double TRAJ[4000]);

static void binary_expand_op_1(double in1_data[], int in1_size[2],
                               const double in2_data[], int in3, int in4,
                               int in5, double in6);

static void binary_expand_op_6(double in1_data[], int in1_size[2],
                               const double in2_data[], const int in2_size[2],
                               const double in3_data[], const int in3_size[2]);

static void binary_expand_op_7(double in1_data[], int in1_size[2],
                               const double in3_data[], const int in3_size[2],
                               const double in4_data[], const int in4_size[2],
                               const double in5_data[], const int in5_size[2],
                               const double in6_data[], int in7,
                               const double in8_data[], const int in8_size[2],
                               int in9, int in10);

static void binary_expand_op_9(double in1_data[], int in1_size[2],
                               const double in2_data[], int in3, int in4,
                               int in5);

static void jncstate_cal(const double x_data[], const int x_size[2],
                         const double si_arr_data[], const int si_arr_size[2],
                         double t0s, double tf0s, double vfs, double v0s,
                         const double ViolInfo_idxTLStp_data[],
                         const int ViolInfo_idxTLStp_size[2],
                         const double ViolInfo_tint_data[],
                         const int ViolInfo_tint_size[2], double ti_arr_data[],
                         int ti_arr_size[2], double vi_arr_data[],
                         int vi_arr_size[2]);

static int matrix_calc_f(const double z_arr_data[], const int z_arr_size[2],
                         const double l_arr_data[], const int l_arr_size[2],
                         double vfs, double v0s, double A_data[], int A_size[2],
                         double Y_data[]);

static double
tLimCal_fcn(const double tmin0_arr_data[], const int tmin0_arr_size[2],
            const double tmax0_arr_data[], const int tmax0_arr_size[2],
            const double dtreq_arr_data[], const int dtreq_arr_size[2],
            double t0s, double tf0s, double out_tmin0n_arr_data[],
            int out_tmin0n_arr_size[2], double out_tmax0n_arr_data[],
            int out_tmax0n_arr_size[2], double out_tmin0f_arr_data[],
            int out_tmin0f_arr_size[2], double out_tmax0f_arr_data[],
            int out_tmax0f_arr_size[2], double out_dtreq_arr_data[],
            int out_dtreq_arr_size[2], double &out_tfs);

// Function Definitions
//
// x0 = tmin0_set + 1e-2;
//
// Arguments    : double t0s
//                double tf0s
//                double vfs
//                double v0s
//                const double si_arr_data[]
//                const int si_arr_size[2]
//                const double tmin0_set_data[]
//                const int tmin0_set_size[2]
//                const double tmax0_set_data[]
//                const int tmax0_set_size[2]
//                const double ViolInfo_idxTLStp_data[]
//                const int ViolInfo_idxTLStp_size[2]
//                const double ViolInfo_tint_data[]
//                const int ViolInfo_tint_size[2]
//                double topt_data[]
//                int topt_size[2]
//                double topt0_data[]
//                int topt0_size[2]
//                double &exitflag
// Return Type  : double
//
static double OptEntTimeSearch_v2(
    double t0s, double tf0s, double vfs, double v0s, const double si_arr_data[],
    const int si_arr_size[2], const double tmin0_set_data[],
    const int tmin0_set_size[2], const double tmax0_set_data[],
    const int tmax0_set_size[2], const double ViolInfo_idxTLStp_data[],
    const int ViolInfo_idxTLStp_size[2], const double ViolInfo_tint_data[],
    const int ViolInfo_tint_size[2], double topt_data[], int topt_size[2],
    double topt0_data[], int topt0_size[2], double &exitflag)
{
  coder::anonymous_function fun;
  double A_data[144];
  double B_data[12];
  double b_expl_temp;
  double c_expl_temp;
  double d_expl_temp;
  double e_expl_temp;
  double f_expl_temp;
  double g_expl_temp;
  int B_size[2];
  int loop_ub;
  int scalarLB;
  int vectorUB;
  char expl_temp[3];
  if (tmin0_set_size[1] == tmax0_set_size[1]) {
    topt0_size[0] = 1;
    loop_ub = tmin0_set_size[1];
    topt0_size[1] = tmin0_set_size[1];
    scalarLB = (tmin0_set_size[1] / 2) << 1;
    vectorUB = scalarLB - 2;
    for (int i{0}; i <= vectorUB; i += 2) {
      _mm_storeu_pd(&topt0_data[i],
                    _mm_div_pd(_mm_add_pd(_mm_loadu_pd(&tmin0_set_data[i]),
                                          _mm_loadu_pd(&tmax0_set_data[i])),
                               _mm_set1_pd(2.0)));
    }
    for (int i{scalarLB}; i < loop_ub; i++) {
      topt0_data[i] = (tmin0_set_data[i] + tmax0_set_data[i]) / 2.0;
    }
  } else {
    binary_expand_op_6(topt0_data, topt0_size, tmin0_set_data, tmin0_set_size,
                       tmax0_set_data, tmax0_set_size);
  }
  //      x0 = tmax0_set - 1e-2;
  loop_ub = topt0_size[1];
  vectorUB = topt0_size[1];
  scalarLB = topt0_size[1] * topt0_size[1];
  if (scalarLB - 1 >= 0) {
    std::memset(&A_data[0], 0,
                static_cast<unsigned int>(scalarLB) * sizeof(double));
  }
  for (scalarLB = 0; scalarLB <= loop_ub - 2; scalarLB++) {
    A_data[scalarLB + vectorUB * scalarLB] = 1.0;
    A_data[scalarLB + vectorUB * (scalarLB + 1)] = -1.0;
  }
  A_data[(topt0_size[1] + topt0_size[1] * (topt0_size[1] - 1)) - 1] = 1.0;
  B_size[0] = 1;
  B_size[1] = topt0_size[1];
  for (int i{0}; i < loop_ub; i++) {
    B_data[i] = -0.1;
  }
  B_data[topt0_size[1] - 1] = (-0.1 + tf0s) - 5.0;
  //  fmincon solver
  fun.workspace.t0s = t0s;
  fun.workspace.tf0s = tf0s;
  fun.workspace.vfs = vfs;
  fun.workspace.v0s = v0s;
  fun.workspace.si_arr.size[0] = 1;
  loop_ub = si_arr_size[1];
  fun.workspace.si_arr.size[1] = si_arr_size[1];
  if (loop_ub - 1 >= 0) {
    std::copy(&si_arr_data[0], &si_arr_data[loop_ub],
              &fun.workspace.si_arr.data[0]);
  }
  fun.workspace.ViolInfo.idxTLStp.size[0] = 1;
  loop_ub = ViolInfo_idxTLStp_size[1];
  fun.workspace.ViolInfo.idxTLStp.size[1] = ViolInfo_idxTLStp_size[1];
  if (loop_ub - 1 >= 0) {
    std::copy(&ViolInfo_idxTLStp_data[0], &ViolInfo_idxTLStp_data[loop_ub],
              &fun.workspace.ViolInfo.idxTLStp.data[0]);
  }
  fun.workspace.ViolInfo.tint.size[0] = 1;
  loop_ub = ViolInfo_tint_size[1];
  fun.workspace.ViolInfo.tint.size[1] = ViolInfo_tint_size[1];
  if (loop_ub - 1 >= 0) {
    std::copy(&ViolInfo_tint_data[0], &ViolInfo_tint_data[loop_ub],
              &fun.workspace.ViolInfo.tint.data[0]);
  }
  //      options.FiniteDifferenceStepSize = 1e-4;
  //      options.DiffMinChange = 1e-6;
  //      topt = fmincon(fun,x0,A,B,Aeq,Beq,lb,ub,nonlcon,options);
  //      fopt = fun(topt);
  return coder::fmincon(fun, topt0_data, topt0_size, A_data, B_data, B_size,
                        tmin0_set_data, tmin0_set_size, tmax0_set_data,
                        tmax0_set_size, topt_data, topt_size, exitflag,
                        b_expl_temp, c_expl_temp, expl_temp, d_expl_temp,
                        e_expl_temp, f_expl_temp, g_expl_temp);
}

//
// vi_arr: entering speeds at each intersection
//  ti_arr: entering times at each intersection
//  si_arr: intersection positions
//  Nti_arr: the array length of each road segment
//      Nt = sum(Nti_arr);
//
// Arguments    : const double vi_arr_data[]
//                const int vi_arr_size[2]
//                const double si_arr_data[]
//                const double ti_arr_data[]
//                const int ti_arr_size[2]
//                const double Nti_arr_data[]
//                const int Nti_arr_size[2]
//                double TRAJ[4000]
// Return Type  : void
//
static void
WHOLE_TRAJ_GEN_fcn1(const double vi_arr_data[], const int vi_arr_size[2],
                    const double si_arr_data[], const double ti_arr_data[],
                    const int ti_arr_size[2], const double Nti_arr_data[],
                    const int Nti_arr_size[2], double TRAJ[4000])
{
  double b_x_data[1000];
  double out_s_a_arr_data[1000];
  double out_s_s_arr_data[1000];
  double t_arr_data[1000];
  double v_arr_data[1000];
  double y_data[1000];
  double z_data[1000];
  double ind_Nti_arr_data[26];
  double x_data[25];
  double dv[2];
  double dv1[2];
  double dts;
  double sip1a;
  int i;
  int ix;
  dts = (ti_arr_data[ti_arr_size[1] - 1] - ti_arr_data[0]) / 999.0;
  ix = Nti_arr_size[1];
  if (ix - 1 >= 0) {
    std::copy(&Nti_arr_data[0], &Nti_arr_data[ix], &x_data[0]);
  }
  if ((Nti_arr_size[1] != 0) && (Nti_arr_size[1] != 1)) {
    for (int k{0}; k <= ix - 2; k++) {
      x_data[k + 1] += x_data[k];
    }
  }
  ind_Nti_arr_data[0] = 0.0;
  if (ix - 1 >= 0) {
    std::copy(&x_data[0], &x_data[ix], &ind_Nti_arr_data[1]);
  }
  //      Nt = ind_Nti_arr(end);
  for (i = 0; i < 4000; i++) {
    TRAJ[i] = rtNaN;
  }
  sip1a = si_arr_data[0];
  i = vi_arr_size[1];
  for (int b_i{0}; b_i <= i - 2; b_i++) {
    __m128d r;
    double d;
    double p10;
    double s0;
    double tf;
    double ylast;
    int i1;
    int ini_s;
    int t_arr_size_idx_1;
    int t_arr_tmp_tmp;
    int y_size_idx_1;
    short tmp_data[1000];
    //              si = si_arr(i);
    s0 = std::fmax(sip1a, si_arr_data[b_i]);
    //  to avoid non-monotonicity of position due to negative speed
    //              tip1 = ti_arr(i + 1);
    sip1a = ind_Nti_arr_data[b_i + 1] - ind_Nti_arr_data[b_i];
    //  specify upper bound
    //              ind_arr = ind_ini + 1:ind_end;
    if (sip1a < 1.0) {
      y_size_idx_1 = 0;
    } else {
      y_size_idx_1 = static_cast<int>(sip1a - 1.0) + 1;
      ix = static_cast<int>(sip1a - 1.0);
      for (i1 = 0; i1 <= ix; i1++) {
        y_data[i1] = static_cast<double>(i1) + 1.0;
      }
    }
    //  must use constant value vectors
    if (b_i + 1 == 1) {
      sip1a = Nti_arr_data[b_i];
      ini_s = 0;
    } else {
      sip1a = Nti_arr_data[b_i] + 1.0;
      ini_s = 1;
    }
    //  specify upper bound
    d = ti_arr_data[b_i];
    tf = std::fmax(d + dts * 0.9, ti_arr_data[b_i + 1]) - d;
    if (!(sip1a >= 0.0)) {
      t_arr_size_idx_1 = 0;
    } else {
      d = std::floor(sip1a);
      ix = static_cast<int>(std::floor(sip1a));
      t_arr_size_idx_1 = static_cast<int>(d);
      if (static_cast<int>(d) >= 1) {
        t_arr_tmp_tmp = static_cast<int>(d) - 1;
        t_arr_data[ix - 1] = tf;
        if (static_cast<int>(d) >= 2) {
          t_arr_data[0] = 0.0;
          if (static_cast<int>(d) >= 3) {
            if (-tf == 0.0) {
              sip1a = tf / (static_cast<double>(static_cast<int>(d)) - 1.0);
              for (int k{2}; k <= t_arr_tmp_tmp; k++) {
                t_arr_data[k - 1] =
                    (static_cast<double>((k << 1) - static_cast<int>(d)) -
                     1.0) *
                    sip1a;
              }
              if ((static_cast<int>(d) & 1) == 1) {
                t_arr_data[static_cast<int>(d) >> 1] = 0.0;
              }
            } else if ((tf < 0.0) && (std::abs(tf) > 8.9884656743115785E+307)) {
              sip1a = tf / (static_cast<double>(static_cast<int>(d)) - 1.0);
              for (int k{0}; k <= ix - 3; k++) {
                t_arr_data[k + 1] = sip1a * (static_cast<double>(k) + 1.0);
              }
            } else {
              sip1a = tf / (static_cast<double>(static_cast<int>(d)) - 1.0);
              for (int k{0}; k <= ix - 3; k++) {
                t_arr_data[k + 1] = (static_cast<double>(k) + 1.0) * sip1a;
              }
            }
          }
        }
      }
    }
    d = si_arr_data[b_i + 1];
    ylast = vi_arr_data[b_i];
    sip1a = vi_arr_data[b_i + 1];
    p10 = (((12.0 * s0 - 12.0 * d) + 6.0 * tf * ylast) + 6.0 * tf * sip1a) /
          rt_powd_snf(tf, 3.0);
    sip1a = (((6.0 * s0 - 6.0 * d) + 4.0 * tf * ylast) + 2.0 * tf * sip1a) /
            (tf * tf);
    ix = t_arr_size_idx_1 / 2 * 2;
    t_arr_tmp_tmp = ix - 2;
    for (i1 = 0; i1 <= t_arr_tmp_tmp; i1 += 2) {
      __m128d r1;
      __m128d r2;
      r = _mm_loadu_pd(&t_arr_data[i1]);
      r1 = _mm_mul_pd(_mm_set1_pd(p10), r);
      r1 = _mm_sub_pd(r1, _mm_set1_pd(sip1a));
      _mm_storeu_pd(&out_s_a_arr_data[i1], r1);
      r1 = _mm_mul_pd(r, r);
      r1 = _mm_mul_pd(_mm_set1_pd(p10), r1);
      r1 = _mm_div_pd(r1, _mm_set1_pd(2.0));
      r2 = _mm_mul_pd(_mm_set1_pd(sip1a), r);
      r1 = _mm_sub_pd(r1, r2);
      r1 = _mm_add_pd(r1, _mm_set1_pd(ylast));
      _mm_storeu_pd(&dv[0], r1);
      dv1[0] = std::fmax(0.0, dv[0]);
      dv1[1] = std::fmax(0.0, dv[1]);
      r1 = _mm_loadu_pd(&dv1[0]);
      _mm_storeu_pd(&v_arr_data[i1], r1);
      _mm_storeu_pd(&b_x_data[i1], r);
    }
    for (i1 = ix; i1 < t_arr_size_idx_1; i1++) {
      d = t_arr_data[i1];
      out_s_a_arr_data[i1] = p10 * d - sip1a;
      v_arr_data[i1] =
          std::fmax(0.0, (p10 * (d * d) / 2.0 - sip1a * d) + ylast);
      b_x_data[i1] = d;
    }
    for (ix = 0; ix <= t_arr_size_idx_1 - 2; ix++) {
      b_x_data[ix] = b_x_data[ix + 1] - b_x_data[ix];
    }
    if (t_arr_size_idx_1 != 0) {
      sip1a = 0.0;
      ix = -1;
      ylast = v_arr_data[0];
      z_data[0] = 0.0;
      for (int k{0}; k <= t_arr_size_idx_1 - 2; k++) {
        if (t_arr_size_idx_1 == 0) {
          d = v_arr_data[k + 1];
          sip1a += (ylast + d) / 2.0;
        } else {
          ix++;
          d = v_arr_data[k + 1];
          sip1a += b_x_data[ix] * ((ylast + d) / 2.0);
        }
        ylast = d;
        z_data[k + 1] = sip1a;
      }
    }
    ix = (t_arr_size_idx_1 / 2) << 1;
    t_arr_tmp_tmp = ix - 2;
    for (i1 = 0; i1 <= t_arr_tmp_tmp; i1 += 2) {
      r = _mm_loadu_pd(&z_data[i1]);
      _mm_storeu_pd(&out_s_s_arr_data[i1], _mm_add_pd(r, _mm_set1_pd(s0)));
    }
    for (i1 = ix; i1 < t_arr_size_idx_1; i1++) {
      out_s_s_arr_data[i1] = z_data[i1] + s0;
    }
    sip1a = ti_arr_data[b_i];
    for (i1 = 0; i1 <= t_arr_tmp_tmp; i1 += 2) {
      r = _mm_loadu_pd(&t_arr_data[i1]);
      _mm_storeu_pd(&b_x_data[i1], _mm_add_pd(r, _mm_set1_pd(sip1a)));
    }
    for (i1 = ix; i1 < t_arr_size_idx_1; i1++) {
      b_x_data[i1] = t_arr_data[i1] + sip1a;
    }
    sip1a = out_s_s_arr_data[t_arr_size_idx_1 - 1];
    if (ini_s + 1 > t_arr_size_idx_1) {
      i1 = 0;
    } else {
      i1 = ini_s;
    }
    ylast = ind_Nti_arr_data[b_i];
    ix = (y_size_idx_1 / 2) << 1;
    t_arr_tmp_tmp = ix - 2;
    for (int k{0}; k <= t_arr_tmp_tmp; k += 2) {
      r = _mm_loadu_pd(&y_data[k]);
      _mm_storeu_pd(&t_arr_data[k], _mm_add_pd(_mm_set1_pd(ylast), r));
    }
    for (int k{ix}; k < y_size_idx_1; k++) {
      t_arr_data[k] = ylast + y_data[k];
    }
    for (int k{0}; k < y_size_idx_1; k++) {
      TRAJ[static_cast<short>(static_cast<short>(t_arr_data[k]) - 1) << 2] =
          b_x_data[i1 + k];
    }
    if (ini_s + 1 > t_arr_size_idx_1) {
      i1 = 0;
    } else {
      i1 = ini_s;
    }
    for (int k{0}; k < y_size_idx_1; k++) {
      short i2;
      i2 = static_cast<short>(static_cast<short>(t_arr_data[k]) - 1);
      tmp_data[k] = i2;
      TRAJ[(i2 << 2) + 1] = out_s_a_arr_data[i1 + k];
    }
    if (ini_s + 1 > t_arr_size_idx_1) {
      i1 = 0;
    } else {
      i1 = ini_s;
    }
    for (int k{0}; k < y_size_idx_1; k++) {
      TRAJ[(tmp_data[k] << 2) + 2] = v_arr_data[i1 + k];
    }
    if (ini_s + 1 > t_arr_size_idx_1) {
      ini_s = 0;
    }
    for (i1 = 0; i1 < y_size_idx_1; i1++) {
      TRAJ[(tmp_data[i1] << 2) + 3] = out_s_s_arr_data[ini_s + i1];
    }
  }
}

//
// Arguments    : double in1_data[]
//                int in1_size[2]
//                const double in2_data[]
//                int in3
//                int in4
//                int in5
//                double in6
// Return Type  : void
//
static void binary_expand_op_1(double in1_data[], int in1_size[2],
                               const double in2_data[], int in3, int in4,
                               int in5, double in6)
{
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1_size[0] = 1;
  i = (in4 - in3) + 1;
  if (in5 + 1 == 1) {
    loop_ub = i;
  } else {
    loop_ub = in5 + 1;
  }
  in1_size[1] = loop_ub;
  stride_0_1 = (i != 1);
  stride_1_1 = (in5 + 1 != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] =
        (in2_data[in3 + i * stride_0_1] - in2_data[i * stride_1_1]) / in6;
  }
}

//
// Arguments    : double in1_data[]
//                int in1_size[2]
//                const double in2_data[]
//                const int in2_size[2]
//                const double in3_data[]
//                const int in3_size[2]
// Return Type  : void
//
static void binary_expand_op_6(double in1_data[], int in1_size[2],
                               const double in2_data[], const int in2_size[2],
                               const double in3_data[], const int in3_size[2])
{
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1_size[0] = 1;
  if (in3_size[1] == 1) {
    loop_ub = in2_size[1];
  } else {
    loop_ub = in3_size[1];
  }
  in1_size[1] = loop_ub;
  stride_0_1 = (in2_size[1] != 1);
  stride_1_1 = (in3_size[1] != 1);
  for (int i{0}; i < loop_ub; i++) {
    in1_data[i] = (in2_data[i * stride_0_1] + in3_data[i * stride_1_1]) / 2.0;
  }
}

//
// Arguments    : double in1_data[]
//                int in1_size[2]
//                const double in3_data[]
//                const int in3_size[2]
//                const double in4_data[]
//                const int in4_size[2]
//                const double in5_data[]
//                const int in5_size[2]
//                const double in6_data[]
//                int in7
//                const double in8_data[]
//                const int in8_size[2]
//                int in9
//                int in10
// Return Type  : void
//
static void binary_expand_op_7(double in1_data[], int in1_size[2],
                               const double in3_data[], const int in3_size[2],
                               const double in4_data[], const int in4_size[2],
                               const double in5_data[], const int in5_size[2],
                               const double in6_data[], int in7,
                               const double in8_data[], const int in8_size[2],
                               int in9, int in10)
{
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1_tmp;
  int stride_2_1_tmp;
  int stride_3_1_tmp;
  int stride_4_1_tmp;
  int stride_6_1_tmp;
  in1_size[0] = 1;
  i = (in10 - in9) + 1;
  if (in8_size[1] == 1) {
    if (in7 + 1 == 1) {
      loop_ub = in5_size[1];
    } else {
      loop_ub = in7 + 1;
    }
    if (i == 1) {
      stride_0_1 = in5_size[1];
    } else {
      stride_0_1 = i;
    }
  } else {
    loop_ub = in8_size[1];
    stride_0_1 = in8_size[1];
  }
  if (in4_size[1] == 1) {
    stride_1_1_tmp = in7 + 1;
    if (i == 1) {
      stride_2_1_tmp = in7 + 1;
    } else {
      stride_2_1_tmp = i;
    }
    stride_3_1_tmp = i;
  } else {
    stride_1_1_tmp = in4_size[1];
    stride_2_1_tmp = in4_size[1];
    stride_3_1_tmp = in4_size[1];
  }
  if (stride_3_1_tmp == 1) {
    if (stride_2_1_tmp == 1) {
      if (stride_1_1_tmp == 1) {
        if (stride_0_1 == 1) {
          if (loop_ub == 1) {
            if (in4_size[1] == 1) {
              loop_ub = in3_size[1];
            } else {
              loop_ub = in4_size[1];
            }
          }
        } else {
          loop_ub = stride_0_1;
        }
      } else {
        loop_ub = stride_1_1_tmp;
      }
    } else {
      loop_ub = stride_2_1_tmp;
    }
  } else {
    loop_ub = stride_3_1_tmp;
  }
  in1_size[1] = loop_ub;
  stride_0_1 = (in3_size[1] != 1);
  stride_1_1_tmp = (in4_size[1] != 1);
  stride_2_1_tmp = (in5_size[1] != 1);
  stride_3_1_tmp = (in7 + 1 != 1);
  stride_4_1_tmp = (in8_size[1] != 1);
  stride_6_1_tmp = (i != 1);
  for (i = 0; i < loop_ub; i++) {
    double d;
    double d1;
    double d2;
    double d3;
    double d4;
    double varargin_1;
    d = in5_data[i * stride_2_1_tmp];
    d1 = in8_data[i * stride_4_1_tmp];
    d2 = in6_data[i * stride_3_1_tmp];
    d3 = in4_data[i * stride_1_1_tmp];
    d4 = in6_data[in9 + i * stride_6_1_tmp];
    varargin_1 = in3_data[i * stride_0_1];
    in1_data[i] = ((((6.0 * (varargin_1 * varargin_1) / rt_powd_snf(d3, 3.0) -
                      d * d2 / d1) -
                     d * d4 / d1) +
                    2.0 * (d2 * d2) / d3) +
                   2.0 * d2 * d4 / d3) +
                  2.0 * (d4 * d4) / d3;
  }
}

//
// Arguments    : double in1_data[]
//                int in1_size[2]
//                const double in2_data[]
//                int in3
//                int in4
//                int in5
// Return Type  : void
//
static void binary_expand_op_9(double in1_data[], int in1_size[2],
                               const double in2_data[], int in3, int in4,
                               int in5)
{
  int i;
  int loop_ub;
  int stride_0_1;
  int stride_1_1;
  in1_size[0] = 1;
  i = (in4 - in3) + 1;
  if (in5 + 1 == 1) {
    loop_ub = i;
  } else {
    loop_ub = in5 + 1;
  }
  in1_size[1] = loop_ub;
  stride_0_1 = (i != 1);
  stride_1_1 = (in5 + 1 != 1);
  for (i = 0; i < loop_ub; i++) {
    in1_data[i] = in2_data[in3 + i * stride_0_1] - in2_data[i * stride_1_1];
  }
}

//
// Arguments    : const double x_data[]
//                const int x_size[2]
//                const double si_arr_data[]
//                const int si_arr_size[2]
//                double t0s
//                double tf0s
//                double vfs
//                double v0s
//                const double ViolInfo_idxTLStp_data[]
//                const int ViolInfo_idxTLStp_size[2]
//                const double ViolInfo_tint_data[]
//                const int ViolInfo_tint_size[2]
//                double ti_arr_data[]
//                int ti_arr_size[2]
//                double vi_arr_data[]
//                int vi_arr_size[2]
// Return Type  : void
//
static void jncstate_cal(const double x_data[], const int x_size[2],
                         const double si_arr_data[], const int si_arr_size[2],
                         double t0s, double tf0s, double vfs, double v0s,
                         const double ViolInfo_idxTLStp_data[],
                         const int ViolInfo_idxTLStp_size[2],
                         const double ViolInfo_tint_data[],
                         const int ViolInfo_tint_size[2], double ti_arr_data[],
                         int ti_arr_size[2], double vi_arr_data[],
                         int vi_arr_size[2])
{
  double A_data[169];
  double b_A_data[144];
  double l_arr_s_data[37];
  double b_data[25];
  double vi_arr_s_data[15];
  double b_ti_arr_data[14];
  double z_arr_data[14];
  double b_z_arr_data[13];
  double tint_n_data[13];
  double vopt_data[13];
  double Y_data[12];
  double b_tmp_data[12];
  double idxTLStp_s;
  int idx_data[25];
  int iwork_data[25];
  int b_A_size[2];
  int b_si_arr_size[2];
  int b_tmp_size[2];
  int b_z_arr_size[2];
  int tmp_size[2];
  int z_arr_size[2];
  int NTLStp;
  int b_i;
  int i;
  int i2;
  int j;
  int k;
  int khi;
  int n;
  int nb;
  int pEnd;
  int q;
  int qEnd;
  signed char unnamed_idx_1_tmp;
  khi = si_arr_size[1];
  n = si_arr_size[1] + 1;
  if (khi - 1 >= 0) {
    std::memset(&idx_data[0], 0, static_cast<unsigned int>(khi) * sizeof(int));
  }
  i = si_arr_size[1] - 1;
  for (k = 1; k <= i; k += 2) {
    idxTLStp_s = si_arr_data[k];
    if ((si_arr_data[k - 1] <= idxTLStp_s) || std::isnan(idxTLStp_s)) {
      idx_data[k - 1] = k;
      idx_data[k] = k + 1;
    } else {
      idx_data[k - 1] = k + 1;
      idx_data[k] = k;
    }
  }
  if ((si_arr_size[1] & 1) != 0) {
    idx_data[si_arr_size[1] - 1] = si_arr_size[1];
  }
  b_i = 2;
  while (b_i < n - 1) {
    i2 = b_i << 1;
    j = 1;
    for (pEnd = b_i + 1; pEnd < n; pEnd = qEnd + b_i) {
      nb = j;
      q = pEnd - 1;
      qEnd = j + i2;
      if (qEnd > n) {
        qEnd = n;
      }
      k = 0;
      NTLStp = qEnd - j;
      while (k + 1 <= NTLStp) {
        idxTLStp_s = si_arr_data[idx_data[q] - 1];
        i = idx_data[nb - 1];
        if ((si_arr_data[i - 1] <= idxTLStp_s) || std::isnan(idxTLStp_s)) {
          iwork_data[k] = i;
          nb++;
          if (nb == pEnd) {
            while (q + 1 < qEnd) {
              k++;
              iwork_data[k] = idx_data[q];
              q++;
            }
          }
        } else {
          iwork_data[k] = idx_data[q];
          q++;
          if (q + 1 == qEnd) {
            while (nb < pEnd) {
              k++;
              iwork_data[k] = idx_data[nb - 1];
              nb++;
            }
          }
        }
        k++;
      }
      for (k = 0; k < NTLStp; k++) {
        idx_data[(j + k) - 1] = iwork_data[k];
      }
      j = qEnd;
    }
    b_i = i2;
  }
  for (k = 0; k < khi; k++) {
    b_data[k] = si_arr_data[idx_data[k] - 1];
  }
  n = coder::count_nonfinites(b_data, si_arr_size[1], khi, i2, pEnd);
  nb = 0;
  if (n > 0) {
    nb = 1;
  }
  khi += n;
  while (n + 1 <= khi) {
    idxTLStp_s = b_data[n];
    do {
      n++;
    } while (!((n + 1 > khi) || (b_data[n] != idxTLStp_s)));
    nb++;
    b_data[nb - 1] = idxTLStp_s;
  }
  if (i2 > 0) {
    nb++;
  }
  for (j = 0; j < pEnd; j++) {
    nb++;
  }
  if (nb < 1) {
    q = 0;
  } else {
    q = nb;
  }
  //  exclude initial and final position
  unnamed_idx_1_tmp = static_cast<signed char>(si_arr_size[1]);
  ti_arr_size[0] = 1;
  ti_arr_size[1] = static_cast<signed char>(si_arr_size[1]);
  qEnd = static_cast<signed char>(si_arr_size[1]);
  vi_arr_size[0] = 1;
  vi_arr_size[1] = static_cast<signed char>(si_arr_size[1]);
  for (i = 0; i < qEnd; i++) {
    ti_arr_data[i] = rtNaN;
    vi_arr_data[i] = rtNaN;
  }
  NTLStp = ViolInfo_idxTLStp_size[1] + 1;
  tint_n_data[0] = t0s;
  nb = ViolInfo_tint_size[1];
  if (nb - 1 >= 0) {
    std::copy(&ViolInfo_tint_data[0], &ViolInfo_tint_data[nb], &tint_n_data[1]);
  }
  if (ViolInfo_idxTLStp_size[1] > 0) {
    i = ViolInfo_idxTLStp_size[1];
    tmp_size[0] = 1;
    tmp_size[1] = unnamed_idx_1_tmp;
    for (b_i = 0; b_i <= i; b_i++) {
      double idxVal_arr_data[38];
      double idxTLStpPrv_s;
      int l_arr_s_size[2];
      boolean_T tmp_data[25];
      if (b_i + 1 == NTLStp) {
        idxTLStp_s = static_cast<double>(q) - 2.0;
      } else {
        idxTLStp_s = ViolInfo_idxTLStp_data[b_i];
      }
      if (b_i + 1 == 1) {
        //  the first selected TL
        idxTLStpPrv_s = 0.0;
      } else {
        idxTLStpPrv_s = ViolInfo_idxTLStp_data[b_i - 1];
      }
      if (idxTLStpPrv_s + 1.0 > idxTLStp_s) {
        k = 0;
        pEnd = 1;
      } else {
        k = static_cast<int>(idxTLStpPrv_s + 1.0) - 1;
        pEnd = static_cast<int>(idxTLStp_s) + 1;
      }
      if (b_i + 1 == NTLStp) {
        z_arr_size[0] = 1;
        nb = pEnd - k;
        z_arr_size[1] = nb + 1;
        z_arr_data[0] = tint_n_data[b_i];
        for (i2 = 0; i2 <= nb - 2; i2++) {
          z_arr_data[i2 + 1] = x_data[k + i2];
        }
        z_arr_data[nb] = tf0s;
      } else {
        z_arr_size[0] = 1;
        nb = pEnd - k;
        z_arr_size[1] = nb;
        z_arr_data[0] = tint_n_data[b_i];
        for (i2 = 0; i2 <= nb - 2; i2++) {
          z_arr_data[i2 + 1] = x_data[k + i2];
        }
      }
      for (i2 = 0; i2 < qEnd; i2++) {
        tmp_data[i2] = std::isnan(ti_arr_data[i2]);
      }
      coder::c_eml_find(tmp_data, tmp_size, idx_data, l_arr_s_size);
      khi = idx_data[0];
      idxTLStp_s = static_cast<double>(idx_data[0]) +
                   (static_cast<double>(z_arr_size[1]) - 1.0);
      if (idxTLStp_s < idx_data[0]) {
        n = 0;
      } else {
        nb = static_cast<int>(idxTLStp_s - static_cast<double>(idx_data[0]));
        n = nb + 1;
        for (i2 = 0; i2 <= nb; i2++) {
          idxVal_arr_data[i2] =
              static_cast<double>(khi) + static_cast<double>(i2);
        }
      }
      nb = z_arr_size[1] - 1;
      for (i2 = 0; i2 <= nb; i2++) {
        ti_arr_data[static_cast<int>(idxVal_arr_data[i2]) - 1] = z_arr_data[i2];
      }
      if (z_arr_size[1] == 2) {
        //  (idxTLStp_s - idxTLStpPrv_s) <= 1 % no TL between the two selected
        //  TLs
        if (b_i + 1 == 1) {
          //  the first selected TL
          khi = 2;
          vi_arr_s_data[0] = v0s;
          vi_arr_s_data[1] = 0.0;
        } else if (b_i + 1 == NTLStp) {
          double b_si_arr_data[38];
          b_si_arr_size[0] = 1;
          b_si_arr_size[1] = n;
          for (k = 0; k < n; k++) {
            b_si_arr_data[k] =
                si_arr_data[static_cast<int>(idxVal_arr_data[k]) - 1];
          }
          coder::diff(b_si_arr_data, b_si_arr_size, l_arr_s_data, l_arr_s_size);
          b_tmp_size[0] = 1;
          nb = l_arr_s_size[1];
          b_tmp_size[1] = l_arr_s_size[1];
          khi = (l_arr_s_size[1] / 2) << 1;
          n = khi - 2;
          for (k = 0; k <= n; k += 2) {
            __m128d r;
            r = _mm_loadu_pd(&l_arr_s_data[k]);
            _mm_storeu_pd(&b_tmp_data[k], _mm_mul_pd(_mm_set1_pd(3.0), r));
          }
          for (k = khi; k < nb; k++) {
            b_tmp_data[k] = 3.0 * l_arr_s_data[k];
          }
          khi = 2;
          vi_arr_s_data[0] = 0.0;
          coder::diff(z_arr_data, z_arr_size, l_arr_s_data, l_arr_s_size);
          vi_arr_s_data[1] =
              coder::internal::mrdiv(b_tmp_data, b_tmp_size, l_arr_s_data,
                                     l_arr_s_size) /
              2.0;
        } else {
          khi = 2;
          vi_arr_s_data[0] = 0.0;
          vi_arr_s_data[1] = 0.0;
        }
      } else {
        double b_si_arr_data[38];
        //  at least one TL between the two selected TLs
        b_si_arr_size[0] = 1;
        b_si_arr_size[1] = n;
        for (i2 = 0; i2 < n; i2++) {
          b_si_arr_data[i2] =
              si_arr_data[static_cast<int>(idxVal_arr_data[i2]) - 1];
        }
        coder::diff(b_si_arr_data, b_si_arr_size, l_arr_s_data, l_arr_s_size);
        if (b_i + 1 == NTLStp) {
          int A_size[2];
          //  the last segment
          idxTLStp_s = vfs;
          if (!std::isnan(tf0s)) {
            nb = pEnd - k;
            khi = nb + 1;
            b_ti_arr_data[0] = z_arr_data[0];
            for (pEnd = 0; pEnd <= nb - 2; pEnd++) {
              b_ti_arr_data[pEnd + 1] = x_data[k + pEnd];
            }
            b_ti_arr_data[nb] = tf0s;
          } else {
            khi = pEnd - k;
            b_ti_arr_data[0] = z_arr_data[0];
            for (pEnd = 0; pEnd <= khi - 2; pEnd++) {
              b_ti_arr_data[pEnd + 1] = x_data[k + pEnd];
            }
          }
          if (khi < 2) {
            k = 0;
            pEnd = 0;
          } else {
            k = 1;
            pEnd = khi;
          }
          if (khi - 1 < 1) {
            i2 = 0;
          } else {
            i2 = khi - 1;
          }
          nb = pEnd - k;
          if (nb == i2) {
            z_arr_size[0] = 1;
            z_arr_size[1] = nb;
            khi = (nb / 2) << 1;
            n = khi - 2;
            for (pEnd = 0; pEnd <= n; pEnd += 2) {
              __m128d r;
              __m128d r1;
              r = _mm_loadu_pd(&b_ti_arr_data[k + pEnd]);
              r1 = _mm_loadu_pd(&b_ti_arr_data[pEnd]);
              _mm_storeu_pd(&z_arr_data[pEnd], _mm_sub_pd(r, r1));
            }
            for (pEnd = khi; pEnd < nb; pEnd++) {
              z_arr_data[pEnd] = b_ti_arr_data[k + pEnd] - b_ti_arr_data[pEnd];
            }
          } else {
            binary_expand_op_9(z_arr_data, z_arr_size, b_ti_arr_data, k,
                               pEnd - 1, i2 - 1);
          }
          n = matrix_calc_f(z_arr_data, z_arr_size, l_arr_s_data, l_arr_s_size,
                            vfs, 0.0, A_data, A_size, vopt_data);
          coder::mldivide(A_data, A_size, vopt_data, n);
          //  vopt = inv(A)*Y;
          if (std::isnan(vfs)) {
            idxTLStp_s = (3.0 * l_arr_s_data[l_arr_s_size[1] - 1] /
                              z_arr_data[z_arr_size[1] - 1] -
                          vopt_data[n - 1]) /
                         2.0;
          }
          khi = n + 2;
          vi_arr_s_data[0] = 0.0;
          if (n - 1 >= 0) {
            std::copy(&vopt_data[0], &vopt_data[n], &vi_arr_s_data[1]);
          }
          vi_arr_s_data[n + 1] = idxTLStp_s;
        } else {
          if (b_i + 1 == 1) {
            idxTLStpPrv_s = v0s;
          } else {
            idxTLStpPrv_s = 0.0;
          }
          nb = pEnd - k;
          vopt_data[0] = z_arr_data[0];
          for (pEnd = 0; pEnd <= nb - 2; pEnd++) {
            vopt_data[pEnd + 1] = x_data[k + pEnd];
          }
          if (nb < 2) {
            k = 0;
            pEnd = 0;
          } else {
            k = 1;
            pEnd = nb;
          }
          if (nb - 1 < 1) {
            i2 = 0;
          } else {
            i2 = nb - 1;
          }
          nb = pEnd - k;
          if (nb == i2) {
            b_z_arr_size[0] = 1;
            b_z_arr_size[1] = nb;
            khi = (nb / 2) << 1;
            n = khi - 2;
            for (pEnd = 0; pEnd <= n; pEnd += 2) {
              __m128d r;
              __m128d r1;
              r = _mm_loadu_pd(&vopt_data[k + pEnd]);
              r1 = _mm_loadu_pd(&vopt_data[pEnd]);
              _mm_storeu_pd(&b_z_arr_data[pEnd], _mm_sub_pd(r, r1));
            }
            for (pEnd = khi; pEnd < nb; pEnd++) {
              b_z_arr_data[pEnd] = vopt_data[k + pEnd] - vopt_data[pEnd];
            }
          } else {
            binary_expand_op_9(b_z_arr_data, b_z_arr_size, vopt_data, k,
                               pEnd - 1, i2 - 1);
          }
          //  z_arr - 1x(N+1): travel time on each road section
          //  l_arr - 1x(N+1): length of each road section
          //  vfs - 1: final speed, nan (free condition) or value (fixed
          //  condition) v0s - 1: initial speed, value
          k = b_z_arr_size[1];
          khi = b_z_arr_size[1] - 1;
          b_A_size[0] = b_z_arr_size[1] - 1;
          b_A_size[1] = b_z_arr_size[1] - 1;
          for (nb = 0; nb <= k - 2; nb++) {
            double d;
            //  row
            idxTLStp_s = b_z_arr_data[nb];
            d = b_z_arr_data[nb + 1];
            Y_data[nb] = 6.0 * (l_arr_s_data[nb] / (idxTLStp_s * idxTLStp_s) +
                                l_arr_s_data[nb + 1] / (d * d));
            for (n = 0; n <= k - 2; n++) {
              //  column
              if (n == nb) {
                b_A_data[nb + (k - 1) * n] = 4.0 * (1.0 / idxTLStp_s + 1.0 / d);
              } else if (n + 1 == nb + 2) {
                b_A_data[nb + (k - 1) * n] = 2.0 / d;
              } else if (n + 1 == nb) {
                b_A_data[nb + (k - 1) * n] = 2.0 / idxTLStp_s;
              } else {
                b_A_data[nb + (k - 1) * n] = 0.0;
              }
            }
          }
          Y_data[0] -= 2.0 * idxTLStpPrv_s / b_z_arr_data[0];
          //  stop sign
          Y_data[b_z_arr_size[1] - 2] -=
              0.0 / b_z_arr_data[b_z_arr_size[1] - 1];
          //  vopt = inv(A)*Y;
          n = b_z_arr_size[1] - 1;
          if (khi - 1 >= 0) {
            std::copy(&Y_data[0], &Y_data[khi], &vopt_data[0]);
          }
          coder::mldivide(b_A_data, b_A_size, vopt_data, n);
          khi = n + 2;
          vi_arr_s_data[0] = idxTLStpPrv_s;
          if (n - 1 >= 0) {
            std::copy(&vopt_data[0], &vopt_data[n], &vi_arr_s_data[1]);
          }
          vi_arr_s_data[n + 1] = 0.0;
        }
      }
      nb = khi - 1;
      for (k = 0; k <= nb; k++) {
        vi_arr_data[static_cast<int>(idxVal_arr_data[k]) - 1] =
            vi_arr_s_data[k];
      }
    }
  } else {
    int A_size[2];
    int l_arr_s_size[2];
    coder::diff(si_arr_data, si_arr_size, l_arr_s_data, l_arr_s_size);
    idxTLStp_s = vfs;
    if (!std::isnan(tf0s)) {
      khi = x_size[1] + 2;
      b_ti_arr_data[0] = t0s;
      nb = x_size[1];
      if (nb - 1 >= 0) {
        std::copy(&x_data[0], &x_data[nb], &b_ti_arr_data[1]);
      }
      b_ti_arr_data[x_size[1] + 1] = tf0s;
    } else {
      khi = x_size[1] + 1;
      b_ti_arr_data[0] = t0s;
      nb = x_size[1];
      if (nb - 1 >= 0) {
        std::copy(&x_data[0], &x_data[nb], &b_ti_arr_data[1]);
      }
    }
    if (khi < 2) {
      i = 0;
      k = 0;
    } else {
      i = 1;
      k = khi;
    }
    nb = k - i;
    if (nb == khi - 1) {
      z_arr_size[0] = 1;
      z_arr_size[1] = nb;
      khi = (nb / 2) << 1;
      n = khi - 2;
      for (k = 0; k <= n; k += 2) {
        __m128d r;
        __m128d r1;
        r = _mm_loadu_pd(&b_ti_arr_data[i + k]);
        r1 = _mm_loadu_pd(&b_ti_arr_data[k]);
        _mm_storeu_pd(&z_arr_data[k], _mm_sub_pd(r, r1));
      }
      for (k = khi; k < nb; k++) {
        z_arr_data[k] = b_ti_arr_data[i + k] - b_ti_arr_data[k];
      }
    } else {
      binary_expand_op_9(z_arr_data, z_arr_size, b_ti_arr_data, i, k - 1,
                         khi - 2);
    }
    n = matrix_calc_f(z_arr_data, z_arr_size, l_arr_s_data, l_arr_s_size, vfs,
                      v0s, A_data, A_size, vopt_data);
    coder::mldivide(A_data, A_size, vopt_data, n);
    //  vopt = inv(A)*Y;
    if (std::isnan(vfs)) {
      idxTLStp_s = (3.0 * l_arr_s_data[l_arr_s_size[1] - 1] /
                        z_arr_data[z_arr_size[1] - 1] -
                    vopt_data[n - 1]) /
                   2.0;
    }
    ti_arr_size[0] = 1;
    ti_arr_size[1] = x_size[1] + 2;
    ti_arr_data[0] = t0s;
    nb = x_size[1];
    if (nb - 1 >= 0) {
      std::copy(&x_data[0], &x_data[nb], &ti_arr_data[1]);
    }
    ti_arr_data[x_size[1] + 1] = tf0s;
    vi_arr_size[0] = 1;
    vi_arr_size[1] = n + 2;
    vi_arr_data[0] = v0s;
    if (n - 1 >= 0) {
      std::copy(&vopt_data[0], &vopt_data[n], &vi_arr_data[1]);
    }
    vi_arr_data[n + 1] = idxTLStp_s;
  }
}

//
// z_arr - 1x(N+1): travel time on each road section
//  l_arr - 1x(N+1): length of each road section
//  vfs - 1: final speed, nan (free condition) or value (fixed condition)
//  v0s - 1: initial speed, value
//
// Arguments    : const double z_arr_data[]
//                const int z_arr_size[2]
//                const double l_arr_data[]
//                const int l_arr_size[2]
//                double vfs
//                double v0s
//                double A_data[]
//                int A_size[2]
//                double Y_data[]
// Return Type  : int
//
static int matrix_calc_f(const double z_arr_data[], const int z_arr_size[2],
                         const double l_arr_data[], const int l_arr_size[2],
                         double vfs, double v0s, double A_data[], int A_size[2],
                         double Y_data[])
{
  double k0;
  double k0_tmp;
  double k1;
  int A_data_tmp;
  int Y_size;
  A_size[0] = z_arr_size[1] - 1;
  A_size[1] = z_arr_size[1] - 1;
  Y_size = z_arr_size[1] - 1;
  A_data_tmp = z_arr_size[1];
  for (int idx0{0}; idx0 <= A_data_tmp - 2; idx0++) {
    //  row
    k0_tmp = z_arr_data[idx0];
    k1 = z_arr_data[idx0 + 1];
    Y_data[idx0] = 6.0 * (l_arr_data[idx0] / (k0_tmp * k0_tmp) +
                          l_arr_data[idx0 + 1] / (k1 * k1));
    for (int idx1{0}; idx1 <= A_data_tmp - 2; idx1++) {
      //  column
      if (idx1 == idx0) {
        A_data[idx0 + A_size[0] * idx1] = 4.0 * (1.0 / k0_tmp + 1.0 / k1);
      } else if (idx1 + 1 == idx0 + 2) {
        A_data[idx0 + A_size[0] * idx1] = 2.0 / k1;
      } else if (idx1 + 1 == idx0) {
        A_data[idx0 + A_size[0] * idx1] = 2.0 / k0_tmp;
      } else {
        A_data[idx0 + A_size[0] * idx1] = 0.0;
      }
    }
  }
  Y_data[0] -= 2.0 * v0s / z_arr_data[0];
  if (!std::isnan(vfs)) {
    //  stop sign
    k0 = 2.0 * vfs / z_arr_data[z_arr_size[1] - 1];
    k1 = 0.0;
  } else {
    k0_tmp = z_arr_data[z_arr_size[1] - 1];
    k0 = 3.0 * l_arr_data[l_arr_size[1] - 1] / (k0_tmp * k0_tmp);
    k1 = 1.0 / k0_tmp;
  }
  A_data_tmp = (z_arr_size[1] + (z_arr_size[1] - 1) * (z_arr_size[1] - 2)) - 2;
  A_data[A_data_tmp] -= k1;
  Y_data[z_arr_size[1] - 2] -= k0;
  return Y_size;
}

//
// calculate feasible min and max in green windows in forward
//  and backward way
//
// Arguments    : const double tmin0_arr_data[]
//                const int tmin0_arr_size[2]
//                const double tmax0_arr_data[]
//                const int tmax0_arr_size[2]
//                const double dtreq_arr_data[]
//                const int dtreq_arr_size[2]
//                double t0s
//                double tf0s
//                double out_tmin0n_arr_data[]
//                int out_tmin0n_arr_size[2]
//                double out_tmax0n_arr_data[]
//                int out_tmax0n_arr_size[2]
//                double out_tmin0f_arr_data[]
//                int out_tmin0f_arr_size[2]
//                double out_tmax0f_arr_data[]
//                int out_tmax0f_arr_size[2]
//                double out_dtreq_arr_data[]
//                int out_dtreq_arr_size[2]
//                double &out_tfs
// Return Type  : double
//
static double
tLimCal_fcn(const double tmin0_arr_data[], const int tmin0_arr_size[2],
            const double tmax0_arr_data[], const int tmax0_arr_size[2],
            const double dtreq_arr_data[], const int dtreq_arr_size[2],
            double t0s, double tf0s, double out_tmin0n_arr_data[],
            int out_tmin0n_arr_size[2], double out_tmax0n_arr_data[],
            int out_tmax0n_arr_size[2], double out_tmin0f_arr_data[],
            int out_tmin0f_arr_size[2], double out_tmax0f_arr_data[],
            int out_tmax0f_arr_size[2], double out_dtreq_arr_data[],
            int out_dtreq_arr_size[2], double &out_tfs)
{
  double maxval_data[12];
  double tmin0n_s;
  int maxval_size[2];
  int k;
  int loop_ub;
  int vlen_tmp;
  tmin0n_s = t0s;
  out_tmin0n_arr_size[0] = 1;
  loop_ub = tmin0_arr_size[1];
  out_tmin0n_arr_size[1] = tmin0_arr_size[1];
  for (int idx0{0}; idx0 < loop_ub; idx0++) {
    tmin0n_s = std::fmax(tmin0_arr_data[idx0], tmin0n_s + dtreq_arr_data[idx0]);
    out_tmin0n_arr_data[idx0] = tmin0n_s;
  }
  //  sum of all required times from current time t0s
  vlen_tmp = dtreq_arr_size[1];
  if (dtreq_arr_size[1] == 0) {
    tmin0n_s = 0.0;
  } else {
    tmin0n_s = dtreq_arr_data[0];
    for (k = 2; k <= vlen_tmp; k++) {
      tmin0n_s += dtreq_arr_data[k - 1];
    }
  }
  out_tfs =
      std::fmax((t0s + tmin0n_s) + 0.1,
                std::fmax(tf0s, (out_tmin0n_arr_data[tmin0_arr_size[1] - 1] +
                                 dtreq_arr_data[dtreq_arr_size[1] - 1]) +
                                    0.1));
  //      tfs = max(tf0s, tf0req);
  tmin0n_s = out_tfs;
  out_tmax0n_arr_size[0] = 1;
  out_tmax0n_arr_size[1] = tmin0_arr_size[1];
  for (int idx0{0}; idx0 < loop_ub; idx0++) {
    k = tmin0_arr_size[1] - idx0;
    tmin0n_s = std::fmin(tmax0_arr_data[k - 1], tmin0n_s - dtreq_arr_data[k]);
    out_tmax0n_arr_data[k - 1] = tmin0n_s;
  }
  //  set the new min and max in green windows
  maxval_size[1] = tmin0_arr_size[1];
  for (k = 0; k < loop_ub; k++) {
    maxval_data[k] = std::fmax(tmin0_arr_data[k], out_tmin0n_arr_data[k]);
  }
  out_tmin0f_arr_size[0] = 1;
  loop_ub = maxval_size[1];
  out_tmin0f_arr_size[1] = maxval_size[1];
  if (loop_ub - 1 >= 0) {
    std::copy(&maxval_data[0], &maxval_data[loop_ub], &out_tmin0f_arr_data[0]);
  }
  if (tmax0_arr_size[1] == tmin0_arr_size[1]) {
    loop_ub = tmax0_arr_size[1];
    maxval_size[1] = tmax0_arr_size[1];
    for (k = 0; k < loop_ub; k++) {
      maxval_data[k] = std::fmin(tmax0_arr_data[k], out_tmax0n_arr_data[k]);
    }
  } else {
    coder::internal::expand_min(tmax0_arr_data, tmax0_arr_size,
                                out_tmax0n_arr_data, out_tmax0n_arr_size,
                                maxval_data, maxval_size);
  }
  out_tmax0f_arr_size[0] = 1;
  loop_ub = maxval_size[1];
  out_tmax0f_arr_size[1] = maxval_size[1];
  if (loop_ub - 1 >= 0) {
    std::copy(&maxval_data[0], &maxval_data[loop_ub], &out_tmax0f_arr_data[0]);
  }
  out_dtreq_arr_size[0] = 1;
  out_dtreq_arr_size[1] = dtreq_arr_size[1];
  if (vlen_tmp - 1 >= 0) {
    std::copy(&dtreq_arr_data[0], &dtreq_arr_data[vlen_tmp],
              &out_dtreq_arr_data[0]);
  }
  return tf0s;
}

//
// Arguments    : const double si_arr_data[]
//                const int si_arr_size[2]
//                double t0s
//                double tf0s
//                double vfs
//                double v0s
//                const double ViolInfo_idxTLStp_data[]
//                const int ViolInfo_idxTLStp_size[2]
//                const double ViolInfo_tint_data[]
//                const int ViolInfo_tint_size[2]
//                const double x_data[]
//                const int x_size[2]
// Return Type  : double
//
double OptEntTimeSearch_v2_anonFcn1(
    const double si_arr_data[], const int si_arr_size[2], double t0s,
    double tf0s, double vfs, double v0s, const double ViolInfo_idxTLStp_data[],
    const int ViolInfo_idxTLStp_size[2], const double ViolInfo_tint_data[],
    const int ViolInfo_tint_size[2], const double x_data[], const int x_size[2])
{
  __m128d r;
  double y_data[1000];
  double tmp_data[37];
  double ti_arr_data[25];
  double vi_arr_data[25];
  double z_arr_data[25];
  double b_tmp_data[24];
  double d;
  double d1;
  double d2;
  double varargin_1;
  int c_tmp_data[25];
  int b_tmp_size[2];
  int b_vi_arr_size[2];
  int ti_arr_size[2];
  int tmp_size[2];
  int vi_arr_size[2];
  int y_size[2];
  int z_arr_size[2];
  int Efpnlt;
  int b_loop_ub;
  int i;
  int i1;
  int i10;
  int i11;
  int i12;
  int i2;
  int i3;
  int i4;
  int i5;
  int i6;
  int i7;
  int i8;
  int i9;
  int loop_ub;
  int scalarLB;
  int vectorUB;
  coder::diff(si_arr_data, si_arr_size, tmp_data, tmp_size);
  jncstate_cal(x_data, x_size, si_arr_data, si_arr_size, t0s, tf0s, vfs, v0s,
               ViolInfo_idxTLStp_data, ViolInfo_idxTLStp_size,
               ViolInfo_tint_data, ViolInfo_tint_size, ti_arr_data, ti_arr_size,
               vi_arr_data, vi_arr_size);
  if (ti_arr_size[1] < 2) {
    i = 0;
    i1 = 0;
  } else {
    i = 1;
    i1 = ti_arr_size[1];
  }
  if (ti_arr_size[1] - 1 < 1) {
    i2 = 0;
  } else {
    i2 = ti_arr_size[1] - 1;
  }
  loop_ub = i1 - i;
  if (loop_ub == i2) {
    z_arr_size[0] = 1;
    z_arr_size[1] = loop_ub;
    scalarLB = (loop_ub / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i1 = 0; i1 <= vectorUB; i1 += 2) {
      __m128d r1;
      r = _mm_loadu_pd(&ti_arr_data[i + i1]);
      r1 = _mm_loadu_pd(&ti_arr_data[i1]);
      _mm_storeu_pd(&z_arr_data[i1], _mm_sub_pd(r, r1));
    }
    for (i1 = scalarLB; i1 < loop_ub; i1++) {
      z_arr_data[i1] = ti_arr_data[i + i1] - ti_arr_data[i1];
    }
  } else {
    binary_expand_op_9(z_arr_data, z_arr_size, ti_arr_data, i, i1 - 1, i2 - 1);
  }
  if (vi_arr_size[1] - 1 < 1) {
    loop_ub = 0;
  } else {
    loop_ub = vi_arr_size[1] - 1;
  }
  if (vi_arr_size[1] < 2) {
    i = -1;
    i1 = -1;
  } else {
    i = 0;
    i1 = vi_arr_size[1] - 1;
  }
  b_tmp_size[0] = 1;
  b_loop_ub = tmp_size[1];
  b_tmp_size[1] = tmp_size[1];
  scalarLB = (tmp_size[1] / 2) << 1;
  vectorUB = scalarLB - 2;
  for (i2 = 0; i2 <= vectorUB; i2 += 2) {
    r = _mm_loadu_pd(&tmp_data[i2]);
    _mm_storeu_pd(&b_tmp_data[i2], _mm_mul_pd(_mm_set1_pd(6.0), r));
  }
  for (i2 = scalarLB; i2 < b_loop_ub; i2++) {
    b_tmp_data[i2] = 6.0 * tmp_data[i2];
  }
  y_size[0] = 1;
  b_loop_ub = z_arr_size[1];
  y_size[1] = z_arr_size[1];
  scalarLB = b_loop_ub / 2 * 2;
  vectorUB = scalarLB - 2;
  for (i2 = 0; i2 <= vectorUB; i2 += 2) {
    r = _mm_loadu_pd(&z_arr_data[i2]);
    r = _mm_mul_pd(r, r);
    _mm_storeu_pd(&y_data[i2], r);
  }
  for (i2 = scalarLB; i2 < b_loop_ub; i2++) {
    varargin_1 = z_arr_data[i2];
    y_data[i2] = varargin_1 * varargin_1;
  }
  //  calculate acceleration at the boundaries
  //  to avoid negative speed
  Efpnlt = 0;
  i2 = i1 - i;
  if (loop_ub == 1) {
    scalarLB = i2;
  } else {
    scalarLB = loop_ub;
  }
  if (tmp_size[1] == 1) {
    i3 = z_arr_size[1];
  } else {
    i3 = tmp_size[1];
  }
  if (scalarLB == 1) {
    vectorUB = z_arr_size[1];
  } else {
    vectorUB = scalarLB;
  }
  if (i3 == 1) {
    b_loop_ub = vectorUB;
  } else {
    b_loop_ub = i3;
  }
  if ((loop_ub == i2) && (tmp_size[1] == z_arr_size[1]) && (loop_ub == i2) &&
      (scalarLB == z_arr_size[1]) && (i3 == vectorUB) &&
      (scalarLB == b_loop_ub) && (loop_ub == i2) &&
      (tmp_size[1] == z_arr_size[1]) && (loop_ub == i2) &&
      (scalarLB == z_arr_size[1]) && (i3 == vectorUB) &&
      (scalarLB == b_loop_ub)) {
    boolean_T b_vi_arr_data[25];
    b_vi_arr_size[0] = 1;
    b_vi_arr_size[1] = loop_ub;
    for (i3 = 0; i3 < loop_ub; i3++) {
      d = tmp_data[i3];
      d1 = y_data[i3];
      d2 = z_arr_data[i3];
      varargin_1 = vi_arr_data[(i + i3) + 1];
      b_vi_arr_data[i3] =
          (((vi_arr_data[i3] > 0.0) && (varargin_1 == 0.0) &&
            (-6.0 * d / d1 + 2.0 * (vi_arr_data[i3] + 2.0 * varargin_1) / d2 >
             0.0)) ||
           ((vi_arr_data[i3] == 0.0) && (varargin_1 > 0.0) &&
            (6.0 * d / d1 - 2.0 * (2.0 * vi_arr_data[i3] + varargin_1) / d2 <
             0.0)));
    }
    coder::c_eml_find(b_vi_arr_data, b_vi_arr_size, c_tmp_data, vi_arr_size);
  } else {
    binary_expand_op_8(c_tmp_data, vi_arr_data, loop_ub - 1, i + 1, i1,
                       tmp_data, tmp_size, y_data, y_size, z_arr_data,
                       z_arr_size, vi_arr_size);
  }
  if (static_cast<signed char>(vi_arr_size[1]) != 0) {
    Efpnlt = 100000 * static_cast<signed char>(vi_arr_size[1]);
  }
  if (tmp_size[1] == 1) {
    i3 = loop_ub;
    vectorUB = z_arr_size[1];
  } else {
    i3 = tmp_size[1];
    vectorUB = tmp_size[1];
  }
  if (i3 == 1) {
    b_loop_ub = z_arr_size[1];
  } else {
    b_loop_ub = i3;
  }
  if (tmp_size[1] == 1) {
    i4 = i2;
  } else {
    i4 = tmp_size[1];
  }
  if (vectorUB == 1) {
    i5 = b_loop_ub;
  } else {
    i5 = vectorUB;
  }
  if (i4 == 1) {
    i6 = z_arr_size[1];
  } else {
    i6 = i4;
  }
  if (i5 == 1) {
    i7 = i6;
  } else {
    i7 = i5;
  }
  if (loop_ub == 1) {
    i8 = z_arr_size[1];
  } else {
    i8 = loop_ub;
  }
  if (i7 == 1) {
    i9 = i8;
  } else {
    i9 = i7;
  }
  if (scalarLB == 1) {
    i10 = z_arr_size[1];
  } else {
    i10 = scalarLB;
  }
  if (i9 == 1) {
    i11 = i10;
  } else {
    i11 = i9;
  }
  if (i2 == 1) {
    i12 = z_arr_size[1];
  } else {
    i12 = i2;
  }
  if ((tmp_size[1] == z_arr_size[1]) && (tmp_size[1] == loop_ub) &&
      (i3 == z_arr_size[1]) && (vectorUB == b_loop_ub) && (tmp_size[1] == i2) &&
      (i4 == z_arr_size[1]) && (i5 == i6) && (loop_ub == z_arr_size[1]) &&
      (i7 == i8) && (loop_ub == i2) && (scalarLB == z_arr_size[1]) &&
      (i9 == i10) && (i2 == z_arr_size[1]) && (i11 == i12)) {
    loop_ub = tmp_size[1];
    ti_arr_size[1] = tmp_size[1];
    for (i1 = 0; i1 < loop_ub; i1++) {
      double b_varargin_1;
      double ti_arr_tmp;
      d = z_arr_data[i1];
      d1 = b_tmp_data[i1];
      d2 = y_data[i1];
      ti_arr_tmp = vi_arr_data[(i + i1) + 1];
      varargin_1 = tmp_data[i1];
      b_varargin_1 = vi_arr_data[i1];
      ti_arr_data[i1] =
          ((((6.0 * (varargin_1 * varargin_1) / rt_powd_snf(d, 3.0) -
              d1 * vi_arr_data[i1] / d2) -
             d1 * ti_arr_tmp / d2) +
            2.0 * (b_varargin_1 * b_varargin_1) / d) +
           2.0 * vi_arr_data[i1] * ti_arr_tmp / d) +
          2.0 * (ti_arr_tmp * ti_arr_tmp) / d;
    }
  } else {
    binary_expand_op_7(ti_arr_data, ti_arr_size, tmp_data, tmp_size, z_arr_data,
                       z_arr_size, b_tmp_data, b_tmp_size, vi_arr_data,
                       loop_ub - 1, y_data, y_size, i + 1, i1);
  }
  scalarLB = ti_arr_size[1];
  if (ti_arr_size[1] == 0) {
    varargin_1 = 0.0;
  } else {
    varargin_1 = ti_arr_data[0];
    for (vectorUB = 2; vectorUB <= scalarLB; vectorUB++) {
      varargin_1 += ti_arr_data[vectorUB - 1];
    }
  }
  return varargin_1 + static_cast<double>(Efpnlt);
}

//
// =================================================================== %
//      coder.extrinsic('tic');
//      coder.extrinsic('toc');
//
// Arguments    : const double IntscDesEntrInfo[48]
//                const double IntscGrnSlctdInfo[50]
//                const double CtrlInfo[14]
//                const double CtrlPar[15]
//                const double StopInfo[2]
//                double TRAJ[4000]
//                double Xis[60]
//                double TimeCnstr[80]
//                double OptSlvPrf[2]
// Return Type  : double
//
double RefTrjGnrtr_240503(const double IntscDesEntrInfo[48],
                          const double IntscGrnSlctdInfo[50],
                          const double CtrlInfo[14], const double CtrlPar[15],
                          const double StopInfo[2], double TRAJ[4000],
                          double Xis[60], double TimeCnstr[80],
                          double OptSlvPrf[2])
{
  double TRAJ_raw[4000];
  double b_TRAJ_raw[1000];
  double maxval_data[1000];
  double d_tmp_data[37];
  double Nti_arr_data[25];
  double idx_nan_data[25];
  double si_arr_data[25];
  double ti_arr_data[25];
  double vi_arr_data[25];
  double b_maxval_data[24];
  double IntscType_all_data[13];
  double pos_intsc_all_data[13];
  double t_arrv_des_all_data[13];
  double ViolInfo_tint_data[12];
  double b_l_arr_data[12];
  double b_tmax0_set0_data[12];
  double tmax0_set0_data[12];
  double tmax0_set_data[12];
  double tmax0_setn_data[12];
  double tmin0_set_data[12];
  double tmin0_setn_data[12];
  double b_expl_temp_data[10];
  double dtMin_arr_data[10];
  double expl_temp_data[10];
  double tLimOut_tmax0f_arr_data[10];
  double tLimOut_tmin0f_arr_data[10];
  double TrajType;
  double d;
  double sfs;
  double t0s;
  double tLimOut_tfs;
  double tfs;
  double tfs_tmp;
  double v0;
  double vfs;
  int indx_data[1000];
  int e_tmp_data[25];
  int tmp_data[12];
  int c_tmp_data[10];
  int IntscGrnSlctdInfo_size[2];
  int Nti_arr_size[2];
  int ViolInfo_tint_size[2];
  int b_l_arr_size[2];
  int b_maxval_size[2];
  int b_tmax0_set0_size[2];
  int b_tmp_size[2];
  int c_tmp_size[2];
  int dtMin_arr_size[2];
  int idx_nan_size[2];
  int l_arr_size[2];
  int maxval_size[2];
  int pos_intsc_all_size[2];
  int si_arr_size[2];
  int t_arrv_des_all_size[2];
  int ti_arr_size[2];
  int tmax0_set0_size[2];
  int tmax0_setn_size[2];
  int tmin0_setn_size[2];
  int tmp_size[2];
  int vi_arr_size[2];
  int b_i;
  int b_loop_ub;
  int c_loop_ub;
  int d_loop_ub;
  int i;
  int i1;
  int i2;
  int i3;
  int idx0Info;
  int idxTLStp_s;
  int loop_ub;
  int loop_ub_tmp;
  int size_tmp_idx_1_tmp;
  int trueCount;
  signed char f_tmp_data[13];
  boolean_T l_arr_data[12];
  boolean_T b_tmp_data[10];
  boolean_T guard1;
  //      time_Val=tic;
  //  =================================================================== %
  //  Input processing %
  for (i = 0; i < 12; i++) {
    l_arr_data[i] = !std::isnan(IntscDesEntrInfo[i << 2]);
  }
  coder::b_eml_find(l_arr_data, tmp_data, tmp_size);
  if (tmp_size[1] < 1) {
    i = -1;
  } else {
    for (i = 0; i < 12; i++) {
      l_arr_data[i] = !std::isnan(IntscDesEntrInfo[i << 2]);
    }
    coder::b_eml_find(l_arr_data, tmp_data, tmp_size);
    i = tmp_size[1] - 1;
  }
  for (i1 = 0; i1 < 10; i1++) {
    b_tmp_data[i1] = !std::isnan(IntscGrnSlctdInfo[5 * i1]);
  }
  coder::eml_find(b_tmp_data, c_tmp_data, tmp_size);
  if (tmp_size[1] < 1) {
    i1 = 0;
  } else {
    for (i1 = 0; i1 < 10; i1++) {
      b_tmp_data[i1] = !std::isnan(IntscGrnSlctdInfo[5 * i1]);
    }
    coder::eml_find(b_tmp_data, c_tmp_data, tmp_size);
    i1 = tmp_size[1];
  }
  //  last intersection type
  //  initial states and time update
  t0s = CtrlInfo[0];
  //  Ref states
  //  position
  //  speed
  //  acceleration
  //  Actual states
  //  position
  v0 = CtrlInfo[5];
  //  speed
  //  acceleration
  //  Intersection type (TL 10, Stop 20)
  //  current TL state (green 2, yellow 1 red 0)
  //  current maximum speed limit
  //  Stop mode (Stopped -1, Nearby 0, Moving 1)
  //  hold the stopped time
  d = IntscDesEntrInfo[(i << 2) + 3];
  if (d == 0.0) {
    //  virtual point at the end of the preview horizon
    vfs = rtNaN;
  } else {
    vfs = 0.0;
  }
  idx0Info = 1;
  if ((i1 != 0) && (IntscDesEntrInfo[0] - CtrlInfo[4] < 10.0) &&
      (IntscGrnSlctdInfo[3] == 0.0)) {
    //  at least one TL
    idx0Info = 2;
  }
  if (idx0Info > i + 1) {
    b_i = 0;
    trueCount = -1;
  } else {
    b_i = idx0Info - 1;
    trueCount = i;
  }
  pos_intsc_all_size[0] = 1;
  loop_ub = trueCount - b_i;
  b_loop_ub = loop_ub + 2;
  pos_intsc_all_size[1] = loop_ub + 2;
  pos_intsc_all_data[0] = CtrlInfo[4];
  for (trueCount = 0; trueCount <= loop_ub; trueCount++) {
    pos_intsc_all_data[trueCount + 1] =
        IntscDesEntrInfo[(b_i + trueCount) << 2];
  }
  if (idx0Info > i + 1) {
    b_i = 0;
    trueCount = -1;
  } else {
    b_i = idx0Info - 1;
    trueCount = i;
  }
  t_arrv_des_all_size[0] = 1;
  c_loop_ub = trueCount - b_i;
  t_arrv_des_all_size[1] = c_loop_ub + 2;
  t_arrv_des_all_data[0] = CtrlInfo[0];
  for (trueCount = 0; trueCount <= c_loop_ub; trueCount++) {
    t_arrv_des_all_data[trueCount + 1] =
        IntscDesEntrInfo[((b_i + trueCount) << 2) + 1];
  }
  if (idx0Info > i + 1) {
    b_i = 0;
    trueCount = 0;
    i = -1;
  } else {
    b_i = idx0Info - 1;
    trueCount = idx0Info - 1;
  }
  d_loop_ub = i - trueCount;
  IntscType_all_data[0] = -10.0;
  for (i = 0; i <= d_loop_ub; i++) {
    IntscType_all_data[i + 1] = IntscDesEntrInfo[((trueCount + i) << 2) + 3];
  }
  //  -10 current pos
  if (idx0Info > i1) {
    i = 1;
    trueCount = 0;
    i2 = 0;
    i3 = -1;
  } else {
    i = idx0Info;
    trueCount = i1;
    i2 = idx0Info - 1;
    i3 = i1 - 1;
  }
  //  =================================================================== %
  //  parameter setting %
  //  set the safety time margin to initial and end timing of green lights
  //  set the desired speed
  //  confine the contraint based on desired arrival times
  //  delta time before the desired arrival time for minimum time limit
  //  delta time after the desired arrival time for maximum time limit
  //  desired acceleration
  //  1 Acceleration, 0: No acceleration after the destination (the last stop
  //  sign) 1: MC Roll-Out, 2: Nonlinear Opt (fmincon) in case of algsel=1
  //  =================================================================== %
  //  output definition %
  //  TRAJ: Time, Accel, Speed, Position
  //  grid size
  for (idxTLStp_s = 0; idxTLStp_s < 4000; idxTLStp_s++) {
    TRAJ[idxTLStp_s] = rtNaN;
  }
  //  Xis: Optimal Time, Optimal Speed, Position
  for (idxTLStp_s = 0; idxTLStp_s < 60; idxTLStp_s++) {
    Xis[idxTLStp_s] = rtNaN;
  }
  //  Time Constraints
  //  TimeCnstr: intsc position, initial minimum time limit,
  //  initial maximum time limit, final minimum time limit,
  //  final maximum time limit
  if (idx0Info > i1) {
    idxTLStp_s = 0;
    size_tmp_idx_1_tmp = 0;
  } else {
    idxTLStp_s = idx0Info - 1;
    size_tmp_idx_1_tmp = i1;
  }
  size_tmp_idx_1_tmp -= idxTLStp_s;
  for (idxTLStp_s = 0; idxTLStp_s < 80; idxTLStp_s++) {
    TimeCnstr[idxTLStp_s] = rtNaN;
  }
  if (idx0Info > i1) {
    idx0Info = 1;
  }
  if (size_tmp_idx_1_tmp < 1) {
    loop_ub_tmp = 0;
  } else {
    loop_ub_tmp = static_cast<signed char>(size_tmp_idx_1_tmp);
  }
  for (i1 = 0; i1 < loop_ub_tmp; i1++) {
    TimeCnstr[i1 << 3] = IntscGrnSlctdInfo[5 * ((idx0Info + i1) - 1)];
  }
  //  =================================================================== %
  //  algorithm %
  //  size: 1 x N_intsc
  //  size: 1 x N_intsc
  TrajType = -1.0;
  OptSlvPrf[0] = rtNaN;
  OptSlvPrf[1] = rtNaN;
  //   traffic lights in the middle
  //  ini and final times are NOT in a green phase in the case of one traffic
  //  light with a virtual point in the preview horizon
  //      if t0s < tf0s - 0.5
  guard1 = false;
  if (size_tmp_idx_1_tmp != 0) {
    int end;
    boolean_T b;
    boolean_T b1;
    b = (static_cast<signed char>(size_tmp_idx_1_tmp) != 1);
    tfs = t_arrv_des_all_data[c_loop_ub + 1];
    end = d_loop_ub + 1;
    b1 = !(IntscType_all_data[d_loop_ub + 1] == 0.0);
    idx0Info = trueCount - i;
    b_tmp_size[0] = 1;
    b_tmp_size[1] = idx0Info + 1;
    for (i1 = 0; i1 <= idx0Info; i1++) {
      trueCount = 5 * ((i + i1) - 1);
      tfs_tmp = IntscGrnSlctdInfo[trueCount + 1];
      sfs = IntscGrnSlctdInfo[trueCount + 2];
      b_tmp_data[i1] =
          (b || (!(CtrlInfo[0] > tfs_tmp)) || (!(CtrlInfo[0] < sfs)) ||
           (!(tfs > tfs_tmp)) || (!(tfs < sfs)) || b1);
    }
    if (coder::internal::ifWhileCond(b_tmp_data, b_tmp_size)) {
      __m128d r;
      __m128d r1;
      double IntscGrnSlctdInfo_data[10];
      int tLimOut_tmax0f_arr_size[2];
      int tmax0_set_size[2];
      int e_loop_ub;
      boolean_T b_IntscGrnSlctdInfo_data[11];
      coder::diff(pos_intsc_all_data, pos_intsc_all_size, d_tmp_data, tmp_size);
      l_arr_size[0] = 1;
      e_loop_ub = tmp_size[1];
      l_arr_size[1] = tmp_size[1];
      if (e_loop_ub - 1 >= 0) {
        std::copy(&d_tmp_data[0], &d_tmp_data[e_loop_ub], &b_l_arr_data[0]);
      }
      //             %%
      dtMin_arr_size[0] = 1;
      dtMin_arr_size[1] = idx0Info + 1;
      IntscGrnSlctdInfo_size[0] = 1;
      IntscGrnSlctdInfo_size[1] = idx0Info + 1;
      for (i1 = 0; i1 <= idx0Info; i1++) {
        idxTLStp_s = 5 * ((i + i1) - 1);
        dtMin_arr_data[i1] = IntscGrnSlctdInfo[idxTLStp_s + 1];
        IntscGrnSlctdInfo_data[i1] = IntscGrnSlctdInfo[idxTLStp_s + 2];
      }
      tmax0_set0_size[0] = 1;
      tmax0_set0_size[1] = d_loop_ub + 1;
      for (i = 0; i <= d_loop_ub; i++) {
        tmax0_set0_data[i] = IntscDesEntrInfo[((b_i + i) << 2) + 2];
      }
      tLimCal_fcn(dtMin_arr_data, dtMin_arr_size, IntscGrnSlctdInfo_data,
                  IntscGrnSlctdInfo_size, tmax0_set0_data, tmax0_set0_size,
                  CtrlInfo[0], tfs, expl_temp_data, vi_arr_size,
                  b_expl_temp_data, ti_arr_size, tLimOut_tmin0f_arr_data,
                  b_tmp_size, tLimOut_tmax0f_arr_data, tLimOut_tmax0f_arr_size,
                  tmax0_set_data, tmax0_set_size, tLimOut_tfs);
      //             %%
      coder::diff(t_arrv_des_all_data, t_arrv_des_all_size, d_tmp_data,
                  tmp_size);
      //  dtMin should be reduced for very short link
      dtMin_arr_size[0] = 1;
      dtMin_arr_size[1] = static_cast<signed char>(b_tmp_size[1]);
      d_loop_ub = static_cast<signed char>(b_tmp_size[1]);
      for (i = 0; i < d_loop_ub; i++) {
        dtMin_arr_data[i] = CtrlPar[6];
      }
      dtMin_arr_data[0] =
          std::fmin(std::fmax(d_tmp_data[0] * 0.1, 0.01), CtrlPar[6]);
      if (c_loop_ub + 1 < 2) {
        i = 0;
        i1 = 0;
      } else {
        i = 1;
        i1 = c_loop_ub + 1;
      }
      c_loop_ub = i1 - i;
      if (c_loop_ub == static_cast<signed char>(b_tmp_size[1])) {
        tmax0_set0_size[0] = 1;
        tmax0_set0_size[1] = c_loop_ub;
        idxTLStp_s = (c_loop_ub / 2) << 1;
        idx0Info = idxTLStp_s - 2;
        for (i1 = 0; i1 <= idx0Info; i1 += 2) {
          r = _mm_loadu_pd(&t_arrv_des_all_data[i + i1]);
          r1 = _mm_loadu_pd(&dtMin_arr_data[i1]);
          _mm_storeu_pd(&tmax0_set0_data[i1], _mm_sub_pd(r, r1));
        }
        for (i1 = idxTLStp_s; i1 < c_loop_ub; i1++) {
          tmax0_set0_data[i1] =
              t_arrv_des_all_data[i + i1] - dtMin_arr_data[i1];
        }
        coder::internal::maximum2(tLimOut_tmin0f_arr_data, b_tmp_size,
                                  tmax0_set0_data, tmax0_set0_size,
                                  tmin0_set_data, pos_intsc_all_size);
      } else {
        binary_expand_op_5(tmin0_set_data, tLimOut_tmin0f_arr_data, b_tmp_size,
                           t_arrv_des_all_data, i, i1 - 1, dtMin_arr_data,
                           dtMin_arr_size, pos_intsc_all_size);
      }
      tmax0_set0_size[0] = 1;
      c_loop_ub = pos_intsc_all_size[1];
      tmax0_set0_size[1] = pos_intsc_all_size[1];
      d = 2.0 * CtrlPar[7];
      //  dtMax should be reduced for very short link
      //  need to check whether the minimum speed limit is violated for each
      //  link 1/3, 2/5, 1/2, 2/3
      tmin0_setn_size[0] = 1;
      tmin0_setn_size[1] = pos_intsc_all_size[1];
      idxTLStp_s = (pos_intsc_all_size[1] / 2) << 1;
      idx0Info = idxTLStp_s - 2;
      for (i = 0; i <= idx0Info; i += 2) {
        r = _mm_loadu_pd(&tmin0_set_data[i]);
        _mm_storeu_pd(&tmax0_set0_data[i], _mm_add_pd(r, _mm_set1_pd(d)));
        _mm_storeu_pd(&tmin0_setn_data[i], r);
      }
      for (i = idxTLStp_s; i < c_loop_ub; i++) {
        tfs_tmp = tmin0_set_data[i];
        tmax0_set0_data[i] = tfs_tmp + d;
        tmin0_setn_data[i] = tfs_tmp;
      }
      tmax0_set0_data[0] =
          tmin0_set_data[0] +
          2.0 *
              std::fmin(
                  CtrlPar[7],
                  b_l_arr_data[0] /
                      std::fmax(std::fmin(b_l_arr_data[0] /
                                              (tmin0_set_data[0] - CtrlInfo[0]),
                                          CtrlInfo[5]) *
                                    0.8,
                                5.0));
      coder::internal::minimum2(
          tLimOut_tmax0f_arr_data, tLimOut_tmax0f_arr_size, tmax0_set0_data,
          tmax0_set0_size, tmax0_set_data, tmax0_set_size);
      tmax0_setn_size[0] = 1;
      c_loop_ub = tmax0_set_size[1];
      tmax0_setn_size[1] = tmax0_set_size[1];
      if (c_loop_ub - 1 >= 0) {
        std::copy(&tmax0_set_data[0], &tmax0_set_data[c_loop_ub],
                  &tmax0_setn_data[0]);
      }
      c_loop_ub = i3 - i2;
      if (l_arr_size[1] == 1) {
        i = tmp_size[1];
      } else {
        i = l_arr_size[1];
      }
      if ((l_arr_size[1] == tmp_size[1]) && (i == c_loop_ub + 2)) {
        for (i = 0; i <= c_loop_ub; i++) {
          b_IntscGrnSlctdInfo_data[i] =
              (IntscGrnSlctdInfo[5 * (i2 + i) + 3] == 1.0);
        }
        b_IntscGrnSlctdInfo_data[c_loop_ub + 1] = false;
        b_l_arr_size[0] = 1;
        b_l_arr_size[1] = e_loop_ub;
        for (i = 0; i < e_loop_ub; i++) {
          l_arr_data[i] = ((b_l_arr_data[i] / d_tmp_data[i] < 4.0) &&
                           b_IntscGrnSlctdInfo_data[i]);
        }
        coder::c_eml_find(l_arr_data, b_l_arr_size, e_tmp_data, b_tmp_size);
      } else {
        binary_expand_op_3(e_tmp_data, b_l_arr_data, l_arr_size, d_tmp_data,
                           tmp_size, IntscGrnSlctdInfo, i2, i3, b_tmp_size);
      }
      Nti_arr_size[0] = 1;
      d_loop_ub = b_tmp_size[1];
      Nti_arr_size[1] = b_tmp_size[1];
      for (i = 0; i < d_loop_ub; i++) {
        Nti_arr_data[i] = e_tmp_data[i];
      }
      if (l_arr_size[1] == 1) {
        i = tmp_size[1];
      } else {
        i = l_arr_size[1];
      }
      if ((l_arr_size[1] == tmp_size[1]) && (i == c_loop_ub + 2)) {
        for (i = 0; i <= c_loop_ub; i++) {
          b_IntscGrnSlctdInfo_data[i] =
              (IntscGrnSlctdInfo[5 * (i2 + i) + 3] == 1.0);
        }
        b_IntscGrnSlctdInfo_data[c_loop_ub + 1] = false;
        b_l_arr_size[0] = 1;
        c_loop_ub = l_arr_size[1];
        b_l_arr_size[1] = l_arr_size[1];
        for (i = 0; i < c_loop_ub; i++) {
          l_arr_data[i] = ((b_l_arr_data[i] / d_tmp_data[i] < 4.0) &&
                           b_IntscGrnSlctdInfo_data[i]);
        }
        coder::c_eml_find(l_arr_data, b_l_arr_size, e_tmp_data, b_tmp_size);
      } else {
        binary_expand_op_3(e_tmp_data, b_l_arr_data, l_arr_size, d_tmp_data,
                           tmp_size, IntscGrnSlctdInfo, i2, i3, b_tmp_size);
      }
      c_loop_ub = b_tmp_size[1];
      tmp_size[1] = b_tmp_size[1];
      for (i = 0; i < c_loop_ub; i++) {
        d_tmp_data[i] = e_tmp_data[i];
      }
      //  in the array of tmin0_set
      //  idxTLStp = [];
      trueCount = 0;
      size_tmp_idx_1_tmp = 0;
      for (b_i = 0; b_i <= end; b_i++) {
        if (IntscType_all_data[b_i] == 10.0) {
          trueCount++;
          f_tmp_data[size_tmp_idx_1_tmp] = static_cast<signed char>(b_i);
          size_tmp_idx_1_tmp++;
        }
      }
      for (i = 0; i < trueCount; i++) {
        t_arrv_des_all_data[i] = pos_intsc_all_data[f_tmp_data[i]];
      }
      tmax0_set0_size[1] = b_tmp_size[1];
      for (i = 0; i < c_loop_ub; i++) {
        tmax0_set0_data[i] = rtNaN;
      }
      if (b_tmp_size[1] > 0) {
        for (i = 0; i < d_loop_ub; i++) {
          t_arrv_des_all_data[static_cast<int>(d_tmp_data[i]) - 1] = std::fmax(
              CtrlInfo[4] + 0.001,
              pos_intsc_all_data[f_tmp_data[static_cast<int>(Nti_arr_data[i]) -
                                            1]] -
                  0.5);
        }
        //  stop at 0.5m before the TL
        si_arr_size[0] = 1;
        b_loop_ub = (loop_ub + b_tmp_size[1]) + 2;
        si_arr_size[1] = b_loop_ub;
        for (i = 0; i < b_loop_ub; i++) {
          si_arr_data[i] = rtNaN;
        }
        tmax0_set0_size[1] = b_tmp_size[1];
        for (b_i = 0; b_i < c_loop_ub; b_i++) {
          i = static_cast<int>(d_tmp_data[b_i]);
          if (i == 1) {
            //  the first TL
            sfs = t_arrv_des_all_data[0] - CtrlInfo[4];
            if (sfs > 0.5) {
              //  average speed when acceleration is zero at the end
              //                              t_brk_des = (tmin0_set_org_s -
              //                              t0s)*(1-rRedWait); t_brk_des =
              //                              min(t_brk_des, t_brk_max);
              sfs = (t0s + sfs / std::fmax(4.0, v0 / 3.0 + 0.01)) + 1.1;
            } else {
              sfs = (t0s + 2.0 * sfs / (v0 + 0.01)) + 1.1;
              //  kinematic
            }
            i1 = static_cast<int>(Nti_arr_data[b_i]);
            sfs = std::fmin(tmin0_set_data[i1 - 1], sfs);
            //  can be adjusted
            tmax0_setn_data[0] = sfs;
            tmin0_setn_data[0] = std::fmin(t0s + 1.0, sfs);
            //  (t0s + tmin0_set(idxTLStp_s))/2;
          } else {
            i1 = static_cast<int>(Nti_arr_data[b_i]);
            sfs = tmax0_set_data[i1 - 2];
            tmax0_setn_data[i - 1] = sfs + (tmin0_set_data[i1 - 1] - sfs) * 0.5;
            tmin0_setn_data[i - 1] = sfs;
            //  can be adjusted
          }
          tmax0_set0_data[b_i] = tmin0_set_data[i1 - 1] + 0.1;
        }
        c_tmp_size[0] = 1;
        c_tmp_size[1] = b_loop_ub;
        idx_nan_size[0] = 1;
        for (b_i = 0; b_i <= d_loop_ub; b_i++) {
          double idxVal_arr_data[38];
          double si_arrn_s_data[14];
          boolean_T g_tmp_data[25];
          if (b_i == Nti_arr_size[1]) {
            idxTLStp_s = trueCount;
          } else {
            idxTLStp_s = static_cast<int>(Nti_arr_data[b_i]);
          }
          if (b_i + 1 == 1) {
            idx0Info = 0;
            sfs = pos_intsc_all_data[0];
          } else {
            idx0Info = static_cast<int>(Nti_arr_data[b_i - 1]);
            sfs =
                t_arrv_des_all_data[static_cast<int>(d_tmp_data[b_i - 1]) - 1];
          }
          if (static_cast<double>(idx0Info) + 1.0 > idxTLStp_s) {
            idx0Info = 0;
            idxTLStp_s = 0;
          }
          e_loop_ub = idxTLStp_s - idx0Info;
          si_arrn_s_data[0] = sfs;
          for (i = 0; i < e_loop_ub; i++) {
            si_arrn_s_data[i + 1] = t_arrv_des_all_data[idx0Info + i];
          }
          for (i = 0; i < b_loop_ub; i++) {
            g_tmp_data[i] = std::isnan(si_arr_data[i]);
          }
          coder::c_eml_find(g_tmp_data, c_tmp_size, e_tmp_data, b_tmp_size);
          size_tmp_idx_1_tmp = b_tmp_size[1];
          idx_nan_size[1] = b_tmp_size[1];
          for (i = 0; i < size_tmp_idx_1_tmp; i++) {
            idx_nan_data[i] = e_tmp_data[i];
          }
          sfs = idx_nan_data[0] + (static_cast<double>(e_loop_ub + 1) - 1.0);
          if (!(sfs < idx_nan_data[0])) {
            i = static_cast<int>(idx_nan_data[0]);
            size_tmp_idx_1_tmp = static_cast<int>(sfs - static_cast<double>(i));
            for (i1 = 0; i1 <= size_tmp_idx_1_tmp; i1++) {
              idxVal_arr_data[i1] =
                  static_cast<double>(i) + static_cast<double>(i1);
            }
          }
          for (i = 0; i <= e_loop_ub; i++) {
            si_arr_data[static_cast<int>(idxVal_arr_data[i]) - 1] =
                si_arrn_s_data[i];
          }
        }
        si_arr_data[b_loop_ub - 1] = pos_intsc_all_data[loop_ub + 1];
      } else {
        si_arr_size[0] = 1;
        si_arr_size[1] = loop_ub + 2;
        if (b_loop_ub - 1 >= 0) {
          std::copy(&pos_intsc_all_data[0], &pos_intsc_all_data[b_loop_ub],
                    &si_arr_data[0]);
        }
      }
      ViolInfo_tint_size[0] = 1;
      ViolInfo_tint_size[1] = c_loop_ub;
      if (c_loop_ub - 1 >= 0) {
        std::copy(&tmax0_set0_data[0], &tmax0_set0_data[c_loop_ub],
                  &ViolInfo_tint_data[0]);
      }
      b_tmax0_set0_size[0] = 1;
      b_tmax0_set0_size[1] = tmax0_set0_size[1];
      loop_ub = tmax0_set0_size[1] - 1;
      if (loop_ub >= 0) {
        std::copy(&tmax0_set0_data[0], &tmax0_set0_data[loop_ub + 1],
                  &b_tmax0_set0_data[0]);
      }
      OptSlvPrf[0] = OptEntTimeSearch_v2(
          CtrlInfo[0], tLimOut_tfs, vfs, CtrlInfo[5], si_arr_data, si_arr_size,
          tmin0_setn_data, tmin0_setn_size, tmax0_setn_data, tmax0_setn_size,
          Nti_arr_data, Nti_arr_size, b_tmax0_set0_data, b_tmax0_set0_size,
          tmax0_set0_data, tmax0_set0_size, b_l_arr_data, l_arr_size,
          OptSlvPrf[1]);
      sfs = (tLimOut_tfs - CtrlInfo[0]) / 999.0;
      jncstate_cal(tmax0_set0_data, tmax0_set0_size, si_arr_data, si_arr_size,
                   CtrlInfo[0], tLimOut_tfs, vfs, CtrlInfo[5], Nti_arr_data,
                   Nti_arr_size, ViolInfo_tint_data, ViolInfo_tint_size,
                   ti_arr_data, ti_arr_size, vi_arr_data, vi_arr_size);
      if (ti_arr_size[1] < 2) {
        i = 0;
        i1 = 0;
      } else {
        i = 1;
        i1 = ti_arr_size[1];
      }
      if (ti_arr_size[1] - 1 < 1) {
        b_i = 0;
      } else {
        b_i = ti_arr_size[1] - 1;
      }
      loop_ub = i1 - i;
      if (loop_ub == b_i) {
        idx_nan_size[0] = 1;
        idx_nan_size[1] = loop_ub;
        idxTLStp_s = (loop_ub / 2) << 1;
        idx0Info = idxTLStp_s - 2;
        for (trueCount = 0; trueCount <= idx0Info; trueCount += 2) {
          r = _mm_loadu_pd(&ti_arr_data[i + trueCount]);
          r1 = _mm_loadu_pd(&ti_arr_data[trueCount]);
          _mm_storeu_pd(&idx_nan_data[trueCount],
                        _mm_div_pd(_mm_sub_pd(r, r1), _mm_set1_pd(sfs)));
        }
        for (trueCount = idxTLStp_s; trueCount < loop_ub; trueCount++) {
          idx_nan_data[trueCount] =
              (ti_arr_data[i + trueCount] - ti_arr_data[trueCount]) / sfs;
        }
      } else {
        binary_expand_op_1(idx_nan_data, idx_nan_size, ti_arr_data, i, i1 - 1,
                           b_i - 1, sfs);
      }
      coder::b_round(idx_nan_data, idx_nan_size);
      b_loop_ub = idx_nan_size[1];
      maxval_size[1] = idx_nan_size[1];
      for (trueCount = 0; trueCount < b_loop_ub; trueCount++) {
        maxval_data[trueCount] = std::fmax(2.0, idx_nan_data[trueCount]);
      }
      if (loop_ub == b_i) {
        idx_nan_size[0] = 1;
        idx_nan_size[1] = loop_ub;
        idxTLStp_s = (loop_ub / 2) << 1;
        idx0Info = idxTLStp_s - 2;
        for (i1 = 0; i1 <= idx0Info; i1 += 2) {
          r = _mm_loadu_pd(&ti_arr_data[i + i1]);
          r1 = _mm_loadu_pd(&ti_arr_data[i1]);
          _mm_storeu_pd(&idx_nan_data[i1],
                        _mm_div_pd(_mm_sub_pd(r, r1), _mm_set1_pd(sfs)));
        }
        for (i1 = idxTLStp_s; i1 < loop_ub; i1++) {
          idx_nan_data[i1] = (ti_arr_data[i + i1] - ti_arr_data[i1]) / sfs;
        }
      } else {
        binary_expand_op_1(idx_nan_data, idx_nan_size, ti_arr_data, i, i1 - 1,
                           b_i - 1, sfs);
      }
      coder::b_round(idx_nan_data, idx_nan_size);
      Nti_arr_size[0] = 1;
      loop_ub = idx_nan_size[1];
      Nti_arr_size[1] = idx_nan_size[1];
      for (i = 0; i < loop_ub; i++) {
        Nti_arr_data[i] = std::fmax(2.0, idx_nan_data[i]);
      }
      if (maxval_size[1] - 1 < 1) {
        loop_ub = 0;
      } else {
        loop_ub = idx_nan_size[1] - 1;
      }
      b_maxval_size[0] = 1;
      b_maxval_size[1] = loop_ub;
      if (loop_ub - 1 >= 0) {
        std::copy(&maxval_data[0], &maxval_data[loop_ub], &b_maxval_data[0]);
      }
      Nti_arr_data[maxval_size[1] - 1] =
          1000.0 - coder::sum(b_maxval_data, b_maxval_size);
      for (i = 0; i < loop_ub_tmp; i++) {
        size_tmp_idx_1_tmp = i << 3;
        TimeCnstr[size_tmp_idx_1_tmp + 1] = tLimOut_tmin0f_arr_data[i];
        TimeCnstr[size_tmp_idx_1_tmp + 2] = tLimOut_tmax0f_arr_data[i];
        TimeCnstr[size_tmp_idx_1_tmp + 3] = tmin0_set_data[i];
        TimeCnstr[size_tmp_idx_1_tmp + 4] = tmax0_set_data[i];
      }
      idx0Info = pos_intsc_all_size[1];
      for (i = 0; i < idx0Info; i++) {
        size_tmp_idx_1_tmp = i << 3;
        TimeCnstr[size_tmp_idx_1_tmp + 5] = tmin0_setn_data[i];
        TimeCnstr[size_tmp_idx_1_tmp + 6] = tmax0_setn_data[i];
      }
      for (i = 0; i < loop_ub_tmp; i++) {
        TimeCnstr[(i << 3) + 7] = b_l_arr_data[i];
      }
      TrajType = tmp_size[1];
    } else {
      guard1 = true;
    }
  } else {
    guard1 = true;
  }
  if (guard1) {
    //  no traffic lights
    tfs_tmp = t_arrv_des_all_data[c_loop_ub + 1];
    tfs = std::fmax(tfs_tmp, CtrlInfo[0] + IntscDesEntrInfo[(b_i << 2) + 2]);
    if (d == 0.0) {
      //  one virtual point
      vi_arr_size[0] = 1;
      vi_arr_size[1] = 2;
      vi_arr_data[0] = CtrlInfo[5];
      sfs = pos_intsc_all_data[loop_ub + 1];
      vi_arr_data[1] =
          (3.0 * (sfs - CtrlInfo[4]) / (tfs - CtrlInfo[0]) - CtrlInfo[5]) / 2.0;
      si_arr_data[0] = CtrlInfo[4];
      si_arr_data[1] = sfs;
      ti_arr_size[0] = 1;
      ti_arr_size[1] = 2;
      ti_arr_data[0] = CtrlInfo[0];
      ti_arr_data[1] = tfs;
      Nti_arr_size[0] = 1;
      Nti_arr_size[1] = 1;
      Nti_arr_data[0] = 1000.0;
    } else {
      //  near one stop sign
      if ((StopInfo[0] == -1.0) && (CtrlPar[14] == 1.0)) {
        //  | StpMd == 0 % stopped
        sfs = CtrlInfo[4];
        tfs = CtrlInfo[0];
      } else {
        sfs = std::fmax(CtrlInfo[4], pos_intsc_all_data[loop_ub + 1] - 0.5);
        tfs = tfs_tmp;
      }
      vi_arr_size[0] = 1;
      vi_arr_size[1] = 2;
      vi_arr_data[0] = CtrlInfo[5];
      vi_arr_data[1] = vfs;
      si_arr_data[0] = CtrlInfo[4];
      si_arr_data[1] = sfs;
      ti_arr_size[0] = 1;
      ti_arr_size[1] = 2;
      ti_arr_data[0] = CtrlInfo[0];
      ti_arr_data[1] = tfs;
      Nti_arr_size[0] = 1;
      Nti_arr_size[1] = 1;
      Nti_arr_data[0] = 1000.0;
    }
  }
  WHOLE_TRAJ_GEN_fcn1(vi_arr_data, vi_arr_size, si_arr_data, ti_arr_data,
                      ti_arr_size, Nti_arr_data, Nti_arr_size, TRAJ_raw);
  //          if mod(t0s,10) == 0
  //          figure(100);
  //          subplot(311); hold on; grid on; box on;
  //          plot(TRAJ_raw(1,:),TRAJ_raw(2,:));
  //          subplot(312); hold on; grid on; box on;
  //          plot(TRAJ_raw(1,:),TRAJ_raw(3,:));
  //          plot(ti_arr, vi_arr,'o')
  //          subplot(313); hold on; grid on; box on;
  //          plot(TRAJ_raw(1,:),TRAJ_raw(4,:));
  //          plot(ti_arr,si_arr, 'o');
  //          end
  //          [C,IA,IC] = unique(TRAJ_raw(4,:)); % position
  for (i = 0; i < 1000; i++) {
    b_TRAJ_raw[i] = TRAJ_raw[i << 2];
  }
  int ipos[1000];
  size_tmp_idx_1_tmp = coder::unique_vector(b_TRAJ_raw, maxval_data,
                                            maxval_size, indx_data, ipos);
  //  time
  for (i = 0; i < size_tmp_idx_1_tmp; i++) {
    idxTLStp_s = (indx_data[i] - 1) << 2;
    idx0Info = i << 2;
    TRAJ[idx0Info] = TRAJ_raw[idxTLStp_s];
    TRAJ[idx0Info + 1] = TRAJ_raw[idxTLStp_s + 1];
    TRAJ[idx0Info + 2] = TRAJ_raw[idxTLStp_s + 2];
    TRAJ[idx0Info + 3] = TRAJ_raw[idxTLStp_s + 3];
  }
  //          TRAJ(:, 1:Nt) = TRAJ_raw;
  loop_ub = vi_arr_size[1];
  for (i = 0; i < loop_ub; i++) {
    Xis[3 * i] = ti_arr_data[i];
    Xis[3 * i + 1] = vi_arr_data[i];
    Xis[3 * i + 2] = si_arr_data[i];
  }
  //      end
  //  =================================================================== %
  //      time_calc = toc(time_Val);
  return TrajType;
}

//
// File trailer for RefTrjGnrtr_240503.cpp
//
// [EOF]
//
