//
// File: RefUpdate_230407.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "RefUpdate_230407.h"
#include "CAV_ctrl_mdl_wTraJ_241219_rtwutil.h"
#include "find.h"
#include "interp1.h"
#include "rt_nonfinite.h"
#include "xgeev.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
//
// initial states and time update
//
// Arguments    : const double TRAJ[4000]
//                const double CtrlInfo[14]
//                const double CtrlPar[15]
//                const double StopInfo[2]
//                double *vRef
//                double *sRef
//                double *UpdType
//                double TRAJ_CF[400]
// Return Type  : double
//
double RefUpdate_230407(const double TRAJ[4000], const double CtrlInfo[14],
                        const double CtrlPar[15], const double StopInfo[2],
                        double *vRef, double *sRef, double *UpdType,
                        double TRAJ_CF[400])
{
  creal_T eiga_data[49];
  creal_T vf_arr_data[3];
  double t_arr[100];
  double a_data[9];
  double Coeff[4];
  double aRef;
  double sp0;
  int tmp_data[1000];
  int TRAJ_size[2];
  int a_size[2];
  int b_tmp_size[2];
  int c_tmp_size[2];
  int t_arr_size[2];
  int tmp_size[2];
  int k1;
  boolean_T bv[1000];
  //  Ref states
  //  Actual states
  //  PV states
  //  Intersection position
  //  Intersection type (TL 10, Stop 20)
  //  current TL state (green 2, yellow 1 red 0)
  //  current maximum speed limit
  //  set the desired speed
  //  Car-following parameters
  //  1 Acceleration, 0: No acceleration after the destination (the last stop
  //  sign)
  //      v0s = v0Ref;
  //      a0s = a0Ref;
  sp0 = CtrlInfo[4] + CtrlInfo[7];
  //  Stop mode (Stopped -1, Nearby 0, Moving 1)
  //  hold the stopped time
  //  =================================================================== %
  //  output definition %
  //  TRAJ_CF: Time, Accel, Speed, Position
  //  grid size
  for (k1 = 0; k1 < 400; k1++) {
    TRAJ_CF[k1] = rtNaN;
  }
  for (k1 = 0; k1 < 1000; k1++) {
    bv[k1] = !std::isnan(TRAJ[(k1 << 2) + 1]);
  }
  coder::d_eml_find(bv, tmp_data, tmp_size);
  coder::d_eml_find(bv, tmp_data, b_tmp_size);
  for (k1 = 0; k1 < 1000; k1++) {
    bv[k1] = !std::isnan(TRAJ[(k1 << 2) + 1]);
  }
  coder::d_eml_find(bv, tmp_data, c_tmp_size);
  if (static_cast<short>(tmp_size[1]) > 1) {
    double t_arr_data[1000];
    double d;
    double t0_set;
    double y;
    int loop_ub;
    for (k1 = 0; k1 < 1000; k1++) {
      bv[k1] = !std::isnan(TRAJ[(k1 << 2) + 1]);
    }
    t_arr_size[0] = 1;
    coder::d_eml_find(bv, tmp_data, tmp_size);
    loop_ub = tmp_size[1];
    coder::d_eml_find(bv, tmp_data, tmp_size);
    t_arr_size[1] = tmp_size[1];
    for (k1 = 0; k1 < loop_ub; k1++) {
      t_arr_data[k1] = TRAJ[k1 << 2];
    }
    d = TRAJ[(c_tmp_size[1] - 1) << 2];
    if (d != 0.0) {
      //              if StpMd == 0 | (v0 < 1e-1 & a0s <= 0 & TLState == 0 &
      //              (IntscPos - s0s) < dsSafe) % nearby stop sign
      if (((StopInfo[0] != 1.0) && (CtrlInfo[4] < CtrlInfo[10])) ||
          ((CtrlInfo[5] < 0.1) && (CtrlInfo[6] <= 0.0) &&
           (CtrlInfo[12] == 0.0) && (CtrlInfo[10] - CtrlInfo[4] < 0.5))) {
        //  nearby stop sign
        *UpdType = 3.0;
        aRef = std::fmax(
            -4.0,
            std::fmin(std::fmin(-(CtrlInfo[5] * CtrlInfo[5]) / 2.0 /
                                    std::fmax(0.5, CtrlInfo[10] - CtrlInfo[4]),
                                CtrlInfo[3]),
                      -0.2));
        t0_set = std::fmin(std::fmax(0.0, -CtrlInfo[5] / aRef), 0.1);
        y = CtrlInfo[5] + aRef * t0_set;
        *sRef = (CtrlInfo[4] + CtrlInfo[5] * t0_set) +
                aRef * (t0_set * t0_set) / 2.0;
      } else {
        double TRAJ_data[1000];
        *UpdType = 1.0;
        t0_set = std::fmin(d, std::fmax(TRAJ[0], CtrlInfo[0] + 0.1));
        loop_ub = static_cast<short>(b_tmp_size[1]);
        TRAJ_size[0] = 1;
        TRAJ_size[1] = static_cast<short>(b_tmp_size[1]);
        for (k1 = 0; k1 < loop_ub; k1++) {
          TRAJ_data[k1] = TRAJ[(k1 << 2) + 1];
        }
        aRef = coder::interp1(t_arr_data, t_arr_size, TRAJ_data, TRAJ_size,
                              t0_set);
        TRAJ_size[0] = 1;
        TRAJ_size[1] = static_cast<short>(b_tmp_size[1]);
        for (k1 = 0; k1 < loop_ub; k1++) {
          TRAJ_data[k1] = TRAJ[(k1 << 2) + 2];
        }
        y = coder::interp1(t_arr_data, t_arr_size, TRAJ_data, TRAJ_size,
                           t0_set);
        TRAJ_size[0] = 1;
        TRAJ_size[1] = static_cast<short>(b_tmp_size[1]);
        for (k1 = 0; k1 < loop_ub; k1++) {
          TRAJ_data[k1] = TRAJ[(k1 << 2) + 3];
        }
        *sRef = coder::interp1(t_arr_data, t_arr_size, TRAJ_data, TRAJ_size,
                               t0_set);
      }
    } else {
      *UpdType = -2.0;
      aRef = 0.0;
      y = 0.0;
      *sRef = 0.0;
    }
    if (CtrlPar[8] == 0.0) {
      double c7;
      // %%%% IDM %%%%%%
      //      v_des = 30;
      //      as = acc_comf*(1 - (v0s/v_des)^acc_exp - (d_des/ds)^2);
      c7 = (CtrlPar[11] +
            std::fmax(0.0, CtrlInfo[5] * CtrlPar[10] +
                               CtrlInfo[5] * (CtrlInfo[5] - CtrlInfo[8]) /
                                   5.6568542494923806)) /
           (sp0 - CtrlInfo[4]);
      t0_set = std::fmax(-4.0, 4.0 * (1.0 - c7 * c7));
      if (aRef > t0_set) {
        if (CtrlInfo[5] < 0.1) {
          aRef = -1.0;
          y = CtrlInfo[5] - 0.1;
          *sRef = (CtrlInfo[4] + CtrlInfo[5] * 0.1) - 0.005000000000000001;
        } else {
          aRef = t0_set;
          y = std::fmax(0.0, t0_set * 0.1 + CtrlInfo[5]);
          *sRef = (CtrlInfo[4] + CtrlInfo[5] * 0.1) +
                  0.5 * t0_set * 0.010000000000000002;
        }
        *UpdType = 2.0;
      }
    } else if (CtrlPar[8] == 1.0) {
      double vf_re_arr_data[3];
      double y_data[3];
      double c1o;
      double c1o_tmp;
      double c2;
      double c3o_tmp_tmp;
      double c4;
      double c5o;
      double c7;
      double tpf;
      double vf;
      int a_data_tmp;
      int j;
      int k2;
      int nTrailingZeros;
      boolean_T b_tmp_data[3];
      //  boundary condition optimization considering PV
      //     %%
      //  constant PV acceleration assumption
      if (CtrlInfo[9] > 0.0) {
        t0_set = CtrlInfo[13];
      } else if (CtrlInfo[9] == 0.0) {
        t0_set = CtrlInfo[8];
      } else {
        t0_set = 0.0;
      }
      //  compute the time to reach v_des when maintaining ap0
      tpf = std::fmin(CtrlPar[9],
                      std::fmax(0.0, (t0_set - CtrlInfo[8]) / CtrlInfo[9]));
      //  estimate the PV distance after tpf
      c7 = std::fmax(0.0, CtrlInfo[8] + CtrlInfo[9] * tpf);
      //      b0 = sp0 + vp0*tf - d_min; % constant PV speed assumption
      c1o_tmp = rt_powd_snf(CtrlPar[9], 3.0);
      c1o = 6.0 / c1o_tmp;
      c3o_tmp_tmp = CtrlPar[9] * CtrlPar[9];
      vf = -6.0 / c3o_tmp_tmp;
      c5o = 2.0 / CtrlPar[9];
      c2 = -2.0 * c1o * CtrlInfo[4] + vf * CtrlInfo[5];
      c4 = -vf * CtrlInfo[4] + 2.0 * CtrlInfo[5] / CtrlPar[9];
      //  Eaf = c0o + c1o*(sf - s0)^2 + c2o*(sf - s0) + c3o*(sf - s0)*vf +
      //  c4o*vf + c5o*vf^2; Eaf = c0 + c1*sf^2 + c2*sf + c3*sf*vf + c4*vf +
      //  c5*vf^2;
      //      sf_cand0 = (c3*c4 - 2*c2*c5)/(4*c1*c5 - c3^2);
      //      vf_cand0 = (c2*c3 - 2*c1*c4)/(4*c1*c5 - c3^2);
      //      eps_s = 1e-1;
      //      cond0 = sf_cand0 + b1*vf_cand0 - b0 <= eps_s;
      //  %     if cond0 == true
      //  %         sf = sf_cand0;
      //  %         vf = vf_cand0;
      //  %     else
      //          % active constraint
      //          vf = -((2*c1*b1-c3)*b0 + (c2*b1 - c4))/2/(c3*b1 - c5 -
      //          c1*b1^2); sf = b0 - b1*vf;
      //  %     end
      c7 = ((sp0 + ((CtrlInfo[8] * tpf + 0.5 * CtrlInfo[9] * (tpf * tpf)) +
                    t0_set * (CtrlPar[9] - tpf))) +
            0.0 * (c7 * c7)) -
           CtrlPar[11];
      Coeff[0] = 4.0 * c1o * 0.0;
      Coeff[1] = 6.0 * c1o * CtrlPar[10] * 0.0 - 3.0 * vf * 0.0;
      //  active 1st inequality constraint (dd_m)
      t0_set = c2 * CtrlPar[10];
      Coeff[3] = ((-2.0 * c1o * CtrlPar[10] * c7 - t0_set) + vf * c7) + c4;
      tpf = CtrlPar[10] * CtrlPar[10];
      Coeff[2] = (((2.0 * c1o * tpf - 4.0 * c1o * c7 * 0.0) - 2.0 * c2 * 0.0) -
                  2.0 * vf * CtrlPar[10]) +
                 2.0 * c5o;
      vf_arr_data[0].re = 0.0;
      vf_arr_data[0].im = 0.0;
      vf_arr_data[1].re = 0.0;
      vf_arr_data[1].im = 0.0;
      vf_arr_data[2].re = 0.0;
      vf_arr_data[2].im = 0.0;
      k1 = 1;
      while ((k1 <= 4) && (!(Coeff[k1 - 1] != 0.0))) {
        k1++;
      }
      k2 = 4;
      while ((k2 >= k1) && (!(Coeff[k2 - 1] != 0.0))) {
        k2--;
      }
      nTrailingZeros = 3 - k2;
      if (k1 < k2) {
        double ctmp[4];
        int companDim;
        boolean_T exitg1;
        companDim = k2 - k1;
        exitg1 = false;
        while ((!exitg1) && (companDim > 0)) {
          boolean_T exitg2;
          j = 0;
          exitg2 = false;
          while ((!exitg2) && (j + 1 <= companDim)) {
            ctmp[j] = Coeff[k1 + j] / Coeff[k1 - 1];
            if (std::isinf(std::abs(ctmp[j]))) {
              exitg2 = true;
            } else {
              j++;
            }
          }
          if (j + 1 > companDim) {
            exitg1 = true;
          } else {
            k1++;
            companDim--;
          }
        }
        if (companDim < 1) {
          loop_ub = 4 - k2;
        } else {
          a_size[0] = companDim;
          a_size[1] = companDim;
          loop_ub = companDim * companDim;
          std::memset(&a_data[0], 0,
                      static_cast<unsigned int>(loop_ub) * sizeof(double));
          for (k1 = 0; k1 <= companDim - 2; k1++) {
            a_data_tmp = companDim * k1;
            a_data[a_data_tmp] = -ctmp[k1];
            a_data[(k1 + a_data_tmp) + 1] = 1.0;
          }
          a_data[companDim * (companDim - 1)] = -ctmp[companDim - 1];
          if (nTrailingZeros >= 0) {
            std::memset(&vf_arr_data[0], 0,
                        static_cast<unsigned int>(nTrailingZeros + 1) *
                            sizeof(creal_T));
          }
          if (companDim == 1) {
            for (k1 = 0; k1 < companDim; k1++) {
              eiga_data[k1].re = a_data[k1];
              eiga_data[k1].im = 0.0;
            }
          } else {
            coder::internal::lapack::xgeev(a_data, a_size, eiga_data, k1);
          }
          for (k1 = 0; k1 < companDim; k1++) {
            vf_arr_data[(k1 - k2) + 4] = eiga_data[k1];
          }
          loop_ub = (companDim - k2) + 4;
        }
      } else {
        loop_ub = 4 - k2;
      }
      for (k1 = 0; k1 < loop_ub; k1++) {
        d = vf_arr_data[k1].im;
        vf_re_arr_data[k1] = d;
        y_data[k1] = std::abs(d);
      }
      for (k1 = 0; k1 < loop_ub; k1++) {
        b_tmp_data[k1] =
            ((vf_arr_data[k1].re > -0.01) && (y_data[k1] < 1.0E-8));
      }
      loop_ub--;
      k1 = 0;
      j = 0;
      for (a_data_tmp = 0; a_data_tmp <= loop_ub; a_data_tmp++) {
        if (b_tmp_data[a_data_tmp]) {
          k1++;
          vf_re_arr_data[j] = vf_arr_data[a_data_tmp].re;
          j++;
        }
      }
      if (k1 != 0) {
        vf = vf_re_arr_data[0];
      } else {
        vf = -((2.0 * c1o * CtrlPar[10] - vf) * c7 + (t0_set - c4)) / 2.0 /
             ((vf * CtrlPar[10] - c5o) - c1o * tpf);
      }
      t0_set = (c7 - CtrlPar[10] * vf) - 0.0 * (vf * vf);
      //  inputs: v0s, s0s, vfs, sfs, tfs, dts
      tpf = std::fmax(0.1, CtrlPar[9]);
      c1o = 12.0 * CtrlInfo[4] - 12.0 * t0_set;
      c5o = ((c1o + 6.0 * tpf * CtrlInfo[5]) + 6.0 * tpf * vf) /
            rt_powd_snf(tpf, 3.0);
      c7 = 6.0 * CtrlInfo[4] - 6.0 * t0_set;
      c2 = ((c7 + 4.0 * tpf * CtrlInfo[5]) + 2.0 * tpf * vf) / (tpf * tpf);
      t0_set = c5o * 0.1 - c2;
      if ((CtrlInfo[5] < 0.5) && (CtrlInfo[12] == 0.0) &&
          ((((sp0 - CtrlInfo[4]) - CtrlPar[11] < 2.0) && (CtrlInfo[8] < 0.1)) ||
           (CtrlInfo[10] - CtrlInfo[4] < 0.5))) {
        aRef = std::fmax(
            -4.0,
            std::fmin(
                std::fmin(-(CtrlInfo[5] * CtrlInfo[5]) / 2.0 /
                              std::fmax(0.5, std::fmin(sp0, CtrlInfo[10]) -
                                                 CtrlInfo[4]),
                          CtrlInfo[3]),
                -0.2));
        t0_set = std::fmin(std::fmax(0.0, -CtrlInfo[5] / aRef), 0.1);
        y = CtrlInfo[5] + aRef * t0_set;
        *sRef = (CtrlInfo[4] + CtrlInfo[5] * t0_set) +
                aRef * (t0_set * t0_set) / 2.0;
        *UpdType = -4.0;
      } else if (aRef > t0_set) {
        double b[100];
        double v_arr[100];
        aRef = t0_set;
        y = (c5o * 0.010000000000000002 / 2.0 - c2 * 0.1) + CtrlInfo[5];
        *sRef = ((c5o * 0.0010000000000000002 / 6.0 -
                  c2 * 0.010000000000000002 / 2.0) +
                 CtrlInfo[5] * 0.1) +
                CtrlInfo[4];
        *UpdType = 2.0;
        t_arr[99] = CtrlPar[9];
        t_arr[0] = 0.0;
        if (-CtrlPar[9] == 0.0) {
          t0_set = CtrlPar[9] / 99.0;
          for (k1 = 0; k1 < 98; k1++) {
            t_arr[k1 + 1] =
                (2.0 * (static_cast<double>(k1) + 2.0) - 101.0) * t0_set;
          }
        } else if ((CtrlPar[9] < 0.0) &&
                   (std::abs(CtrlPar[9]) > 8.9884656743115785E+307)) {
          t0_set = CtrlPar[9] / 99.0;
          for (k1 = 0; k1 < 98; k1++) {
            t_arr[k1 + 1] = t0_set * (static_cast<double>(k1) + 1.0);
          }
        } else {
          t0_set = CtrlPar[9] / 99.0;
          for (k1 = 0; k1 < 98; k1++) {
            t_arr[k1 + 1] = (static_cast<double>(k1) + 1.0) * t0_set;
          }
        }
        c5o = ((c1o + 6.0 * CtrlPar[9] * CtrlInfo[5]) + 6.0 * CtrlPar[9] * vf) /
              c1o_tmp;
        c2 = ((c7 + 4.0 * CtrlPar[9] * CtrlInfo[5]) + 2.0 * CtrlPar[9] * vf) /
             c3o_tmp_tmp;
        //  TRAJ_CF: Time, Accel, Speed, Position
        d = CtrlInfo[5];
        t0_set = CtrlInfo[0];
        for (k1 = 0; k1 < 100; k1++) {
          tpf = t_arr[k1];
          c7 = std::fmax(0.0, (c5o * (tpf * tpf) / 2.0 - c2 * tpf) + d);
          v_arr[k1] = c7;
          loop_ub = k1 << 2;
          TRAJ_CF[loop_ub] = t0_set + tpf;
          TRAJ_CF[loop_ub + 1] = c5o * tpf - c2;
          TRAJ_CF[loop_ub + 2] = c7;
        }
        for (a_data_tmp = 0; a_data_tmp <= 96; a_data_tmp += 2) {
          __m128d r;
          __m128d r1;
          r = _mm_loadu_pd(&t_arr[a_data_tmp + 1]);
          r1 = _mm_loadu_pd(&t_arr[a_data_tmp]);
          _mm_storeu_pd(&t_arr[a_data_tmp], _mm_sub_pd(r, r1));
        }
        t_arr[98] = t_arr[99] - t_arr[98];
        t0_set = 0.0;
        tpf = v_arr[0];
        b[0] = 0.0;
        for (k1 = 0; k1 < 99; k1++) {
          c7 = v_arr[k1 + 1];
          t0_set += t_arr[k1] * ((tpf + c7) / 2.0);
          tpf = c7;
          b[k1 + 1] = t0_set;
        }
        for (k1 = 0; k1 < 100; k1++) {
          TRAJ_CF[(k1 << 2) + 3] = b[k1] + CtrlInfo[4];
        }
      }
    }
    aRef = std::fmax(-4.0, std::fmin(4.0, aRef));
    *vRef = std::fmax(0.0, std::fmin(CtrlInfo[13] + 10.0, y));

    //  Accel after destination: Yes 1, No 0
  } else if ((CtrlInfo[0] > StopInfo[1] + 2.0) && (StopInfo[1] != 0.0) &&
             (CtrlPar[14] == 1.0)) {
    //  accel right after stopped
    *UpdType = 4.0;
    aRef = 1.0;
    *vRef = CtrlInfo[5] + 0.1;
    *sRef = (CtrlInfo[4] + CtrlInfo[5] * 0.1) + 0.005000000000000001;
  } else {
    *UpdType = -1.0;
    aRef = 0.0;
    *vRef = 0.0;
    *sRef = CtrlInfo[4];
  }
  //  UpdType
  //  -1: No Ref
  //  1: Free-flow (normal)
  //  2: Car-following
  //
  return aRef;
}

//
// File trailer for RefUpdate_230407.cpp
//
// [EOF]
//
