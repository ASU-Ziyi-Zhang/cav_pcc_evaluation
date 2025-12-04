//
// File: CAV_ctrl_mdl_wTraJ_241219.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

// Include Files
#include "CAV_ctrl_mdl_wTraJ_241219.h"
#include "GrnWndSlctr_230512.h"
#include "RefTrjGnrtr_240503.h"
#include "RefUpdate_230407.h"
#include "diff.h"
#include "find.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
//
// Arguments    : const double IntscInfo[30]
//                const double SpdLimInfo[4]
//                const double CtrlInfo[14]
//                const double CtrlPar[15]
//                const double StopInfo[2]
//                double *aRef
//                double *vRef
//                double *sRef
//                double *UpdType
//                double time_LTtraj[1000]
//                double acc_LTtraj[1000]
//                double vel_LTtraj[1000]
//                double pos_LTtraj[1000]
//                double time_STtraj[100]
//                double acc_STtraj[100]
//                double vel_STtraj[100]
//                double pos_STtraj[100]
// Return Type  : void
//
void CAV_ctrl_mdl_wTraJ_241219(
    const double IntscInfo[30], const double SpdLimInfo[4],
    const double CtrlInfo[14], const double CtrlPar[15],
    const double StopInfo[2], double *aRef, double *vRef, double *sRef,
    double *UpdType, double time_LTtraj[1000], double acc_LTtraj[1000],
    double vel_LTtraj[1000], double pos_LTtraj[1000], double time_STtraj[100],
    double acc_STtraj[100], double vel_STtraj[100], double pos_STtraj[100])
{
  static double TRAJ[4000];
  double TRAJ_CF[400];
  double TimeCnstr[80];
  double IntscPrvInfo[60];
  double IntscGrnSlctdInfo[50];
  double IntscDesEntrInfo[48];
  double b_tmp_data[37];
  double IntscPrvInfo_s_data[36];
  double grn_arr[20];
  double CtrlInfo_data[11];
  double phase_intsc_data[10];
  double time_cycle_intsc_data[10];
  double OptSlvPrf[2];
  double NxtIntscCntDstMrgn;
  double SpdDes;
  double aDes;
  double bDes;
  double d;
  double dGap;
  double dtGrnEndTimeMrgn;
  double s_prv;
  double t0_arrv_des_s;
  double t0s;
  double v0;
  double vfs;
  double vp0;
  int idx_grn_phase_find_data[10];
  int tmp_data[10];
  int CtrlInfo_size[2];
  int tmp_size[2];
  int NIntsc;
  int i;
  int i1;
  int idx;
  int ii;
  int loop_ub;
  signed char ii_data[5];
  signed char b_ii_data[2];
  boolean_T t_arrv[10];
  boolean_T bv[5];
  boolean_T bv1[5];
  boolean_T exitg1;
  //  =================================================================== %
  //      coder.extrinsic('tic');
  //      coder.extrinsic('toc');
  //      time_Val=tic;
  //  =================================================================== %
  //  Input processing %
  //  traffic lights between the two stop signs
  //  include the last stop sign
  //  initial states and time update
  //  Ref states
  //  position
  //  speed
  //  acceleration
  //  Actual states
  //  position
  //  speed
  //  acceleration
  //  current IntscType
  //  current TL state (green 2, yellow 1 red 0)
  //  connectivity range
  //  distance margin to count next intersections in preview horizon when
  //  traffic light is NOT red
  //  must be larger than zero
  NxtIntscCntDstMrgn = CtrlPar[1];
  //  virtual point distance margin from the last intersection
  //  desired acceleration
  //  =================================================================== %
  //  Output definition
  for (i = 0; i < 60; i++) {
    IntscPrvInfo[i] = rtNaN;
  }
  for (i = 0; i < 20; i++) {
    grn_arr[i] = rtNaN;
  }
  //  =================================================================== %
  s_prv = CtrlPar[0] + CtrlInfo[4];
  if (CtrlInfo[12] == 0.0) {
    //  red light
    //  In RR, next intsc. will be the current intsc. 0.5 m after the
    //  nearest intsc. pos
    NxtIntscCntDstMrgn = 0.0;
    //  changed [08/24/2021]
  }
  //  find all intersections in preview horizon
  for (i = 0; i < 5; i++) {
    d = IntscInfo[6 * i];
    bv[i] = ((d - CtrlInfo[4]) - NxtIntscCntDstMrgn > 0.0);
    bv1[i] = (d - s_prv < 0.0);
  }
  idx = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 5)) {
    if (bv[ii] && bv1[ii]) {
      idx++;
      ii_data[idx - 1] = static_cast<signed char>(ii + 1);
      if (idx >= 5) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (idx < 1) {
    loop_ub = 0;
  } else {
    loop_ub = idx;
  }
  //  fine all speed limits in preview horizon
  idx = 0;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 2)) {
    if (SpdLimInfo[ii << 1] - CtrlInfo[4] > 0.0) {
      idx++;
      b_ii_data[idx - 1] = static_cast<signed char>(ii + 1);
      if (idx >= 2) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (idx < 1) {
    idx = 0;
  }
  if (s_prv < IntscInfo[24]) {
    NIntsc = loop_ub + 1;
    if (loop_ub + 1 != 1) {
      //  virtual point must be 100 meters far from the last intersection
      //  position
      //              s_VrtPt = max(s_prv, pos_intsc(idx_NxtIntsc(end)) +
      //              VrtPtDstMrgn);
      NxtIntscCntDstMrgn =
          IntscInfo[6 * (ii_data[loop_ub - 1] - 1)] + CtrlPar[2];
    } else {
      //  no intersections in preview horizon
      //              s_VrtPt = s_prv;
      //              s_VrtPt = s0s + VrtPtDstMrgn;
      NxtIntscCntDstMrgn =
          CtrlInfo[4] +
          CtrlPar[5] * CtrlPar[5] / (0.66666666666666663 * CtrlPar[12]) / 2.0;
    }
    for (i = 0; i < loop_ub; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        IntscPrvInfo_s_data[i1 + 6 * i] = IntscInfo[i1 + 6 * (ii_data[i] - 1)];
      }
    }
    IntscPrvInfo_s_data[6 * loop_ub] = NxtIntscCntDstMrgn;
    IntscPrvInfo_s_data[6 * loop_ub + 1] = 0.0;
    IntscPrvInfo_s_data[6 * loop_ub + 2] = rtNaN;
    IntscPrvInfo_s_data[6 * loop_ub + 3] = rtNaN;
    IntscPrvInfo_s_data[6 * loop_ub + 4] = rtNaN;
    IntscPrvInfo_s_data[6 * loop_ub + 5] = rtNaN;
  } else {
    //  stop sign in preview horizon
    if (loop_ub == 0) {
      loop_ub = 1;
      ii_data[0] = 5;
    }
    NIntsc = loop_ub;
    for (i = 0; i < loop_ub; i++) {
      for (i1 = 0; i1 < 6; i1++) {
        IntscPrvInfo_s_data[i1 + 6 * i] = IntscInfo[i1 + 6 * (ii_data[i] - 1)];
      }
    }
    if (((CtrlInfo[11] == 20.0) || std::isnan(CtrlInfo[11])) &&
        (CtrlInfo[12] == 2.0)) {
      double b_CtrlInfo[6];
      //  stopsign and green
      NIntsc = 1;
      b_CtrlInfo[0] = CtrlInfo[4] + CtrlPar[5] * CtrlPar[5] /
                                        (0.66666666666666663 * CtrlPar[12]) /
                                        2.0;
      b_CtrlInfo[1] = 0.0;
      b_CtrlInfo[2] = rtNaN;
      b_CtrlInfo[3] = rtNaN;
      b_CtrlInfo[4] = rtNaN;
      b_CtrlInfo[5] = rtNaN;
      for (i = 0; i < 6; i++) {
        IntscPrvInfo_s_data[i] = b_CtrlInfo[i];
      }
    }
  }
  for (i = 0; i < NIntsc; i++) {
    for (i1 = 0; i1 < 6; i1++) {
      ii = i1 + 6 * i;
      IntscPrvInfo[ii] = IntscPrvInfo_s_data[ii];
    }
  }
  for (i = 0; i < idx; i++) {
    NIntsc = (b_ii_data[i] - 1) << 1;
    ii = i << 1;
    grn_arr[ii] = SpdLimInfo[NIntsc];
    grn_arr[ii + 1] = SpdLimInfo[NIntsc + 1];
  }
  //      IntscPrvInfo(:,1:NIntsc) = IntscInfo;
  //      SpdLimPrvInfo(:, 1:NSpdLim) = SpdLimInfo;
  //  =================================================================== %
  //      time_calc = toc(time_Val);
  //  =================================================================== %
  //      coder.extrinsic('tic');
  //      coder.extrinsic('toc');
  //      time_Val=tic;
  //  =================================================================== %
  //  Input processing %
  for (i = 0; i < 10; i++) {
    t_arrv[i] = !std::isnan(IntscPrvInfo[6 * i]);
  }
  coder::eml_find(t_arrv, tmp_data, tmp_size);
  if (tmp_size[1] < 1) {
    loop_ub = 0;
  } else {
    for (i = 0; i < 10; i++) {
      t_arrv[i] = !std::isnan(IntscPrvInfo[6 * i]);
    }
    coder::eml_find(t_arrv, tmp_data, tmp_size);
    loop_ub = tmp_size[1];
  }
  //  traffic lights between the two stop signs
  //  include the last stop sign
  //  IntscType: 20 stop signs, 10 traffic lights, 0 virtual points
  //  traffic light cycle info.
  for (i = 0; i < loop_ub; i++) {
    time_cycle_intsc_data[i] = IntscPrvInfo[6 * i + 2];
    phase_intsc_data[i] = IntscPrvInfo[6 * i + 5];
  }
  //  assume constant max. speed
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
  //  PV states
  dGap = CtrlInfo[7];
  vp0 = CtrlInfo[8];
  //  =================================================================== %
  //  parameter setting
  //  set the safety time margin to initial and end timing of green lights
  dtGrnEndTimeMrgn = CtrlPar[4];
  //  set the desired speed
  SpdDes = CtrlPar[5];
  //  Car-following parameters
  //  desired time gap
  aDes = CtrlPar[12];
  //  desired acceleration
  bDes = CtrlPar[13];
  //  desired braking
  //  To calculate the required minimum time
  //      vmax_arr = spd_max*ones(1, N_intsc); % size: 1 x (N_intsc + 1)
  NxtIntscCntDstMrgn = std::fmin(1.2 * CtrlPar[5], grn_arr[1]);
  //  size: 1 x (N_intsc + 1)
  if (IntscPrvInfo[6 * (loop_ub - 1) + 1] == 20.0) {
    //  stop sign at the end of preview horizon
    vfs = 0.0;
  } else {
    //  virtual point at the end of preview horizon
    vfs = CtrlPar[5];
  }
  //  =================================================================== %
  //  output definition
  //  traffic signal position,
  //  green light start and end timing
  for (i = 0; i < 50; i++) {
    IntscGrnSlctdInfo[i] = rtNaN;
  }
  if (loop_ub - 1 < 1) {
    idx = 0;
  } else {
    idx = loop_ub - 1;
  }
  for (i = 0; i < idx; i++) {
    IntscGrnSlctdInfo[5 * i] = IntscPrvInfo[6 * i];
  }
  //  desired arriving time information at each intersection
  //  include start and end point
  //  position, desired entering time, required travel time,
  //  intersection type
  for (i = 0; i < 48; i++) {
    IntscDesEntrInfo[i] = rtNaN;
  }
  CtrlInfo_size[0] = 1;
  CtrlInfo_size[1] = loop_ub + 1;
  CtrlInfo_data[0] = CtrlInfo[4];
  for (i = 0; i < loop_ub; i++) {
    d = IntscPrvInfo[6 * i];
    ii = i << 2;
    IntscDesEntrInfo[ii] = d;
    IntscDesEntrInfo[ii + 3] = IntscPrvInfo[6 * i + 1];
    CtrlInfo_data[i + 1] = d;
  }
  coder::diff(CtrlInfo_data, CtrlInfo_size, b_tmp_data, tmp_size);
  if (tmp_size[1] == loop_ub) {
    for (i = 0; i < loop_ub; i++) {
      IntscDesEntrInfo[(i << 2) + 2] = b_tmp_data[i] / NxtIntscCntDstMrgn;
    }
  } else {
    binary_expand_op(IntscDesEntrInfo, loop_ub, b_tmp_data, tmp_size,
                     NxtIntscCntDstMrgn);
  }
  //      IntscDesEntrInfo(3,1:N_intsc) = diff([s0s, pos_intsc])./SpdDes;
  //  =================================================================== %
  //  algorithm
  t0_arrv_des_s = CtrlInfo[0];
  s_prv = CtrlPar[5];
  if (loop_ub != 1) {
    //  traffic lights in the middle
    for (idx = 0; idx < loop_ub; idx++) {
      double SpdDesFinal;
      double dist_s;
      double t_arrv_des_ini;
      double trv_time_des_s;
      if (idx + 1 == 1) {
        NxtIntscCntDstMrgn = CtrlInfo[4];
      } else {
        NxtIntscCntDstMrgn = IntscPrvInfo[6 * (idx - 1)];
      }
      dist_s = IntscPrvInfo[6 * idx] - NxtIntscCntDstMrgn;
      //              SpdDesPrv = (pos_prv_intsc - s0s)/(t_arrv_des_fin - t0s);
      if (idx + 1 == 1) {
        //  accelerating
        NxtIntscCntDstMrgn = 3.0 * v0;
        SpdDesFinal = std::fmin(
            SpdDes,
            -0.5 * v0 + 1.5 * dist_s /
                            ((-NxtIntscCntDstMrgn +
                              std::sqrt(std::fmax(NxtIntscCntDstMrgn *
                                                          NxtIntscCntDstMrgn -
                                                      10.0 * (-3.0 * dist_s),
                                                  0.0))) /
                             2.0 / 2.5));
        trv_time_des_s = 3.0 * dist_s / (v0 + 2.0 * SpdDesFinal);
      } else if (idx + 1 == loop_ub) {
        //  braking to stop
        SpdDesFinal = vfs;
        trv_time_des_s =
            tfs_des_cal_fcn(s_prv, vfs, SpdDes, dist_s, aDes, bDes);
      } else {
        SpdDesFinal = SpdDes;
        trv_time_des_s =
            tfs_des_cal_fcn(s_prv, SpdDes, SpdDes, dist_s, aDes, bDes);
      }
      t_arrv_des_ini = t0_arrv_des_s + trv_time_des_s;
      if (idx + 1 != loop_ub) {
        double grn_ini[10];
        double b_t_arrv;
        double t_arrv_des_fin_max;
        double t_arrv_des_fin_min;
        NxtIntscCntDstMrgn = std::floor(t0s / time_cycle_intsc_data[idx]);
        //                  cyc_numb_arr = [cyc_numb_ini:1:cyc_numb_ini + Ncyc -
        //                  1];
        //  must use constant value vectors
        t_arrv_des_fin_min = IntscPrvInfo[6 * idx + 2];
        s_prv = phase_intsc_data[idx];
        for (i = 0; i < 10; i++) {
          grn_ini[i] = t_arrv_des_fin_min *
                           (NxtIntscCntDstMrgn + static_cast<double>(i)) -
                       s_prv;
        }
        b_t_arrv = t_arrv_des_ini;
        //                  if idx == 1 & dGap > dist_s - 5 & dGap < dist_s &
        //                  vp0 > 1 % dGap < 150
        if ((idx + 1 == 1) && (dGap > dist_s - vp0 * 5.0) && (dGap < dist_s) &&
            (vp0 > 1.0)) {
          //  dGap < 150
          //  consider the desired time gap if the PV is nearby
          //  otherwise, require hard braking
          b_t_arrv = std::fmax(
              t_arrv_des_ini,
              (t0_arrv_des_s + std::fmax(0.0, (dist_s - dGap) / vp0)) +
                  CtrlPar[10]);
        }
        t_arrv_des_fin_min = IntscPrvInfo[6 * idx + 3];
        NxtIntscCntDstMrgn = IntscPrvInfo[6 * idx + 4];
        //  spdType: 1 - max speed, 2 - min speed
        for (i = 0; i < 10; i++) {
          NIntsc = i << 1;
          d = grn_ini[i];
          grn_arr[NIntsc] = d;
          s_prv = (d + t_arrv_des_fin_min) + NxtIntscCntDstMrgn;
          grn_arr[NIntsc + 1] = s_prv;
          t_arrv[i] = ((b_t_arrv > d) && (b_t_arrv < s_prv));
        }
        coder::eml_find(t_arrv, idx_grn_phase_find_data, tmp_size);
        i = tmp_size[1];
        if (tmp_size[1] == 0) {
          //  t_arrv is NOT in green phase
          for (i = 0; i < 10; i++) {
            t_arrv[i] = (b_t_arrv < grn_arr[i << 1]);
          }
          i = 1;
          coder::eml_find(t_arrv, tmp_data, tmp_size);
          idx_grn_phase_find_data[0] = tmp_data[0];
          ii = 1;
        } else {
          //  t_arrv is in green phase
          ii = 0;
          //  no need to consider PV when arriving time is in the current green
          b_t_arrv = t_arrv_des_ini;
        }
        if (idx + 1 == 1) {
          t_arrv_des_fin_min = grn_arr[(idx_grn_phase_find_data[0] - 1) << 1];
          if (!(t0s > t_arrv_des_fin_min)) {
            t_arrv_des_fin_min =
                grn_arr[(idx_grn_phase_find_data[0] - 1) << 1] + CtrlPar[3];
          }
        } else {
          t_arrv_des_fin_min =
              grn_arr[(idx_grn_phase_find_data[0] - 1) << 1] + CtrlPar[3];
        }
        i1 = (idx << 2) + 2;
        d = IntscDesEntrInfo[i1];
        NxtIntscCntDstMrgn =
            grn_arr[((idx_grn_phase_find_data[i - 1] - 1) << 1) + 1];
        t_arrv_des_fin_max =
            std::fmax(std::fmin(NxtIntscCntDstMrgn, t0_arrv_des_s + d),
                      NxtIntscCntDstMrgn - dtGrnEndTimeMrgn);
        if (trv_time_des_s < dtGrnEndTimeMrgn) {
          //  to avoid zero trvl time
          t_arrv_des_fin_max = NxtIntscCntDstMrgn;
        }
        t_arrv_des_ini =
            std::fmin(t_arrv_des_fin_max - 0.1,
                      std::fmax(b_t_arrv, t_arrv_des_fin_min + 0.1));
        //                  t_arrv_des_fin = max(t_arrv_des_ini, grn_set(1));
        //                  t_arrv_des_fin = max(t_arrv_des_ini, grn_set(1)+
        //                  dtGrnIniTimeMrgn_s);
        NxtIntscCntDstMrgn = t_arrv_des_ini - t0_arrv_des_s;
        s_prv = dist_s / NxtIntscCntDstMrgn;
        //                  SpdDesPrv = SpdDesFinal;
        t0_arrv_des_s = t_arrv_des_ini;
        //  if speed is required to be higher than max speed.
        d = std::fmin(d, NxtIntscCntDstMrgn - 0.1);
        IntscDesEntrInfo[i1] = d;
        IntscGrnSlctdInfo[5 * idx + 1] = t_arrv_des_fin_min;
        IntscGrnSlctdInfo[5 * idx + 2] = t_arrv_des_fin_max;
        //                  IntscGrnSlctdInfo(2,idx) = grn_ini(1);
        //                  IntscGrnSlctdInfo(3,idx) = grn_end(1);
        IntscGrnSlctdInfo[5 * idx + 3] = ii;
        IntscGrnSlctdInfo[5 * idx + 4] = SpdDesFinal;
      }
      IntscDesEntrInfo[(idx << 2) + 1] = t_arrv_des_ini;
    }
  } else {
    double dist_s;
    dist_s = std::fmax(0.0, IntscPrvInfo[0] - CtrlInfo[4]);
    if (IntscPrvInfo[1] == 20.0) {
      //  in case of only one stop
      IntscDesEntrInfo[1] =
          CtrlInfo[0] + tfs_des_cal_fcn(CtrlInfo[5], vfs, CtrlPar[5], dist_s,
                                        CtrlPar[12], CtrlPar[13]);
    } else {
      //  in case of only virtual point
      IntscDesEntrInfo[1] = CtrlInfo[0] + 2.0 * dist_s / (vfs + CtrlInfo[5]);
    }
  }
  //  =================================================================== %
  //      time_calc = toc(time_Val);
  RefTrjGnrtr_240503(IntscDesEntrInfo, IntscGrnSlctdInfo, CtrlInfo, CtrlPar,
                     StopInfo, TRAJ, IntscPrvInfo, TimeCnstr, OptSlvPrf);
  *aRef = RefUpdate_230407(TRAJ, CtrlInfo, CtrlPar, StopInfo, vRef, sRef,
                           UpdType, TRAJ_CF);
  //  Long-term speed planning level
  //  TRAJ: 4x1000
  //  from t0 to t0+tHrznLT, where tHrznLT is varying depending the number of
  //  connected intersections and their SPaT sequences, but preview distance
  //  horizon must be less than ConnRange (default 450m)
  for (i = 0; i < 1000; i++) {
    ii = i << 2;
    time_LTtraj[i] = TRAJ[ii];
    acc_LTtraj[i] = TRAJ[ii + 1];
    vel_LTtraj[i] = TRAJ[ii + 2];
    pos_LTtraj[i] = TRAJ[ii + 3];
  }
  //  Short-term collision-avoidance level
  //  TRAJ_CF: 4x100
  //  from t0 to t0+tHrznCF, where tHrznCF is fixed (default 3s)
  for (i = 0; i < 100; i++) {
    ii = i << 2;
    time_STtraj[i] = TRAJ_CF[ii];
    acc_STtraj[i] = TRAJ_CF[ii + 1];
    vel_STtraj[i] = TRAJ_CF[ii + 2];
    pos_STtraj[i] = TRAJ_CF[ii + 3];
  }
  //  UpdType:
  //  -4: constant braking for stops (at very low speeds)
  //  -1: Stops (no LT traj)
  //  -2: Stops (no LT traj)
  //  1: LT free-flow
  //  2: ST car-following
  //  3: constant braking for stops
  //  example
  //  tPrv = t0 + tPrvSpdRcmnd; tLTMax = max(time_LTtraj); tSTMax =
  //  max(time_STtraj); UpdType == -4 or 3    -> SpdRcmnd = v0s +
  //  aRef*tPrvSpdRcmnd; UpdType == 1          -> SpdRcmnd =
  //  interp1(time_LTtraj, vel_LTtraj, min(tPrv, tLTMax)); UpdType == 2 ->
  //  SpdRcmnd = interp1(time_STtraj, vel_STtraj, min(tPrv, tSTMax)); others ->
  //  SpdRcmnd = 0;
}

//
// File trailer for CAV_ctrl_mdl_wTraJ_241219.cpp
//
// [EOF]
//
