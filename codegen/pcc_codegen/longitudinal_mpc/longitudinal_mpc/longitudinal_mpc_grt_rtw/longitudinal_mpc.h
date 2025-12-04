/*
 * longitudinal_mpc.h
 *
 * Code generation for model "longitudinal_mpc".
 *
 * Model version              : 1.201
 * Simulink Coder version : 24.1 (R2024a) 19-Nov-2023
 * C++ source code generated on : Mon Aug 19 14:47:00 2024
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Execution efficiency
 * Validation result: Passed (8), Warning (1), Error (0)
 */

#ifndef longitudinal_mpc_h_
#define longitudinal_mpc_h_
#include <cmath>
#include <cstdio>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "longitudinal_mpc_types.h"
#include "coder_array.h"

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
struct B_longitudinal_mpc_T {
  real_T GammaP[40000];
  real_T Cov[40000];
  real_T K[29760];
  s1rlrl8wYvAiB01zuzfeYY_longit_T b_WorkingSet;
  s1rlrl8wYvAiB01zuzfeYY_longit_T b_WorkingSet_m;
  s1rlrl8wYvAiB01zuzfeYY_longit_T b_workingset;
  s1rlrl8wYvAiB01zuzfeYY_longit_T c_workingset;
  s1rlrl8wYvAiB01zuzfeYY_longit_T obj;
  s1rlrl8wYvAiB01zuzfeYY_longit_T workingset;
  s1rlrl8wYvAiB01zuzfeYY_longit_T workingset_c;
  s1rlrl8wYvAiB01zuzfeYY_longit_T workingset_k;
  real_T Gamma[20000];
  real_T Mean[20000];
  real_T b[20000];
  sCCE9T0P8IQkk6PxUdCnbBE_longi_T QRManager;
  sCCE9T0P8IQkk6PxUdCnbBE_longi_T qrmanager;
  sCCE9T0P8IQkk6PxUdCnbBE_longi_T qrmanager_c;
  sK6ng1KsrjtpGD3SgQmbb8_longit_T b_memspace;
  real_T b_memspace_b[12321];
  real_T B[12321];
  real_T dv[11952];
  real_T GammaP_p[9216];
  int16_T tmp_data[20000];
  int16_T tmp_data_c[20000];
  skczSSN0IseekIuhVW4znFG_longi_T CholRegManager;
  real_T a[3072];
  real_T b_f[3072];
  real_T b_g[3072];
  real_T b_g1[3072];
  real_T H[1296];
  int8_T K_m[10240];
  real_T H_n[1024];
  real_T b_this[1024];
  real_T e[1000];
  real_T K_p[960];
  real_T pp_coefs[960];
};

/* Block states (default storage) for system '<Root>' */
struct DW_longitudinal_mpc_T {
  solver_longitudinal_mpc_T obj;       /* '<S1>/MPC System' */
  real_T Xopt[36];                     /* '<S1>/MPC System' */
  real_T Xk[36];                       /* '<S1>/MPC System' */
  real_T J;                            /* '<S1>/MPC System' */
  flags flag;                          /* '<S1>/MPC System' */
  boolean_T objisempty;                /* '<S1>/MPC System' */
};

/* External inputs (root inport signals with default storage) */
struct ExtU_longitudinal_mpc_T {
  real_T t;                            /* '<Root>/t' */
  real_T ego_state[3];                 /* '<Root>/ego_state' */
  real_T pos_pred[201];                /* '<Root>/pos_pred' */
  real_T vel_pred[201];                /* '<Root>/vel_pred' */
  real_T acc_pred[201];                /* '<Root>/acc_pred' */
  real_T time_pred[201];               /* '<Root>/time_pred' */
  real_T pos_max;                      /* '<Root>/pos_max' */
  real_T vel_max;                      /* '<Root>/vel_max' */
};

/* External outputs (root outports fed by signals with default storage) */
struct ExtY_longitudinal_mpc_T {
  real_T acc_des;                      /* '<Root>/acc_des' */
  real_T state_trajectory[96];         /* '<Root>/state_trajectory' */
  real_T control_trajectory[32];       /* '<Root>/control_trajectory' */
  real_T time_trajectory[32];          /* '<Root>/time_trajectory' */
  real_T slacks[4];                    /* '<Root>/slacks' */
  real_T reference[3];                 /* '<Root>/reference' */
  real_T constraint;                   /* '<Root>/constraint' */
  real_T cost;                         /* '<Root>/cost' */
  flags exitflag;                      /* '<Root>/exitflag' */
};

/* Real-time Model Data Structure */
struct tag_RTM_longitudinal_mpc_T {
  const char_T *errorStatus;
};

/* Class declaration for model longitudinal_mpc */
class longitudinal_mpc final
{
  /* public data and function members */
 public:
  /* Copy Constructor */
  longitudinal_mpc(longitudinal_mpc const&) = delete;

  /* Assignment Operator */
  longitudinal_mpc& operator= (longitudinal_mpc const&) & = delete;

  /* Move Constructor */
  longitudinal_mpc(longitudinal_mpc &&) = delete;

  /* Move Assignment Operator */
  longitudinal_mpc& operator= (longitudinal_mpc &&) = delete;

  /* Real-Time Model get method */
  RT_MODEL_longitudinal_mpc_T * getRTM();

  /* Root inports set method */
  void setExternalInputs(const ExtU_longitudinal_mpc_T *pExtU_longitudinal_mpc_T)
  {
    longitudinal_mpc_U = *pExtU_longitudinal_mpc_T;
  }

  /* Root outports get method */
  const ExtY_longitudinal_mpc_T &getExternalOutputs() const
  {
    return longitudinal_mpc_Y;
  }

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  longitudinal_mpc();

  /* Destructor */
  ~longitudinal_mpc();

  /* private data and function members */
 private:
  /* External inputs */
  ExtU_longitudinal_mpc_T longitudinal_mpc_U;

  /* External outputs */
  ExtY_longitudinal_mpc_T longitudinal_mpc_Y;

  /* Block signals */
  B_longitudinal_mpc_T longitudinal_mpc_B;

  /* Block states */
  DW_longitudinal_mpc_T longitudinal_mpc_DW;

  /* private member function(s) for subsystem '<Root>'*/
  real_T longitudinal_mpc_norm(const real_T x[16]);
  void longitudinal_mpc_mpower(const real_T a[16], real_T b, real_T c[16]);
  real_T longitudinal_mpc_log2(real_T x);
  void longitudinal_mpc_norminv(const coder::array<real_T, 1U> &mu, const coder::
    array<real_T, 1U> &sigma, real_T x[100]);
  void longitudinal_m_PCCMPC_getChance(real_T s_p[100], real_T v_p[100]);
  void longitudinal_mpc_interp1(const real_T varargin_2[100], real_T Vq[33]);
  void longitudinal_mpc_PCCMPC_initMPC(solver_longitudinal_mpc_T *b_this);
  solver_longitudinal_mpc_T *longitudinal_mpc_solver_solver
    (solver_longitudinal_mpc_T *b_this);
  void longitudinal__solver_initSolver(solver_longitudinal_mpc_T *b_this);
  void longitudinal_mpc_interp1_h(const real_T varargin_1[201], const real_T
    varargin_2[201], const real_T varargin_3[81], real_T Vq[81]);
  boolean_T longitudinal_mpc_vectorAny(const boolean_T x_data[], const int32_T
    x_size[1]);
  void longitudinal_mpc_pchip(const real_T x[81], const real_T y[81], real_T
    v_breaks[81], real_T v_coefs[320]);
  real_T longitudinal_mpc_ppval(const real_T pp_breaks[81], const real_T
    pp_coefs[320], real_T x);
  void longitudinal_mpc_interp1_b(const real_T varargin_1[81], const real_T
    varargin_2[243], const real_T varargin_3[32], real_T Vq[96]);
  void longitudinal_m_factoryConstruct(skczSSN0IseekIuhVW4znFG_longi_T *obj);
  real_T longitudinal_mpc_xnrm2(int32_T n, const real_T x[12321], int32_T ix0);
  void longitudinal_mpc_xzlarfg(int32_T n, real_T alpha1, real_T x[12321],
    int32_T ix0, real_T *b_alpha1, real_T *tau);
  void longitudinal_mpc_xzlarf(int32_T m, int32_T n, int32_T iv0, real_T tau,
    real_T C[12321], int32_T ic0, real_T work[333]);
  void longitudinal_mpc_qrf(real_T A[12321], int32_T m, int32_T n, int32_T nfxd,
    real_T tau[37]);
  void longitudinal_mpc_factorQRE(const sCCE9T0P8IQkk6PxUdCnbBE_longi_T *obj,
    int32_T mrows, int32_T ncols, sCCE9T0P8IQkk6PxUdCnbBE_longi_T *b_obj);
  void longitudinal_mpc_countsort(int32_T x[333], int32_T xLen, int32_T
    workspace[333], int32_T xMin, int32_T xMax);
  void longitudinal_mpc_removeConstr(s1rlrl8wYvAiB01zuzfeYY_longit_T *obj,
    int32_T idx_global);
  void longitudin_RemoveDependentIneq_(s1rlrl8wYvAiB01zuzfeYY_longit_T
    *workingset, sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager,
    sK6ng1KsrjtpGD3SgQmbb8_longit_T *memspace, real_T tolfactor);
  void longitudinal_mpc_factorQR(sCCE9T0P8IQkk6PxUdCnbBE_longi_T *obj, const
    real_T A[12321], int32_T mrows, int32_T ncols);
  void longitudinal_mpc_computeQ_(sCCE9T0P8IQkk6PxUdCnbBE_longi_T *obj, int32_T
    nrows);
  void longitudinal_mpc_xgemv(int32_T m, const real_T A[12284], const real_T x
    [12321], real_T y[333]);
  void longitud_maxConstraintViolation(const s1rlrl8wYvAiB01zuzfeYY_longit_T
    *obj, const real_T x[12321], real_T *v, s1rlrl8wYvAiB01zuzfeYY_longit_T
    *b_obj);
  void longitudinal_mpc_xgemv_b(int32_T m, const real_T A[12284], const real_T
    x[12321], real_T y[333]);
  void longit_maxConstraintViolation_b(const s1rlrl8wYvAiB01zuzfeYY_longit_T
    *obj, const real_T x[12321], real_T *v, s1rlrl8wYvAiB01zuzfeYY_longit_T
    *b_obj);
  boolean_T longitu_feasibleX0ForWorkingSet(real_T workspace[12321], real_T
    xCurrent[37], s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset,
    sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager);
  void longitudi_PresolveWorkingSet_b4(const sJvHSAPlL1SbbU0gnSE72ZG_longi_T
    *solution, const real_T memspace_workspace_float[12321], const int32_T
    memspace_workspace_int[333], const int32_T memspace_workspace_sort[333],
    s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset, sJvHSAPlL1SbbU0gnSE72ZG_longi_T
    *b_solution, sK6ng1KsrjtpGD3SgQmbb8_longit_T *b_memspace,
    sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager);
  void longitudinal_mpc_xgemv_b4(int32_T m, const real_T A[12284], const real_T
    x[37], real_T y[333]);
  void longi_maxConstraintViolation_b4(const s1rlrl8wYvAiB01zuzfeYY_longit_T
    *obj, const real_T x[37], real_T *v, s1rlrl8wYvAiB01zuzfeYY_longit_T *b_obj);
  void longitu_modifyOverheadPhaseOne_(const s1rlrl8wYvAiB01zuzfeYY_longit_T
    *obj, s1rlrl8wYvAiB01zuzfeYY_longit_T *b_obj);
  void longitudinal_mpc_setProblemType(s1rlrl8wYvAiB01zuzfeYY_longit_T *obj,
    int32_T PROBLEM_TYPE);
  void longitudinal_mpc_linearForm_(boolean_T obj_hasLinear, int32_T obj_nvar,
    real_T workspace[12321], const real_T H[1296], const real_T f[36], const
    real_T x[37]);
  real_T longitudinal_mpc_computeFval(const scJprC0tZnwNUG3KoXsnHVD_longi_T *obj,
    real_T workspace[12321], const real_T H[1296], const real_T f[36], const
    real_T x[37]);
  void longitudinal_mpc_xgemv_b4t(int32_T m, int32_T n, const real_T A[1296],
    int32_T lda, const real_T x[37], real_T y[36]);
  void longitudina_computeGrad_StoreHx(scJprC0tZnwNUG3KoXsnHVD_longi_T *obj,
    const real_T H[1296], const real_T f[36], const real_T x[37]);
  real_T longitudina_computeFval_ReuseHx(const scJprC0tZnwNUG3KoXsnHVD_longi_T
    *obj, real_T workspace[12321], const real_T f[36], const real_T x[37]);
  void longitudinal_mpc_xrotg(real_T a, real_T b, real_T *b_a, real_T *b_b,
    real_T *c, real_T *s);
  void longitudinal_m_deleteColMoveEnd(sCCE9T0P8IQkk6PxUdCnbBE_longi_T *obj,
    int32_T idx);
  void longitudinal_mpc_fullColLDL2_(skczSSN0IseekIuhVW4znFG_longi_T *obj,
    int32_T NColsRemain, real_T REG_PRIMAL);
  void longitudinal_mpc_xgemv_b4tv(int32_T m, int32_T n, const real_T A[1369],
    int32_T ia0, const real_T x[12321], real_T y[37]);
  void longitudinal_mpc_compute_deltax(const real_T H[1296],
    sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution, sK6ng1KsrjtpGD3SgQmbb8_longit_T
    *memspace, const sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager,
    skczSSN0IseekIuhVW4znFG_longi_T *cholmanager, const
    scJprC0tZnwNUG3KoXsnHVD_longi_T *objective);
  real_T longitudinal_mpc_xnrm2_b(int32_T n, const real_T x[37]);
  void longitudinal_mpc_xgemv_b4tv3(int32_T m, const real_T A[12284], const
    real_T x[37], real_T y[12321]);
  void longitudinal_mpc_ratiotest(const real_T solution_xstar[37], const real_T
    solution_searchDir[37], real_T workspace[12321], int32_T workingset_nVar,
    const real_T workingset_Aineq[12284], const real_T workingset_bineq[332],
    const int32_T workingset_indexLB[37], const int32_T workingset_sizes[5],
    const int32_T workingset_isActiveIdx[6], const boolean_T
    workingset_isActiveConstr[333], const int32_T workingset_nWConstr[5], real_T
    toldelta, real_T *alpha, boolean_T *newBlocking, int32_T *constrType,
    int32_T *constrIdx, real_T *b_toldelta);
  void longitudinal__feasibleratiotest(const real_T solution_xstar[37], const
    real_T solution_searchDir[37], real_T workspace[12321], int32_T
    workingset_nVar, const real_T workingset_Aineq[12284], const real_T
    workingset_bineq[332], const int32_T workingset_indexLB[37], const int32_T
    workingset_sizes[5], const int32_T workingset_isActiveIdx[6], const
    boolean_T workingset_isActiveConstr[333], const int32_T workingset_nWConstr
    [5], boolean_T isPhaseOne, real_T *alpha, boolean_T *newBlocking, int32_T
    *constrType, int32_T *constrIdx);
  void longit_checkUnboundedOrIllPosed(sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution,
    const scJprC0tZnwNUG3KoXsnHVD_longi_T *objective);
  void long_addBoundToActiveSetMatrix_(const s1rlrl8wYvAiB01zuzfeYY_longit_T
    *obj, int32_T TYPE, int32_T idx_local, s1rlrl8wYvAiB01zuzfeYY_longit_T
    *b_obj);
  void lo_checkStoppingAndUpdateFval_b(int32_T activeSetChangeID, const real_T
    f[36], sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution,
    sK6ng1KsrjtpGD3SgQmbb8_longit_T *memspace, const
    scJprC0tZnwNUG3KoXsnHVD_longi_T *objective, s1rlrl8wYvAiB01zuzfeYY_longit_T *
    workingset, sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager, real_T
    options_ObjectiveLimit, real_T runTimeOptions_ConstrRelTolFact, boolean_T
    updateFval, int32_T *b_activeSetChangeID, boolean_T *b_updateFval);
  void longitudinal_mpc_iterate_b(const real_T H[1296], const real_T f[36],
    sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution, sK6ng1KsrjtpGD3SgQmbb8_longit_T
    *memspace, s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset,
    sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager, skczSSN0IseekIuhVW4znFG_longi_T *
    cholmanager, scJprC0tZnwNUG3KoXsnHVD_longi_T *objective, real_T
    options_ObjectiveLimit, real_T options_StepTolerance, real_T
    runTimeOptions_ConstrRelTolFact, real_T runTimeOptions_ProbRelTolFactor,
    boolean_T runTimeOptions_RemainFeasible);
  void longitudinal_mpc_phaseone_b4(const real_T H[1296], const real_T f[36],
    sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution, sK6ng1KsrjtpGD3SgQmbb8_longit_T
    *memspace, s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset,
    sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager, skczSSN0IseekIuhVW4znFG_longi_T *
    cholmanager, const sL9bDKomAYkxZSVrG9w6En_longit_T *runTimeOptions,
    scJprC0tZnwNUG3KoXsnHVD_longi_T *objective, s7GW9uShiIXbHYZwohNmyqD_longi_T *
    options);
  int32_T longitudinal_RemoveDependentEq_(sK6ng1KsrjtpGD3SgQmbb8_longit_T
    *memspace, const s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset,
    sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager);
  void longitud_PresolveWorkingSet_b4t(const sJvHSAPlL1SbbU0gnSE72ZG_longi_T
    *solution, sK6ng1KsrjtpGD3SgQmbb8_longit_T *memspace,
    s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset, sCCE9T0P8IQkk6PxUdCnbBE_longi_T
    *qrmanager, sJvHSAPlL1SbbU0gnSE72ZG_longi_T *b_solution);
  boolean_T longitudinal_mpc_strcmp(const char_T a[8]);
  void longitudin_computeFirstOrderOpt(sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution,
    const scJprC0tZnwNUG3KoXsnHVD_longi_T *objective, const
    s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset, real_T workspace[12321]);
  void longitudinal_mpc_phaseone_b4t(const real_T H[1296], const real_T f[36],
    sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution, sK6ng1KsrjtpGD3SgQmbb8_longit_T
    *memspace, const s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset,
    sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager, skczSSN0IseekIuhVW4znFG_longi_T *
    cholmanager, scJprC0tZnwNUG3KoXsnHVD_longi_T *objective,
    s7GW9uShiIXbHYZwohNmyqD_longi_T *options, const
    sL9bDKomAYkxZSVrG9w6En_longit_T *runTimeOptions,
    s1rlrl8wYvAiB01zuzfeYY_longit_T *b_workingset);
  void longitudinal_mpc_driver(const real_T H[1296], const real_T f[36],
    sJvHSAPlL1SbbU0gnSE72ZG_longi_T *solution, const real_T
    memspace_workspace_float[12321], const int32_T memspace_workspace_int[333],
    const int32_T memspace_workspace_sort[333], const
    s1rlrl8wYvAiB01zuzfeYY_longit_T *workingset, skczSSN0IseekIuhVW4znFG_longi_T
    *cholmanager, sL9bDKomAYkxZSVrG9w6En_longit_T runTimeOptions,
    sK6ng1KsrjtpGD3SgQmbb8_longit_T *b_memspace, s1rlrl8wYvAiB01zuzfeYY_longit_T
    *b_workingset, sCCE9T0P8IQkk6PxUdCnbBE_longi_T *qrmanager,
    scJprC0tZnwNUG3KoXsnHVD_longi_T *objective);
  void longitudinal_mpc_linearForm__b(boolean_T obj_hasLinear, int32_T obj_nvar,
    real_T workspace[37], const real_T H[1296], const real_T f[36], const real_T
    x[37]);
  void longitudinal_mpc_quadprog(const real_T H[1296], const real_T f[36], const
    real_T Aineq[11952], const real_T bineq[332], const real_T x0[36], real_T x
    [36], real_T *fval, real_T *exitflag, char_T output_algorithm[10], real_T
    *output_firstorderopt, real_T *output_constrviolation, real_T
    *output_iterations, sCwhQn7ZFnEO6qmUyij3F5_longit_T *lambda);
  boolean_T longitudinal_mpc_isMember(real_T a);
  flags longitudi_convert_to_enum_flags(int32_T input);

  /* Real-Time Model */
  RT_MODEL_longitudinal_mpc_T longitudinal_mpc_M;
};

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('longitudinal_mpc_sl/longitudinal_mpc')    - opens subsystem longitudinal_mpc_sl/longitudinal_mpc
 * hilite_system('longitudinal_mpc_sl/longitudinal_mpc/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'longitudinal_mpc_sl'
 * '<S1>'   : 'longitudinal_mpc_sl/longitudinal_mpc'
 * '<S2>'   : 'longitudinal_mpc_sl/longitudinal_mpc/getReference'
 */
#endif                                 /* longitudinal_mpc_h_ */
