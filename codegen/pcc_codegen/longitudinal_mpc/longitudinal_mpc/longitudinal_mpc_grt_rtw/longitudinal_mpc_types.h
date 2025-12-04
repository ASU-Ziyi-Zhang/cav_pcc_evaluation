/*
 * longitudinal_mpc_types.h
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

#ifndef longitudinal_mpc_types_h_
#define longitudinal_mpc_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_flags_
#define DEFINED_TYPEDEF_FOR_flags_

enum class flags
  : int32_T {
  SOLVED = 1,                          /* Default value */
  MAX_ITER = 0,
  PRIMAL_INFEASIBLE = -2,
  DUAL_INFEASIBLE = -3,
  NONCONVEX_DETECTED = -6,
  MI_SOLVED = 11,
  MI_PRIMAL_INFEASIBLE = 13,
  MI_DUAL_INFEASIBLE = 15,
  MI_MAX_ITER = 12,
  MI_MAX_ITER_UNSOLVED = 14,
  MI_INTEGER_INFEASIBLE = 17,
  UNSOLVED = -1,
  EMPTY = -1235,
  UNKNOWN = -1234
};

#endif

#ifndef struct_sJvHSAPlL1SbbU0gnSE72ZG_longi_T
#define struct_sJvHSAPlL1SbbU0gnSE72ZG_longi_T

struct sJvHSAPlL1SbbU0gnSE72ZG_longi_T
{
  real_T xstar[37];
  real_T fstar;
  real_T firstorderopt;
  real_T lambda[333];
  int32_T state;
  real_T maxConstr;
  int32_T iterations;
  real_T searchDir[37];
};

#endif                              /* struct_sJvHSAPlL1SbbU0gnSE72ZG_longi_T */

#ifndef struct_scJprC0tZnwNUG3KoXsnHVD_longi_T
#define struct_scJprC0tZnwNUG3KoXsnHVD_longi_T

struct scJprC0tZnwNUG3KoXsnHVD_longi_T
{
  real_T grad[37];
  real_T Hx[36];
  boolean_T hasLinear;
  int32_T nvar;
  int32_T maxVar;
  real_T beta;
  real_T rho;
  int32_T objtype;
  int32_T prev_objtype;
  int32_T prev_nvar;
  boolean_T prev_hasLinear;
  real_T gammaScalar;
};

#endif                              /* struct_scJprC0tZnwNUG3KoXsnHVD_longi_T */

#ifndef struct_skczSSN0IseekIuhVW4znFG_longi_T
#define struct_skczSSN0IseekIuhVW4znFG_longi_T

struct skczSSN0IseekIuhVW4znFG_longi_T
{
  real_T FMat[1369];
  int32_T ldm;
  int32_T ndims;
  int32_T info;
  real_T scaleFactor;
  boolean_T ConvexCheck;
  real_T regTol_;
  real_T workspace_[1776];
  real_T workspace2_[1776];
};

#endif                              /* struct_skczSSN0IseekIuhVW4znFG_longi_T */

#ifndef struct_sL9bDKomAYkxZSVrG9w6En_longit_T
#define struct_sL9bDKomAYkxZSVrG9w6En_longit_T

struct sL9bDKomAYkxZSVrG9w6En_longit_T
{
  int32_T MaxIterations;
  real_T ConstrRelTolFactor;
  real_T ProbRelTolFactor;
  boolean_T RemainFeasible;
};

#endif                              /* struct_sL9bDKomAYkxZSVrG9w6En_longit_T */

#ifndef struct_sCCE9T0P8IQkk6PxUdCnbBE_longi_T
#define struct_sCCE9T0P8IQkk6PxUdCnbBE_longi_T

struct sCCE9T0P8IQkk6PxUdCnbBE_longi_T
{
  int32_T ldq;
  real_T QR[12321];
  real_T Q[1369];
  int32_T jpvt[333];
  int32_T mrows;
  int32_T ncols;
  real_T tau[37];
  int32_T minRowCol;
  boolean_T usedPivoting;
};

#endif                              /* struct_sCCE9T0P8IQkk6PxUdCnbBE_longi_T */

#ifndef struct_sK6ng1KsrjtpGD3SgQmbb8_longit_T
#define struct_sK6ng1KsrjtpGD3SgQmbb8_longit_T

struct sK6ng1KsrjtpGD3SgQmbb8_longit_T
{
  real_T workspace_float[12321];
  int32_T workspace_int[333];
  int32_T workspace_sort[333];
};

#endif                              /* struct_sK6ng1KsrjtpGD3SgQmbb8_longit_T */

#ifndef struct_sCwhQn7ZFnEO6qmUyij3F5_longit_T
#define struct_sCwhQn7ZFnEO6qmUyij3F5_longit_T

struct sCwhQn7ZFnEO6qmUyij3F5_longit_T
{
  real_T ineqlin[332];
  real_T lower[36];
  real_T upper[36];
};

#endif                              /* struct_sCwhQn7ZFnEO6qmUyij3F5_longit_T */

#ifndef struct_s1rlrl8wYvAiB01zuzfeYY_longit_T
#define struct_s1rlrl8wYvAiB01zuzfeYY_longit_T

struct s1rlrl8wYvAiB01zuzfeYY_longit_T
{
  int32_T mConstr;
  int32_T mConstrOrig;
  int32_T mConstrMax;
  int32_T nVar;
  int32_T nVarOrig;
  int32_T nVarMax;
  int32_T ldA;
  real_T Aineq[12284];
  real_T bineq[332];
  real_T lb[37];
  real_T ub[37];
  int32_T indexLB[37];
  int32_T indexUB[37];
  int32_T indexFixed[37];
  int32_T mEqRemoved;
  real_T ATwset[12321];
  real_T bwset[333];
  int32_T nActiveConstr;
  real_T maxConstrWorkspace[333];
  int32_T sizes[5];
  int32_T sizesNormal[5];
  int32_T sizesPhaseOne[5];
  int32_T sizesRegularized[5];
  int32_T sizesRegPhaseOne[5];
  int32_T isActiveIdx[6];
  int32_T isActiveIdxNormal[6];
  int32_T isActiveIdxPhaseOne[6];
  int32_T isActiveIdxRegularized[6];
  int32_T isActiveIdxRegPhaseOne[6];
  boolean_T isActiveConstr[333];
  int32_T Wid[333];
  int32_T Wlocalidx[333];
  int32_T nWConstr[5];
  int32_T probType;
  real_T SLACK0;
};

#endif                              /* struct_s1rlrl8wYvAiB01zuzfeYY_longit_T */

#ifndef struct_s7GW9uShiIXbHYZwohNmyqD_longi_T
#define struct_s7GW9uShiIXbHYZwohNmyqD_longi_T

struct s7GW9uShiIXbHYZwohNmyqD_longi_T
{
  boolean_T NonFiniteSupport;
  boolean_T IterDisplaySQP;
  real_T InitDamping;
  char_T FiniteDifferenceType[7];
  boolean_T SpecifyObjectiveGradient;
  boolean_T ScaleProblem;
  boolean_T SpecifyConstraintGradient;
  real_T FiniteDifferenceStepSize;
  real_T MaxFunctionEvaluations;
  boolean_T IterDisplayQP;
  real_T PricingTolerance;
  char_T Algorithm[10];
  real_T ObjectiveLimit;
  real_T ConstraintTolerance;
  real_T OptimalityTolerance;
  real_T StepTolerance;
  real_T MaxIterations;
  real_T FunctionTolerance;
  char_T SolverName[8];
  boolean_T CheckGradients;
  char_T Diagnostics[3];
  real_T DiffMaxChange;
  real_T DiffMinChange;
  char_T Display[3];
  char_T FunValCheck[3];
  boolean_T UseParallel;
  char_T LinearSolver[4];
  char_T SubproblemAlgorithm[2];
};

#endif                              /* struct_s7GW9uShiIXbHYZwohNmyqD_longi_T */

#ifndef struct_solver_longitudinal_mpc_T
#define struct_solver_longitudinal_mpc_T

struct solver_longitudinal_mpc_T
{
  int32_T isInitialized;
  real_T vel_max;
  real_T pos_max;
  real_T S[9];
  real_T Phi[288];
  real_T Gamma[3072];
  real_T GammaC[9216];
  real_T GammaW[3072];
  real_T Omega[9216];
  real_T Psi[1024];
  real_T G[1024];
  real_T F[96];
  real_T Fw[1024];
  real_T Ty[3072];
  real_T Tu[1024];
  real_T bi[10];
  real_T bn[8];
  real_T D[984];
  real_T M[31488];
  real_T Sigma[10496];
  real_T W[984];
  real_T b[328];
  real_T a[10496];
  real_T s_chance[33];
  real_T v_chance[33];
  real_T s_chance_vary[328];
  real_T v_chance_vary[328];
  real_T Upsilon[1312];
  real_T UpsilonI[16];
  real_T Upsilonb[4];
  real_T upsn[32];
  real_T betabn[8];
  real_T beta_rhs[328];
  real_T u;
  real_T X[96];
  real_T U[32];
  real_T E[4];
  real_T Ref[3];
  real_T Con;
  real_T Xopt[36];
  real_T Xk[36];
  real_T J;
  flags flag;
};

#endif                                 /* struct_solver_longitudinal_mpc_T */

/* Forward declaration for rtModel */
typedef struct tag_RTM_longitudinal_mpc_T RT_MODEL_longitudinal_mpc_T;

#endif                                 /* longitudinal_mpc_types_h_ */
