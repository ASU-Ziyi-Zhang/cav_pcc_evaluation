//
// File: CAV_ctrl_mdl_wTraJ_241219_internal_types.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef CAV_CTRL_MDL_WTRAJ_241219_INTERNAL_TYPES_H
#define CAV_CTRL_MDL_WTRAJ_241219_INTERNAL_TYPES_H

// Include Files
#include "CAV_ctrl_mdl_wTraJ_241219_types.h"
#include "anonymous_function.h"
#include "rtwtypes.h"
#include "coder_bounded_array.h"

// Type Definitions
struct struct_T {
  double penaltyParam;
  double threshold;
  int nPenaltyDecreases;
  double linearizedConstrViol;
  double initFval;
  double initConstrViolationEq;
  double initConstrViolationIneq;
  double phi;
  double phiPrimePlus;
  double phiFullStep;
  double feasRelativeFactor;
  double nlpPrimalFeasError;
  double nlpDualFeasError;
  double nlpComplError;
  double firstOrderOpt;
  boolean_T hasObjective;
};

struct b_struct_T {
  boolean_T gradOK;
  boolean_T fevalOK;
  boolean_T done;
  boolean_T stepAccepted;
  boolean_T failedLineSearch;
  int stepType;
};

struct e_struct_T {
  int ldq;
  coder::bounded_array<double, 2401U, 2U> QR;
  coder::bounded_array<double, 2401U, 2U> Q;
  coder::bounded_array<int, 49U, 1U> jpvt;
  int mrows;
  int ncols;
  coder::bounded_array<double, 49U, 1U> tau;
  int minRowCol;
  boolean_T usedPivoting;
};

struct f_struct_T {
  coder::bounded_array<double, 2401U, 2U> FMat;
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  boolean_T ConvexCheck;
  double regTol_;
  double workspace_;
  double workspace2_;
};

struct g_struct_T {
  coder::bounded_array<double, 25U, 1U> grad;
  coder::bounded_array<double, 24U, 1U> Hx;
  boolean_T hasLinear;
  int nvar;
  int maxVar;
  double beta;
  double rho;
  int objtype;
  int prev_objtype;
  int prev_nvar;
  boolean_T prev_hasLinear;
  double gammaScalar;
};

struct h_struct_T {
  coder::bounded_array<double, 1225U, 2U> workspace_float;
  coder::bounded_array<int, 49U, 1U> workspace_int;
  coder::bounded_array<int, 49U, 1U> workspace_sort;
};

struct i_struct_T {
  int nVarMax;
  int mNonlinIneq;
  int mNonlinEq;
  int mIneq;
  int mEq;
  int iNonIneq0;
  int iNonEq0;
  double sqpFval;
  double sqpFval_old;
  coder::bounded_array<double, 12U, 2U> xstarsqp;
  coder::bounded_array<double, 12U, 2U> xstarsqp_old;
  coder::bounded_array<double, 12U, 1U> cIneq;
  coder::bounded_array<double, 12U, 1U> cIneq_old;
  coder::bounded_array<double, 25U, 1U> grad;
  coder::bounded_array<double, 25U, 1U> grad_old;
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  coder::bounded_array<double, 49U, 1U> lambdasqp;
  coder::bounded_array<double, 49U, 1U> lambdaStopTest;
  coder::bounded_array<double, 49U, 1U> lambdaStopTestPrev;
  double steplength;
  coder::bounded_array<double, 25U, 1U> delta_x;
  coder::bounded_array<double, 25U, 1U> socDirection;
  coder::bounded_array<int, 49U, 1U> workingset_old;
  coder::bounded_array<double, 25U, 1U> gradLag;
  coder::bounded_array<double, 25U, 1U> delta_gradLag;
  coder::bounded_array<double, 25U, 1U> xstar;
  double fstar;
  double firstorderopt;
  coder::bounded_array<double, 49U, 1U> lambda;
  int state;
  double maxConstr;
  int iterations;
  coder::bounded_array<double, 25U, 1U> searchDir;
};

struct j_struct_T {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  coder::bounded_array<double, 300U, 1U> Aineq;
  coder::bounded_array<double, 12U, 1U> bineq;
  coder::empty_bounded_array<double, 1U> Aeq;
  coder::bounded_array<double, 25U, 1U> lb;
  coder::bounded_array<double, 25U, 1U> ub;
  coder::bounded_array<int, 25U, 1U> indexLB;
  coder::bounded_array<int, 25U, 1U> indexUB;
  coder::bounded_array<int, 25U, 1U> indexFixed;
  int mEqRemoved;
  coder::bounded_array<double, 1225U, 1U> ATwset;
  coder::bounded_array<double, 49U, 1U> bwset;
  int nActiveConstr;
  coder::bounded_array<double, 49U, 1U> maxConstrWorkspace;
  int sizes[5];
  int sizesNormal[5];
  int sizesPhaseOne[5];
  int sizesRegularized[5];
  int sizesRegPhaseOne[5];
  int isActiveIdx[6];
  int isActiveIdxNormal[6];
  int isActiveIdxPhaseOne[6];
  int isActiveIdxRegularized[6];
  int isActiveIdxRegPhaseOne[6];
  coder::bounded_array<boolean_T, 49U, 1U> isActiveConstr;
  coder::bounded_array<int, 49U, 1U> Wid;
  coder::bounded_array<int, 49U, 1U> Wlocalidx;
  int nWConstr[5];
  int probType;
  double SLACK0;
};

struct k_struct_T {
  int MaxFunctionEvaluations;
};

struct l_struct_T {
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double ObjectiveLimit;
};

struct m_struct_T {
  coder::anonymous_function objfun;
  double f_1;
  double f_2;
  int nVar;
  int mIneq;
  int mEq;
  int numEvals;
  boolean_T SpecifyObjectiveGradient;
  boolean_T SpecifyConstraintGradient;
  boolean_T isEmptyNonlcon;
  coder::bounded_array<boolean_T, 12U, 1U> hasLB;
  coder::bounded_array<boolean_T, 12U, 1U> hasUB;
  boolean_T hasBounds;
  int FiniteDifferenceType;
};

#endif
//
// File trailer for CAV_ctrl_mdl_wTraJ_241219_internal_types.h
//
// [EOF]
//
