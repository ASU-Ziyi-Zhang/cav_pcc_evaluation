//
// File: CAV_ctrl_mdl_wTraJ_241219.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef CAV_CTRL_MDL_WTRAJ_241219_H
#define CAV_CTRL_MDL_WTRAJ_241219_H

// Include Files
#include "CAV_ctrl_mdl_wTraJ_241219_spec.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
#ifdef __cplusplus
extern "C" {
#endif

CAV_CTRL_MDL_WTRAJ_241219_DLL_EXPORT extern void CAV_ctrl_mdl_wTraJ_241219(
    const double IntscInfo[30], const double SpdLimInfo[4],
    const double CtrlInfo[14], const double CtrlPar[15],
    const double StopInfo[2], double *aRef, double *vRef, double *sRef,
    double *UpdType, double time_LTtraj[1000], double acc_LTtraj[1000],
    double vel_LTtraj[1000], double pos_LTtraj[1000], double time_STtraj[100],
    double acc_STtraj[100], double vel_STtraj[100], double pos_STtraj[100]);

#ifdef __cplusplus
}
#endif

#endif
//
// File trailer for CAV_ctrl_mdl_wTraJ_241219.h
//
// [EOF]
//
