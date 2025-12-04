//
// File: RefTrjGnrtr_240503.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 08-Jan-2025 15:08:19
//

#ifndef REFTRJGNRTR_240503_H
#define REFTRJGNRTR_240503_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
double OptEntTimeSearch_v2_anonFcn1(const double si_arr_data[],
                                    const int si_arr_size[2], double t0s,
                                    double tf0s, double vfs, double v0s,
                                    const double ViolInfo_idxTLStp_data[],
                                    const int ViolInfo_idxTLStp_size[2],
                                    const double ViolInfo_tint_data[],
                                    const int ViolInfo_tint_size[2],
                                    const double x_data[], const int x_size[2]);

double RefTrjGnrtr_240503(const double IntscDesEntrInfo[48],
                          const double IntscGrnSlctdInfo[50],
                          const double CtrlInfo[14], const double CtrlPar[15],
                          const double StopInfo[2], double TRAJ[4000],
                          double Xis[60], double TimeCnstr[80],
                          double OptSlvPrf[2]);

#endif
//
// File trailer for RefTrjGnrtr_240503.h
//
// [EOF]
//
