#! /usr/bin/env python3
################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

import unittest
import os
from _cwrapper import c_api

class TestAPI(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        ### Autogen Simulink Model Interface
        folder = 'lib'

        if os.name == 'nt':
            libraryname = 'cav_so'
        elif os.name == 'posix':
            libraryname = 'libcav_so'
        else:
            raise ValueError('CAV class cannot determine system type!')
        
        library = os.path.join(folder, libraryname)
            
        self.api = c_api(library)

    def test_runs(self):
        ### Set inputs and run controller
        # Run control for connected vehicle acceleration
        smax = 1000
        vmax = 20

        ### Assign inputs vectors
        # Intersections
        max_n_tls = 5
        IntscInfo = [0]*max_n_tls*6

        for k in range(0, max_n_tls):
            IntscInfo[0 + k*6] = smax
            IntscInfo[1 + k*6] = 10
            IntscInfo[2 + k*6] = 90
            IntscInfo[3 + k*6] = 80
            IntscInfo[4 + k*6] = 5
            IntscInfo[5 + k*6] = 5

        IntscInfo[1 + (max_n_tls-1)*6] = 20 # Force last to be a stop

        # Speed limit
        SpdLimInfo = [0]*4

        SpdLimInfo[0] = 0.
        SpdLimInfo[1] = vmax
        SpdLimInfo[2] = smax
        SpdLimInfo[3] = vmax
        
        # Control inputs
        CtrlInfo = [0]*14

        CtrlInfo[0] = 0.0
        
        CtrlInfo[4] = 0. # s is rear bumper position -> add length to get front bumper
        CtrlInfo[5] = 0.
        CtrlInfo[6] = 0.
        CtrlInfo[1:4] = CtrlInfo[4:7]

        kv = 0
        if kv >= 0:
            CtrlInfo[7] = 100 # sps - ss - d_min0;
            CtrlInfo[8] = 20
            CtrlInfo[9] = 0.

        else:
            CtrlInfo[7] = smax # sps - ss - d_min0;
            CtrlInfo[8] = 0.
            CtrlInfo[9] = 0.
        
        tlind = 0
        if tlind >= 0:
            CtrlInfo[10] = 100 # IntscPos
            CtrlInfo[11] = 10 # IntscType
            CtrlInfo[12] = 2 # TrfLghtState

        else:
            CtrlInfo[10] = smax # IntscPos
            CtrlInfo[11] = 20 # IntscType
            CtrlInfo[12] = 0 # TrfLghtState

        CtrlInfo[13] = vmax # current maximum speed limit
        
        # CAV controller parameter setup
        CtrlPar = [0]*15

        CtrlPar[0] = 450 # ConnRange, m
        CtrlPar[1] = 10 # NxtIntscCntDstMrgn, m
        CtrlPar[2] = 200 # VrtPtDstMrgn, m
        CtrlPar[3] = 1 # dtGrnIniTimeMrgn, s
        CtrlPar[4] = 2 # dtGrnEndTimeMrgn, s
        CtrlPar[5] = vmax*0.95 # SpdDes, m/s
        CtrlPar[6] = 1 # dtMin, s
        CtrlPar[7] = 1 # dtMax, s
        CtrlPar[8] = 1 # CFopt, 0 or 1
        CtrlPar[9] = 3 # tHrznCF, s
        CtrlPar[10] = 1.2 # tau_d, s
        CtrlPar[11] = 3.2 # d_min, m
        CtrlPar[12] = 1.5 # aDes, m/s^2
        CtrlPar[13] = 1.5 # bDes, m/s^2
        CtrlPar[14] = 0 # IsAccAfterDes, 0 or 1

        StopInfo = [1, 3] # % stop mode (Stopped -1, Nearby 0, Moving 1) AND waiting time

        # Run the autogen controller
        acc_des, vel_des, pos_des, exitflag, time_LTtraj, acc_LTtraj, vel_LTtraj, pos_LTtraj, time_STtraj, acc_STtraj, vel_STtraj, pos_STtraj = self.api.step_controller(IntscInfo, SpdLimInfo, CtrlInfo, CtrlPar, StopInfo)
        
        ### Test output
        self.assertEqual(exitflag, 1)

def suite():
    suite = unittest.TestSuite()
    
    suite.addTest( TestAPI('test_runs') )

    return suite

if __name__ == '__main__':
    tests = unittest.TextTestRunner()
    
    tests.run(suite())
    