#! /usr/bin/env python3
################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

import unittest
import os
import sys
                
from _cppwrapper import cpp_api

class TestAPI(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        ### Autogen Simulink Model Interface
        folder = 'lib'

        if os.name == 'nt':
            libraryname = 'pcc_so'
        elif os.name == 'posix':
            libraryname = 'libpcc_so'
        else:
            raise ValueError('PCC class cannot determine system type!')
        
        library = os.path.join(folder, libraryname)
            
        self.api = cpp_api(library)

    def test_runs(self):
        ### Set inputs and run controller
        # Predict PV motion
        dt_pred = 0.10 # [s]
        t_pred = 0. # [s]
        
        k = 0 # First index is current PV states
        self.api.inputs_p.contents.time_pred[k] = t_pred

        for k in range(1, 51): # Future indices are predicted PV states
            # Propagate kinematic constant acceleration prediction
            t_pred += dt_pred
            self.api.inputs_p.contents.time_pred[k] = t_pred

        # Run the autogen controller
        self.api.step_inputs() # Reads inputs_p and writes them to controller
        self.api.step_controller() # Steps the controller
        self.api.step_outputs() # Writes the outputs from controller to outputs_p

        # Assign outputs struct properties
        acc_des = self.api.outputs_p.contents.acc_des
        exitflag = self.api.outputs_p.contents.exitflag

        ### Test output
        self.assertEqual(exitflag, 1)

def suite():
    suite = unittest.TestSuite()
    
    suite.addTest( TestAPI('test_runs') )

    return suite

if __name__ == '__main__':
    tests = unittest.TextTestRunner()
    
    tests.run(suite())
    