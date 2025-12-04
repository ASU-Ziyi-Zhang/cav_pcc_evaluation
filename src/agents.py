################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

from math import fmod, nan, isfinite

import os
import numpy as np

from src.settings import *
from src.pcc_cppwrapper import cpp_api
from src.cav_cwrapper import c_api
from src.sensing import VehicleState
from src.sensing import get_front_tl, get_n_front_tls, get_n_latest_tls, clamp

class _vehicle:
    '''Virtual vehicle super class'''
    used_ids = []

    def update_states(self, latitude, longitude, speed, acceleration, heading, heading_rate_change, times=[], rel_long_pos=[]):
            self.states = VehicleState(latitude, longitude, nan, nan, speed, acceleration, heading, heading_rate_change, times=times, rel_long_pos=rel_long_pos)

    def reset_nv_states(self):
        self.nv_states.clear()

    def update_nv_states(self, id, latitude=0.0, longitude=0.0, rel_long_gap=nan, rel_lat_gap=nan, speed=0.0, acceleration=0.0,
                heading=0.0, heading_rate_change=0.0, times=[], rel_long_pos=[]):
        self.nv_states[id] = VehicleState(latitude, longitude, rel_long_gap, rel_lat_gap, speed, acceleration, heading, heading_rate_change, times=times, rel_long_pos=rel_long_pos)

    def __init__(self, t0 = -1., vd = 20., id = -1):
        ### Default controller properties
        self.vd = vd # Desired speed [m/s]
        self.d = 3.6 # Desired standstill gap distance [m]
        self.T = 1.2 # Desired time headway [s]

        # Default control actions
        self.ua = 0. # Desired acceleration [m/s2]
        self.ul = 0 # Desired lane change direction - left is >0 right is <0 [-]

        # Unique ID - If an ID is assigned (positive-valued) then it must be unique as checked against used_ids list
        if id < 1:
            id = _vehicle.used_ids[-1]+1 if _vehicle.used_ids else 1

        self.id = id

        assert id not in _vehicle.used_ids, 'Non-unique ID attempted to be assigned for a specific vehicle object! ID {:d} already used.'.format(id)  # ID is being assigned for this particular vehicle object - check that it is unique
        _vehicle.used_ids.append(id)

        # Vehicle states
        self.states = VehicleState() # External vehicle states communicated from client
        self.nv_states = {} # Nearby vehicle states to communicate to client    

        # Intersections
        self.is_stopping_light = False # Flag to indicate if ego is braking to stopping at intersection - initialize as False in constructor

        self.stop_dwell_time = 3. # [s] Time to wait at a stop sign before continuing
        self.t_stop = 0. # [s] Last time that a stop sign calculation was computed
        self.dt_stop = 0. # [s] Time before moving through a stop sign - must be <0 before moving through is allowed
        self.is_stopping_sign = False # Flag to indicate if ego is braking to stopping at stop sign - initialize as False in constructor
        self.is_waiting_sign = False # Flag to indicate if ego is waiting at stop sign - initialize as False in constructor
        self.is_moving_sign = False # Flag to indiciate if ego is moving through stop sign - initialize as False in constructor

        self.is_acc_after_dest = 1 # If ego should stop at stop sign and then travel through, or wait permanently

        # Simulation properties
        self.entering_time = t0 # [s] Time when the vehicle is set to enter the simulation
        
        self.is_exists = False # Flag to detect if the vehicle has entered the simulation or not yet
        self.is_exited = False # Flag to detect if the vehicle has safely exited the network

    def stop_sign_approach(self, t, dt, lighttype, lightds):
        # Check if stop sign logic has already occurred for this simulation step
        if t-self.t_stop < dt:
            pass

        # Compute stop sign approach logic
        self.t_stop = t
        self.dt_stop -= dt

        if lighttype == LightType.STOP.value and self.dt_stop < -3.*self.stop_dwell_time and lightds > 0 and lightds < 100: # Stop sign is a new object ego is approaching and we haven't stopped recently for a sign
            self.is_stopping_sign = True
            self.is_moving_sign = False

        if self.is_waiting_sign and self.dt_stop < 0 and self.is_acc_after_dest: # Finished stopping and dwelling for this sign
            self.is_stopping_sign = False
            self.is_waiting_sign = False
            self.is_moving_sign = True

        if self.is_stopping_sign and self.states.speed < 0.1 and lightds < 4:
            if not self.is_waiting_sign: # Have not detected a stop for this sign yet
                self.is_waiting_sign = True # Detected the ego has now stopped
                self.dt_stop = dt + self.stop_dwell_time # Start the counter

class Basic(_vehicle):
    '''Basic adaptive cruise control driver for virtual automated vehicles'''    
    def __init__(self, t0 = -1., vd = 20., id = -1, active=False):
        super().__init__(t0, vd, id)

        self.type = ControlType.BASIC.value

        # Parameters
        self.vd = vd # Desired velocity [m/s]
        self.active = active # True/false to allow lane changes to activate
        
        # Controls
        self.ua = 0. # Acceleration command
        self.ul = 0 # Lane command - which lane change direction to make

    def getCommand(self, ego_speed, ego_accel, lead_distance, lead_speed, lead_accel):
        desired_time_gap = 1.5
        min_gap = 2.0
        max_accel = 2.0
        max_decel = 4.5

        desired_gap = max(min_gap, ego_speed*desired_time_gap)
        gap_diff = lead_distance - desired_gap
        relative_speed = ego_speed - lead_speed

        if gap_diff > 0:
            desired_acceleration = min(max_accel, gap_diff - relative_speed)

        else:
            desired_acceleration = max(max_decel, gap_diff - relative_speed)

        return desired_acceleration

class PCC(_vehicle):
    '''PCC driver for virtual automated vehicles'''
    def __init__(self, t0 = -1., vd = 20., id = -1, active=False):
        super().__init__(t0, vd, id)

        self.type = ControlType.PCC.value

        # Parameters
        self.vd = vd # Desired velocity [m/s]
        self.active = active # True/false to allow lane changes to activate
        
        # Controls
        self.ua = 0. # Acceleration command
        self.ul = 0 # Lane change command

        # Autogen Simulink Model Interface
        if os.name == 'nt':
            libraryname = 'pcc_so'
        
        elif os.name == 'posix':
            libraryname = 'libpcc_so'
        
        elif os.name == 'darwin':
            libraryname == 'libpcc_so'

        else:
            raise ValueError('PCC class cannot determine system type!')
        
        self.api = cpp_api(libraryname)

    def resetPred(self):
        reset_val = nan

        for k in range(0,201): # Predicted PV states
            self.api.inputs_p.contents.acc_pred[k] = reset_val
            self.api.inputs_p.contents.vel_pred[k] = reset_val
            self.api.inputs_p.contents.pos_pred[k] = reset_val
            self.api.inputs_p.contents.time_pred[k] = reset_val

    def predAcc(self, t, lead_accel, lead_speed, lead_rel_distance, ego_distance, v_max):
        '''Run a constant acceleration prediction from the current measurements of the preceding vehicle'''
        # Current measurements
        dt_pred = 0.2 # Prediction stage step size
        t_pred = t

        k = 0 # Current PV state measurements
        self.api.inputs_p.contents.acc_pred[k] = lead_accel
        self.api.inputs_p.contents.vel_pred[k] = lead_speed
        self.api.inputs_p.contents.pos_pred[k] = ego_distance + lead_rel_distance
        self.api.inputs_p.contents.time_pred[k] = t

        for k in range(1,51): # Predicted PV states
            if lead_speed > v_max:
                lead_speed = v_max
                lead_accel = 0.

            elif lead_speed < 0:
                lead_accel = 0.
                lead_speed = 0.

            # Saturate constant acceleration assumption to only activite for first n seconds
            if t_pred-t > 3.0:
                lead_accel = 0.

            # Propagate kinematic constant acceleration prediction
            t_pred += dt_pred

            self.api.inputs_p.contents.acc_pred[k] = lead_accel
            self.api.inputs_p.contents.vel_pred[k] = max(lead_speed + dt_pred*lead_accel, 0)
            self.api.inputs_p.contents.pos_pred[k] = ego_distance + lead_rel_distance + dt_pred*(max(lead_speed + dt_pred*lead_accel, 0) + dt_pred*lead_accel)
            self.api.inputs_p.contents.time_pred[k] = t_pred

            lead_rel_distance += dt_pred*(max(lead_speed + dt_pred*lead_accel, 0) + dt_pred*lead_accel)
            lead_speed = self.api.inputs_p.contents.vel_pred[k]

    def useComms(self, t, lead_accel, lead_speed, lead_rel_distance, ego_distance, lead_t_comms, lead_s_comms, lead_v_comms=[]):
        '''Use the current communication from the preceding vehicle as prediction'''
        ### Check inputs
        n_pred_steps = len(lead_t_comms)

        assert n_pred_steps == len(lead_s_comms), 'Communicated time and spacing comms are mismatched in length!'

        # No velocity provided so provide a basic estimate
        if len(lead_v_comms) < n_pred_steps:
            lead_v_comms = np.empty_like(lead_s_comms)

            lead_v_comms[0] = lead_speed
            for i in range(1, n_pred_steps):
                lead_v_comms[i] = (lead_s_comms[i]-lead_s_comms[i-1]) / (lead_t_comms[i]-lead_t_comms[i-1])

                lead_v_comms[i] = max(lead_v_comms[i], 0.)

        assert len(lead_s_comms) == n_pred_steps, 'Length of comms vectors are different.'
        assert len(lead_v_comms) == n_pred_steps, 'Length of comms vectors are different.'

        ### Build prediction
        # Current measurements
        t_pred = t

        k = 0 # Current PV state measurements
        self.api.inputs_p.contents.acc_pred[k] = lead_accel
        self.api.inputs_p.contents.vel_pred[k] = lead_speed
        self.api.inputs_p.contents.pos_pred[k] = ego_distance + lead_rel_distance
        self.api.inputs_p.contents.time_pred[k] = t

        for k in range(1, n_pred_steps): # Predicted PV states
            # Propagate kinematic constant acceleration prediction
            t_pred = lead_t_comms[k]

            self.api.inputs_p.contents.acc_pred[k] = lead_accel
            self.api.inputs_p.contents.vel_pred[k] = lead_v_comms[k]
            self.api.inputs_p.contents.pos_pred[k] = ego_distance + lead_rel_distance + lead_s_comms[k]
            self.api.inputs_p.contents.time_pred[k] = t_pred

    def getCommand(self, t, ego_accel, ego_speed, ego_distance, s_max, v_max, lead_accel, lead_speed, lead_rel_distance, lead_t_comms=[], lead_s_comms=[], lead_v_comms=[]):
        '''
        Compute the control command for PCC vehicle given input signals. Calls the PCC compiled controller

        Inputs:
        - t: Current time [s]
        - ego_accel: Current ego vehicle acceleration [m/s2]
        - ego_speed: Current ego vehicle speed [m/s]
        - ego_distance: Current front-axis ego vehicle position. Can be 0 [m]
        
        - s_max: Max distance forward the ego is constrained to go to [m]
        - v_max: Max speed the ego is constrained to go with [m/s]

        - lead_accel: Current measured acceleration of preceding vehicle [m/s2]
        - lead_speed: Current measured speed of preceding vehicle [m/s]
        - lead_rel_distance: Current measured front-to-back bumper gap from preceding vehicle [m]
        
        - lead_t_comms=np.empty((0)): (Optional) Communicated intention time vector from preceding vehicle [s]
        - lead_s_comms=np.empty((0)): (Optional) Communicated intention forward position vector from preceding vehicle [m]
        - lead_v_comms=np.empty((0)): (Optional) Communicated intention speed vector from preceding vehicle [m/s]
        '''

        ### Set the input signals
        self.api.inputs_p.contents.t = t

        self.api.inputs_p.contents.ego_state[0] = ego_distance  # Frenet front bumper position
        self.api.inputs_p.contents.ego_state[1] = ego_speed  # Frenet forward velocity
        self.api.inputs_p.contents.ego_state[2] = ego_accel  # Frenet forward acceleration

        # Run a prediction of the preceding vehicle motion - constant acceleration assumption here
        self.resetPred()

        has_intent_communication = len(lead_t_comms) > 0 and not any(not isfinite(x) for x in lead_t_comms)

        if has_intent_communication: # Use the communicated intentions
            self.useComms(t, lead_accel, lead_speed, lead_rel_distance, ego_distance, lead_t_comms, lead_s_comms, lead_v_comms)
            
        else: # Use a future prediction from the current states
            self.predAcc(t, lead_accel, lead_speed, lead_rel_distance, ego_distance, v_max)    

        # Set constraints
        self.api.inputs_p.contents.pos_max = s_max
        self.api.inputs_p.contents.vel_max = v_max

        ### Run the controller
        self.api.step_inputs()
        self.api.step_controller()
        self.api.step_outputs()

        ### Assign outputs struct properties
        acc_des = self.api.outputs_p.contents.acc_des

        state_trajectory = self.api.outputs_p.contents.state_trajectory
        control_trajectory = self.api.outputs_p.contents.control_trajectory
        time_trajectory = self.api.outputs_p.contents.time_trajectory
        slacks = self.api.outputs_p.contents.slacks
        reference = self.api.outputs_p.contents.reference
        constraint = self.api.outputs_p.contents.constraint
        cost = self.api.outputs_p.contents.cost
        exitflag = self.api.outputs_p.contents.exitflag

        # Expected state trajectory from the MPC model
        # state_trajectory is a 1D list containing the states [s v a] for stages 1..N - stage 0 would be the current ego vehicle states, stage 1 is immediate predicted states, stage 2 is two-step predicted states, etc
        n_states = 3 # Number of states in the MPC - fixed number

        pos_traj = state_trajectory[0::n_states] # Pos state starts at index 0
        vel_traj = state_trajectory[1::n_states] # Vel state starts at index 1
        acc_traj = state_trajectory[2::n_states] # Acc state starts at index 2

        time_traj = time_trajectory[0::]

        # Assign the control
        self.ua = acc_des
        
        return acc_des, time_traj, pos_traj, vel_traj, acc_traj
    
    def __repr__(self):
        inputs = self.api.inputs_p.contents
        outputs = self.api.outputs_p.contents

        return "  t={:0.1f}, s={:0.1f}, v={:0.1f}, a={:0.1f}, pv_gap={:0.1f}, pv_v={:0.1f}, pv_a={:0.1f}, s_max={:0.1f}, v_max={:0.1f}, u={:0.1f}".format(
            inputs.t, inputs.ego_state[0], inputs.ego_state[1], inputs.ego_state[2],
            inputs.pos_pred[0], inputs.vel_pred[0], inputs.acc_pred[0],
            inputs.pos_max, inputs.vel_max,
            outputs.acc_des
            )

class CAV(_vehicle):
    '''CAV driver for virtual automated vehicles'''
    def __init__(self, t0 = -1., vd = 20., id = -1, active=False):
        super().__init__(t0, vd, id)

        self.type = ControlType.CAV.value

        # Parameters
        self.vd = vd # Desired velocity [m/s]
        self.active = active # True/false to allow lane changes to activate
        
        # Controls
        self.ua = 0. # Acceleration command
        self.ul = 0 # Lane change command

        self.acc_des, self.vel_des, self.pos_des = 0., 0., 0.

        # Additional Controls
            # The flags are:
            # -4: constant braking for stops (at very low speeds)
            # -1: Stops (no LT traj)
            # -2: Stops (no LT traj)
            # 1: LT free-flow
            # 2: ST car-following
            # 3: constant braking for stops
            # Others: stops

        self.flag = 0

        self.time_LTtraj = [None]*1000
        self.acc_LTtraj = [None]*1000
        self.vel_LTtraj = [None]*1000
        self.pos_LTtraj = [None]*1000
        
        self.time_STtraj = [None]*100
        self.acc_STtraj = [None]*100
        self.vel_STtraj = [None]*100
        self.pos_STtraj = [None]*100

        # Autogen Simulink Model Interface
        if os.name == 'nt':
            libraryname = 'cav_so'
        
        elif os.name == 'posix':
            libraryname = 'libcav_so'
        
        elif os.name == 'darwin':
            libraryname == 'libcav_so'

        else:
            raise ValueError('CAV class cannot determine system type!')
        
        self.api = c_api(libraryname)

    def getCommand(self, t, ego_accel, ego_speed, ego_distance, s_max, v_max, lead_accel, lead_speed, lead_rel_distance, tls):
        '''Set the control commands, for example desired acceleration and desired lane'''
        
        ### Assign inputs vectors
        # Intersections
        max_n_tls = 5
        IntscInfo = [0]*max_n_tls*6

        if len(tls) > max_n_tls:
            tls = get_n_latest_tls(max_n_tls, self.s, self.len, tls)

        for k in range(0, max_n_tls): # Default values
            IntscInfo[0 + k*6] = s_max
            IntscInfo[1 + k*6] = LightType.LIGHT.value
            IntscInfo[2 + k*6] = 90
            IntscInfo[3 + k*6] = 80
            IntscInfo[4 + k*6] = 5
            IntscInfo[5 + k*6] = 5

        for k in range(0, len(tls)):
            if tls[k].type == LightType.STOP.value:
                i = max_n_tls-1 # If a stop sign we are forcing to write to the last column of IntscInfo
            else:
                i = k

            IntscInfo[0 + i*6] = tls[k].s
            IntscInfo[1 + i*6] = tls[k].type
            IntscInfo[2 + i*6] = tls[k].cycle
            IntscInfo[3 + i*6] = tls[k].green
            IntscInfo[4 + i*6] = tls[k].amber
            IntscInfo[5 + i*6] = tls[k].phase

        IntscInfo[0 + (max_n_tls-1)*6] -= 0.4 # Move stopping distance backwards
        IntscInfo[1 + (max_n_tls-1)*6] = LightType.STOP.value # Force last to be a stop

        assert len(tls) <= max_n_tls, 'Max number of requested traffic lights not supported by CAVs!'

        # Speed limit
        SpdLimInfo = [0]*4

        SpdLimInfo[0] = 0
        SpdLimInfo[1] = v_max
        SpdLimInfo[2] = s_max
        SpdLimInfo[3] = v_max
        
        # Control inputs
        CtrlInfo = [0]*14

        CtrlInfo[0] = t
        
        CtrlInfo[4] = VEHLENGTH # Front bumper position - s is rear bumper position -> add length to get front bumper
        CtrlInfo[5] = ego_speed
        CtrlInfo[6] = ego_accel
        CtrlInfo[1:4] = CtrlInfo[4:7]

        CtrlInfo[7] = lead_rel_distance # sps - ss - d_min0;
        CtrlInfo[8] = lead_speed
        CtrlInfo[9] = lead_accel
        
        lightds, tlstatus, tlind = get_front_tl(self.s, self.len, tls)
        lighttype = tls[tlind].type
        if tlind >= 0:
            CtrlInfo[10] = tls[tlind].s # IntscPos
            CtrlInfo[11] = tls[tlind].type # IntscType
            CtrlInfo[12] = tls[tlind].status # TrfLghtState

        else:
            CtrlInfo[10] = s_max # IntscPos
            CtrlInfo[11] = LightType.STOP.value # IntscType
            CtrlInfo[12] = LightStatus.RED.value # TrfLghtState

        CtrlInfo[13] = v_max # current maximum speed limit
        
        # CAV controller parameter setup
        CtrlPar = [0]*15

        CtrlPar[0] = CONN_RANGE # ConnRange, m
        CtrlPar[1] = 10 # NxtIntscCntDstMrgn, m
        CtrlPar[2] = 200 # VrtPtDstMrgn, m
        CtrlPar[3] = 1 # dtGrnIniTimeMrgn, s
        CtrlPar[4] = 2 # dtGrnEndTimeMrgn, s
        CtrlPar[5] = v_max*0.95 # SpdDes, m/s
        CtrlPar[6] = 1 # dtMin, s
        CtrlPar[7] = 1 # dtMax, s
        CtrlPar[8] = 1 # CFopt, 0 or 1
        CtrlPar[9] = 2 # tHrznCF, s
        CtrlPar[10] = 1.2 # tau_d, s
        CtrlPar[11] = 3.2 # d_min, m
        CtrlPar[12] = 2.5 # aDes, m/s^2 1.5
        CtrlPar[13] = 2.5 # bDes, m/s^2 1.5
        CtrlPar[14] = self.is_acc_after_dest # IsAccAfterDes, 0 or 1
        
        # Stop sign behavior

        if self.is_waiting_sign:
            StopInfo = [-1, 3] # % stop mode (Stopped -1, Nearby 0, Moving 1) AND waiting time
        else:
            StopInfo = [1, 3] # % stop mode (Stopped -1, Nearby 0, Moving 1) AND waiting time

        if self.is_moving_sign:
            CtrlInfo[10] = nan
            CtrlInfo[11] = nan
            CtrlInfo[12] = LightStatus.GREEN.value

        ### Run the autogen controller
        self.acc_des, self.vel_des, self.pos_des, self.flag, self.time_LTtraj, self.acc_LTtraj, self.vel_LTtraj, self.pos_LTtraj, self.time_STtraj, self.acc_STtraj, self.vel_STtraj, self.pos_STtraj = self.api.step_controller(IntscInfo, SpdLimInfo, CtrlInfo, CtrlPar, StopInfo)
        
        # Assign the control
        self.ua = self.acc_des

class EXT(_vehicle):
    '''CAV driver for externally controlled vehicles placed into microsimulation'''
    def __init__(self, t0 = -1., vd = 20., id = -1):
        super().__init__(t0, vd, id)

        self.type = ControlType.EXT.value

        self.type_id = 'ext' # Vehicle type in SUMO
        self.veh_id = f'ext_{self.id}' # Unique vehicle id in SUMO
        self.route_id = 'mainlane' # Vehicle route to follow in SUMO

        # Controls
        self.ua = 0. # Acceleration command
        self.ul = 0 # Lane change command

        # Simulation properties
        self.is_exists = True
        self.is_exited = False

    def getCommand(self, svs, tls, t, dt):
        pass # EXT Wont have any command to compute since it is an external

class SAS(CAV):
    '''Eco Speed Advisory driver placeholder for externally controlled vehicles placed into microsimulation'''
    def __init__(self, t0 = -1., vd = 20., id = -1, active=False):
        super().__init__(t0, vd, id)
        
        self.type = ControlType.SAS.value

        # Simulation properties
        self.is_exists = True
        self.is_exited = False

    def getCommand(self, svs, tls, t, dt):
        super().getCommand(svs, tls, t, dt)
        
class TrafficLight:
    '''Traffic light fixed signal phase and timing for virtual intersections'''
    id = 1

    def __init__(self, s, x, y, theta, type, cycle, green, amber, phase, has_id=False):
        # Light position
        self.s = s 
        self.x = x
        self.y = y
        self.theta = theta

        # Light timings and current status
        self.status = LightStatus.RED.value
        self.type = type
        self.cycle = cycle
        self.green = green
        self.amber = amber
        self.phase = phase

        # Advanced light timings
        self.time_to_red = -1. # [s] Positive time if in amber phase and countdowns til the light changes to red
        self.time_to_green = -1. # [s] Positive time if in red phase and countdowns til the light changes to green

        # Unique light identifier in cases where this object is permanent/tied directly to a unique intersection
        if has_id:
            self.id = TrafficLight.id
            TrafficLight.id += 1

        else:
            self.id = None # Temporary/placeholder traffic light object

    def setStatus(self, status, time_to_red=-1., time_to_green=-1.):
        '''Directly set the current traffic intersection status of green, amber, red'''

        self.status=status

        if self.type == LightType.STOP.value:
            self.time_to_red = -1.
            self.time_to_green = -1.

        elif time_to_red > 0.:
            self.time_to_red = time_to_red
            self.time_to_green = -1.

        elif time_to_green > 0.:
            self.time_to_green = time_to_green
            self.time_to_red = -1.

    def setCommand(self, t):
        '''Set traffic intersection status based on fixed SP&T and car-following'''
        lt = fmod(t + self.phase, self.cycle) # Light timing for how far into the green-amber-red cycle we are

        if self.type == LightType.LIGHT.value:
            if lt < self.green:
                self.status = LightStatus.GREEN.value
            elif lt < self.green + self.amber:
                self.status = LightStatus.AMBER.value
            else:
                self.status = LightStatus.RED.value

        elif self.type == LightType.STOP.value:
            self.status = LightStatus.RED.value

        else:
            raise RuntimeError('Unknown light type')
        
        # Set the time_to_ vars
        if self.type == LightType.STOP.value:
            self.time_to_red = -1.
            self.time_to_green = -1.

        elif self.type == LightType.LIGHT.value:
            if self.status == LightStatus.AMBER.value:
                self.time_to_green = -1.
                self.time_to_red = max(self.green + self.amber - lt, 0)

            elif self.status == LightStatus.RED.value:
                self.time_to_green = max(self.cycle - lt, 0)
                self.time_to_red = -1.
            
            else:
                self.time_to_green = -1.
                self.time_to_red = -1.

        else:
            raise RuntimeError('Unknown light type')
