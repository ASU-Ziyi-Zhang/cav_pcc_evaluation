#! /usr/bin/env python3

#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

# Simulation script to mock the vehicle-level sensors and actuation

import traceback
from time import perf_counter as counter
import argparse
import numpy as np
from math import sin, cos, sqrt, fmod, pi
from random import gauss

from src.settings import *
from src.sensing import *
from src.xil_client import UDPClient
from src.agents import PCC
from src.messages import BSM, SPAT, SIM
import parsers.client

from ext.dynamics import step_dyn

### Settings
MAX_N_RESETS = 1 # The number of trials the client program will attempt
RESETS = 0 # The number of trials the client has currently performed
TEND_XIL = 140. # progressed time until XIL client automatically ends simulation [s]


class Noise:
    '''Measurement noise simulation to mock sensor characteristics'''
    def __init__(self, use=True):
        # Settings
        self.use = use # 0 (no noise) or 1 (use measurement noise)
        self.case = 'ornstein-uhlenbeck' # 'gaussian', 'ornstein-uhlenbeck'
        
        # Sensor characteristics
        self.coord_bias = 0. # [m] - Mean on a reading bias
        self.velx_bias = 0. # [m/s]
        self.vely_bias = 0. # [m/s]
        self.acc_bias = 0. # [m/s2]
        self.heading_bias = 0. # [rad]

        self.coord_std = 0.010 # [(m)^2] # Standard deviation on reading noise
        self.vel_std = 0.080 # [(m/s)^2]
        self.acc_std = 0.200 # [(m/s2)^2]
        self.heading_std = 0.005 # [(rad)^2] - Noise here can cause the visualization to chatter

        # Process
        self.xp = 0. # RTK frame x measurement process state
        self.yp = 0. # RTK frame y process state
        self.vxp = 0. # RTK frame velocity in x process state
        self.vyp = 0. # RTK frame velocity in y process state
        self.ap = 0. # forward acceleration process state
        self.hp = 0. # RTK frame heading process state

        # Measurements
        self.x = 0. # RTK frame x measurement with noise
        self.y = 0. # RTK frame y with noise
        self.vx = 0. # RTK frame velocity in x with noise
        self.vy = 0. # RTK frame velocity in y with noise
        self.a = 0. # forward acceleration with noise
        self.heading = 0. # RTK frame heading with respect to y axis and with noise

    def step_noise(self, dt, x, y, v, a, heading):
        '''returns xn, yn, vxn, vyn, an, thn'''
        if self.use: # If enabling noise
            if self.case == 'gaussian':
                self.step_gauss()
            elif self.case == 'ornstein-uhlenbeck':
                self.step_ou(dt)
            else:
                raise ValueError('Unrecognized noise process case.')
        
        self.x = x + self.xp # True state + sensor process state
        self.y = y + self.yp # True state + sensor process state
        self.vx = v*sin(heading) + self.vxp # True state + sensor process state
        self.vy = v*cos(heading) + self.vyp # True state + sensor process state
        self.a = a + self.ap # True state + sensor process state
        self.heading = heading + self.hp # True state + sensor process state

    def step_ou(self, dt):
        '''Step the states of the noise forward using an Ornstein-Uhlenbeck process'''
        theta_ou = 1.0 # Mean reversion rate

        self.xp += theta_ou*(self.coord_bias - self.xp) + self.coord_std*np.sqrt(dt)*np.random.normal()
        self.yp += theta_ou*(self.coord_bias - self.yp) + self.coord_std*np.sqrt(dt)*np.random.normal()
        self.vxp += theta_ou*(self.velx_bias - self.vxp) + self.vel_std*np.sqrt(dt)*np.random.normal()
        self.vyp += theta_ou*(self.vely_bias - self.vyp) + self.vel_std*np.sqrt(dt)*np.random.normal()
        self.ap += theta_ou*(self.acc_bias - self.ap) + self.acc_std*np.sqrt(dt)*np.random.normal()
        self.hp += theta_ou*(self.heading_bias - self.hp) + self.heading_std*np.sqrt(dt)*np.random.normal()

    def step_gauss(self):
        '''Step the states of the noise forward using a Gaussian process'''
        self.xp = gauss(mu=self.coord_bias, sigma=self.coord_std) # Sensor bias by gaussian sample
        self.yp = gauss(mu=self.coord_bias, sigma=self.coord_std) # Sensor bias by gaussian sample
        self.vxp = gauss(mu=self.velx_bias, sigma=self.vel_std) # Sensor bias by gaussian sample
        self.vyp = gauss(mu=self.vely_bias, sigma=self.vel_std) # Sensor bias by gaussian sample
        self.ap = gauss(mu=self.acc_bias, sigma=self.acc_std) # Sensor bias by gaussian sample
        self.hp = gauss(mu=self.heading_bias, sigma=self.heading_std) # Sensor bias by gaussian sample

class SIL:
    """
    
    """
    def setLocation(self):
        '''Sets the road location where testing to take place'''
        ### Assign road direction and read X and Y coordinates of road
        if self.args.scenario == "ITIC": # TODO for sumo
            # ITIC is set as a straight road but 2 possible directions:
            if self.args.dir == 0:
                x0, y0 = 308.313, -156.179 # From 3-lane ITIC map LONG
                x1, y1 = 587.9804, -1531.023 # From 3-lane ITIC map LONG

            elif self.args.dir == 1:
                x0, y0 = 595.2319, -1529.5 # Start point - From 3-lane ITIC map LONG
                x1, y1 = 315.566, -154.709 # End point - From 3-lane ITIC map LONG

            else:
                raise RuntimeError('Unrecognized VIL network direction')
            
            # Make the road frame using start and end points of straight segment
            rf = RoadFrame(
                road_waypoints_xy=np.array( ([x0, y0], [x1, y1]) ))

        elif self.args.scenario == "CMI": # TODO
            # CMI is a offsite road that can fit two lanes in one direction
            # Make the road frame using csv containing the waypoints
            if self.args.dir == 0:
                filename = 'CMI_right_lane_glob.csv'
            elif self.args.dir == 1:
                filename = 'CMI_right_lane_reversed.csv'
            else:
                raise ValueError('Unrecognized VIL network direction')
            
            rf = RoadFrame(
                road_waypoints_csv=filename)

        else:
            raise ValueError('Unrecognized road case for location!')
        
        return rf

    def __init__(self):
        ### Handle input arguments
        # Setup parser
        parser = argparse.ArgumentParser('vehicle_simulation')

        parser.add_argument('--scenario', nargs='?', help='Road to use: 0 for ITIC, 1 for CMI. Default 0', type=str, default=0)
        parser.add_argument('--noise', help='Flag to additionally simulate sensor measurement noise. Otherwise, use ideal sensors.', action="store_true")
        parser.add_argument('--id_number', type=int, default=1, help='Number to add to ID to identify ego vehicle in this program. E.g. id=python_ego_<n>')

        parsers.client.register_parser(parser)
    
        self.args = parser.parse_args()
        
        ### Initialize variables
        # gamma - Road orientation angle at a point s

        # rf = setLocation() # RoadFrame object
        # x0, y0 = rf.getStartingCoords()
        x0, y0 = 0., -1.5*LANEWIDTH # y=0 is leftmost shoulder in sumo

        # s, s_dot, s_ddot, l, l_dot, gamma = rf.xy2sl(x0, y0, 0., 0., 0.)

        # Ego states
        # s - Forward position of rear axle along the road [m]
        # l - Lateral position of rear axle from right [lanes] - 1.0 is centerline right lane, 2.0 next lane...
        # l_dot - [lanes/s]

        self.x = x0 # RTK x position of center of rear axle [m]
        self.y = y0 # RTK y position of center of rear axle [m]
        self.heading = 0.5*pi # Ego rotation in RTK frame with respect to y-axis going CW [rad]
        self.v = 0.0 # Forward velocity [m/s]
        self.a = 0.0 # Forward acceleration [m/s2]

        self.u0 = 0.0 # Acceleration request [m/s2]
        self.u1 = 1 # Integer lane request - 1 rightmost lane, 2 second lane, ...

        self.id = f"python_ego_{str(self.args.id_number)}" # Unique id associated with ego

        # Ego planned trajectory
        self.time_traj = []
        self.forward_traj = []

        ### Sensor noise
        self.no = Noise(use=self.args.noise)

        ### CAV Controller
        self.control = PCC()

        ### Initialize communications with server
        # Run client to send/listen for server messages
        self.client = UDPClient(server_ip=self.args.server_ip, server_port=self.args.server_port, framerate=self.args.framerate)  
        
        ### Simulation
        self.dt = 1./self.client.framerate
        self.t = 0.
        self.iter = 0
        self.t_start = counter()

    def publish_status(self, sim_status):
        sim_data = SIM.make_sim(self.id, sim_status)

        self.client.send_update_data(sim_data)

    def publish_bsm(self):
        # Pack data
        motion_data = BSM.make_bsm(self.id, self.no.y, self.no.x, sqrt(self.no.vx**2+self.no.vy**2), self.no.a, self.no.heading, 0.0, BSM.Error.OK.value,
                     length=VEHLENGTH, width=VEHWIDTH)

        motion_data = BSM.Trajectory.make_and_add_trajectory(motion_data, self.time_traj, self.forward_traj)

        # Send data
        self.client.send_update_data(motion_data)

    def reset(self):
        '''
        Resets the simulation and communication client.
        '''

        if self.client.is_running():
            self.publish_status(SIM.SimStatus.RESET.value)
            self.client.stop()

        # Restart
        main()

    def stop(self):
        '''Stops the SIL simulation'''
        if self.client.is_running():
            self.publish_status(SIM.SimStatus.OFFLINE.value)
            
            self.client.stop()

    def step(self):
        '''Step a single iteration of the SIL simulation'''
        ### Get most recent update from microsim server
        bsm_data = self.client.get_data('bsm')

        ### Step vehicle control and vehicle dynamics
        # PV states as from server
        pv_gap, pv_speed, pv_accel = 2000., 0., 0. # Defaults
        lead_t_comms, lead_s_comms, lead_v_comms = [], [], [] # Defaults

        if bsm_data:
            for id, bsm in bsm_data.items():      
                # Get closest surrounding vehicle that was sent
                if bsm["rel_long_gap"] < pv_gap:
                    pv_gap = bsm["rel_long_gap"]
                    pv_speed = bsm["speed"]
                    pv_accel = bsm["acceleration"]

                    # Get preceding vehicle communications
                    if BSM.Trajectory.has_trajectory(bsm):
                        lead_t_comms, lead_s_comms = BSM.Trajectory.get_trajectory(bsm)

        # Ego states
        ego_distance = 0. # traci.vehicle.getDistance(ego_id) # From starting point - TODO behavior if multiple starting points?
        ego_speed = sqrt(self.no.vx**2+self.no.vy**2)
        ego_accel = self.no.a
    
        # Ego constraints
        v_max = 30. / 2.23693629
        s_max = 1200 - self.x

        ## Apply control
        self.u0, self.time_traj, self.forward_traj, vel_traj, acc_traj = self.control.getCommand(self.t, ego_accel, ego_speed, ego_distance, s_max, v_max, pv_accel, pv_speed, pv_gap, lead_t_comms, lead_s_comms, lead_v_comms)

        self.t, self.x, self.y, self.v, self.a, self.heading = step_dyn(self.dt, self.t, self.x, self.y, self.v, self.a, self.heading, self.u0)
        self.no.step_noise(self.dt, self.x, self.y, self.v, self.a, self.heading)

        ### Get server data update for clients
        self.publish_bsm()

        if abs(fmod(self.t, 1)) < 1e-6: # Publish every n seconds of simulation time
            self.publish_status(SIM.SimStatus.RUNNING.value)

        # Iterate
        if fmod(self.iter, 5)==0:
            print('real time: {:3.2f}, sim time: {:3.2f} | x: {:6.2f}, y: {:6.2f}'.format(
                counter()-self.t_start, self.t, 
                self.x, self.y)
            )

            print( repr(self.control) )
            
        self.iter += 1

    def rate(self):
        '''Control real time rate'''
        self.client.rate()

    def wait(self):
        '''Wait for other required nodes to come online'''
        first = True

        t = counter()
        while self.client.is_running() and counter() - t < 10:
            if first:
                print('Preparing client for XIL...')
                first = False

            self.publish_bsm()
            self.publish_status(SIM.SimStatus.NOT_READY.value)
        
        first = True

        while self.client.is_running() and not self.client.is_simming():
            if first:
                print('Waiting for first server data...')
                first = False

            self.publish_bsm()
            self.publish_status(SIM.SimStatus.WAITING.value)

            self.client.rate()

        self.publish_status(SIM.SimStatus.RUNNING.value)

    def sim(self):
        '''Run simulation of a real vehicle in SIL interface'''
        global RESETS
        if RESETS >= MAX_N_RESETS:
            return

        try:
            # Wait for other nodes/server
            self.wait()

            ### Simulation
            # Initialize external agent
            self.t_start = counter()

            # Main loop
            while self.client.running.is_set() and counter() - self.t_start < TEND_XIL and self.client.is_simming():
                # Step the SIL simulation
                self.step()

                # Rollover and control runtime rate
                self.client.rate()

            if counter() - self.t_start >= TEND_XIL:
                print('SIL shutdown automatically due to t > t_end.')
        
        except KeyboardInterrupt:
            pass

        except AssertionError as e:
            print(f'An assertion error occured: {e}')

        except IndexError as e:
            print(traceback.format_exc())
            print(f'Index error occurred: {e}')

        except KeyError as e:
            print(traceback.format_exc())
            print(f'Key error occurred: {e}')

        except Exception as e:
            print(traceback.format_exc())
            print(f'An unknown exception occured: {e}')

        # Stop
        try:
            RESETS += 1
            if RESETS < MAX_N_RESETS and self.client.is_simming():
                self.reset()

            self.stop()

        except KeyboardInterrupt as e:
            RESETS = MAX_N_RESETS
            
        except Exception as e:
            print(f'An exception occured: {e}')

def main():
    veh = SIL()
    veh.sim()
  
if __name__ == '__main__':
    # Run simulation
    main()

    print('Exited vehicle simulation.')