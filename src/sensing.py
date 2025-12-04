################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

import numpy as np
from math import nan

from src.settings import *


def clamp(value, min_value, max_value):
    """
    Restricts 'value' to stay within the range [min_value, max_value].
    If 'value' is less than min_value, returns min_value.
    If 'value' is greater than max_value, returns max_value.
    Otherwise, returns 'value' unchanged.
    """
    return max(min_value, min(value, max_value))

class Communications:
    """
    Communications class responsible for storing and accessing virtual V2V communications messages
    
    Functions:
    - update_veh_comms(ego_id, ego_time, ego_pos, ego_vel, ego_acc): Stores the current ego vehicle communications associated with the unique ego id
    - get_veh_comms(veh_id, range): Checks for and returns communications from vehicle id
    - remove_veh_comms(veh_id, range): Checks for and deletes communications from vehicle id
    """

    def __init__(self):
        # Unique mapping of vehicle IDs to their status/intentions
        self.V2V: dict[str, dict] = {} # Vehicle-to-vehicle communications map - most recently available communications are stored here
        self.V2I: dict[str, dict] = {} # Vehicle-to-infrastructure communications map - most recently available communications are stored here
        
    @staticmethod
    def compute_speed_profile(times, positions):
        """
        Computes the speed profile from lists of time and position values.
        Handles edge cases such as unequal list lengths, duplicate times, and smoothing.

        Args:
            times (list of float): List of time values in seconds.
            positions (list of float): List of position values in meters.

        Returns:
            list of float or None: List of speed values in meters per second,
            with None for the first entry to maintain alignment.
        """

        ### Check inputs
        if len(times) != len(positions):
            raise ValueError("Time and position lists must have the same length.")

        # Convert lists to NumPy arrays for efficient processing
        times = np.array(times, dtype=float)
        positions = np.array(positions, dtype=float)

        # Check for duplicate times (would lead to division by zero)
        time_diffs = np.diff(times)
        position_diffs = np.diff(positions)

        if np.any(time_diffs == 0):
            raise ValueError("Time values must be unique to avoid division by zero.")

        ### Compute speeds
        if np.any(time_diffs): # Trajectory provided
            speeds = position_diffs / time_diffs

            # Apply a rolling average for smoothing (optional)
            smoothed_speeds = np.convolve(speeds, np.ones(2)*0.5, mode='same')  

            # Append None for initial entry to match original list length
            speed_profile = [None] + smoothed_speeds.tolist()

        else: # Only a single status provided so differentiation method does not work
            speed_profile = [None]

        return speed_profile

    def update_veh_comms(self, ego_id:str, ego_times:list, ego_pos:list, ego_vel:list=[]):
        '''Update current vehicle communications for ego vehicle id'''
        # Check inputs
        if not ego_times and not ego_pos: # No status/trajectory information provided
            return

        if not ego_vel: # No speed information provided with trajectory so estimate based on differencing
            ego_vel = self.compute_speed_profile(ego_times, ego_pos)

        # Make communications message
        comm = {
            'times': ego_times,
            'rel_long_pos': ego_pos,
            'speed': ego_vel
        }

        self.V2V[ego_id] = comm

    def get_veh_comms(self, veh_id:str, current_t:float, distance_from_sv=CONN_RANGE, conn_range:float=CONN_RANGE):
        '''Get most recent vehicle communications from vehicle id. If vehicle is out of range or is not found, returns [], [], []'''
        t=[]
        s=[]
        v=[]

        if V2V_CONN_ENABLED and veh_id in self.V2V and distance_from_sv <= conn_range:
            comms = self.V2V[veh_id]

            if comms['times'][0] + 0.5 > current_t: # Check that this is a recent comm
                t = comms['times']
                s = comms['rel_long_pos']
                v = comms['speed']
        
        return t, s, v
        
    def remove_veh_comms(self, ego_id:str):
        '''Remove the vehicle comms'''
        if ego_id in self.V2V:
            del self.V2V[ego_id]

    def update_tl_comms(self, ego_id:str, comms):
        self.V2I[ego_id] = comms

    def get_tl_comms(self, ego_id:str):

        comms = {}
        
        if V2I_CONN_ENABLED and ego_id in self.V2I:
            comms = self.V2I[ego_id]
        
        return comms

    def remove_tl_comms(self, ego_id:str):
        if ego_id in self.V2I:
            del self.V2I[ego_id]

class VehicleState:
    """
    Represents the state of a vehicle with key attributes like position, speed, acceleration, heading, and trajectory.

    Parameters:
    - latitude (float): Geographic latitude of the vehicle.
    - longitude (float): Geographic longitude of the vehicle.
    - rel_long_gap (float): If the vehicle is a surrounding vehicle, the detected bumper to bumper gap.
    - rel_lat_gap (float): If the vehicle is a surrounding vehicle, the detected near side to side gap.
    - speed (float): Current vehicle speed in meters per second.
    - acceleration (float): Current acceleration in meters per second squared.
    - heading (float): Vehicle heading in radians.
    - heading_rate_change (float): Rate of heading change in radians per second.
    
    - intended_trajectory (dict): List representing the vehicle's intended path.
    """
    required_traj_keys = [
        'times',
        'rel_long_pos'
    ]
    optional_traj_keys = [
        'rel_lat_pos',
        'speed',
    ]

    @staticmethod
    def check_trajectory_keys_ok(intended_trajectory:dict):
        """
        Check for required keys in the intended_trajectory
        
        Returns:
            True if message keys are correctly formatted
        """

        # Check required and optionally-allowed keys
        if intended_trajectory:
            allowed_keys = VehicleState.required_traj_keys + VehicleState.optional_traj_keys
            invalids = set(intended_trajectory.keys()) - set(allowed_keys)

            return not any(invalids)
        
        else: # Trajectory is empty so no keys that could cause error
            return True
        
    def __init__(self, latitude=0.0, longitude=0.0, rel_long_gap=nan, rel_lat_gap=nan, speed=0.0, acceleration=0.0,
                 heading=0.0, heading_rate_change=0.0, times=[], rel_long_pos=[]):
                
        self.latitude = latitude
        self.longitude = longitude
        self.rel_long_gap = rel_long_gap
        self.rel_lat_gap = rel_lat_gap
        self.speed = speed
        self.acceleration = acceleration
        self.heading = heading
        self.heading_rate_change = heading_rate_change
        
        self.set_trajectory(times=times, rel_long_pos=rel_long_pos)

        # Check
        assert type(self.intended_trajectory) is dict, 'Trajectory information should be a dict'
        assert self.check_trajectory_keys_ok(self.intended_trajectory), 'Intended trajectory keys are mismatched.'

    def set_trajectory(self, intended_trajectory={}, times=[], rel_long_pos=[]):
        """
        Sets the intended vehicle trajectory

        Inputs:
            intended_trajectory (dict: str, list) - keys 'times', 'rel_long_pos'
        """
        # If dictionary is provided
        if intended_trajectory:
            self.intended_trajectory = intended_trajectory
        
        # If lists are provided
        elif times and rel_long_pos:
            self.intended_trajectory = {
                'times': times,
                'rel_long_pos': rel_long_pos
            }

        # Default empty
        else:
            self.intended_trajectory = {key: [] for key in self.required_traj_keys}

        ### Add optional keys that are not there
        for key in self.optional_traj_keys:
            if key not in self.intended_trajectory:
                self.intended_trajectory[key] = []
        
    def get_trajectory(self):
        """
        Unpacks vehicle intended trajectory information
        
        Returns:
            t (list): respective relative times for the trajectory - 0 is the time corresponding to when the message was sent
            
            s (list): respective relative positions for the trajectory
        """

        return self.intended_trajectory['times'], self.intended_trajectory['rel_long_pos']

    def __repr__(self):
        return f"VehicleState(lat={self.latitude}, lon={self.longitude}, speed={self.speed}, accel={self.acceleration}, heading={self.heading}, rate={self.heading_rate_change}, traj={self.intended_trajectory})"

def get_front_tl(s, l, tls):
    '''Get distance, status, and index of nearest traffic light in front of position s+l'''
    ds = 2000.
    ls = LightStatus.GREEN.value
    i = -1

    for k in range(0, len(tls)):
        d0 = tls[k].s - (s + l)
        if d0 > -5 and d0 < ds:
            ds, ls, i = d0, tls[k].status, k

    return ds, ls, i

def get_n_front_tls(n, s, l, tls):
    '''Get vector of n nearest traffic lights in front of position s+l'''
    # Get all traffic light distances from ego vehicle front bumper
    d0s = [tls[k].s - (s + l) for k in range(0, len(tls))]
    is_fronts = [d0 > -5 for d0 in d0s]

    # Sort the distances from smallest -> farthest
    d0s, is_fronts, tls = zip(*sorted(zip(d0s, is_fronts, tls)))

    # Get sorted traffic light objects for all those in front
    ntls = [tls[k] for k in range(0, len(tls)) if is_fronts[k]]

    # Only provide first n tls in front of ego
    if len(ntls) > n:
        ntls = ntls[0:n]
    
    return ntls

def get_n_latest_tls(n, s, l, tls):
    '''Get vector of n latest traffic lights, preferring those in front of position s+l'''
    # Get all traffic light distances from ego vehicle front bumper
    d0s = [tls[k].s - (s + l) for k in range(0, len(tls))]
    is_fronts = [d0 > -5 for d0 in d0s]

    # Sort the distances from smallest -> farthest
    d0s, is_fronts, tls = zip(*sorted(zip(d0s, is_fronts, tls)))

    # Get sorted traffic light objects for all those in front
    rtls = [tls[k] for k in range(0, len(tls)) if not is_fronts[k]]
    ftls = [tls[k] for k in range(0, len(tls)) if is_fronts[k]]

    # Only provide first n tls in front of ego
    len_ftls = len(ftls)
    if len_ftls > n:
        ftls = ftls[0:n]

    # Prepend with latest m tls behind of ego if not enough lights are in front
    ntls = ftls
    if len_ftls < n:
        m = n-len_ftls
        ntls = rtls[-m::] + ftls

    assert len(ntls) == n, 'Incorrect number of traffic lights found!'

    return ntls
