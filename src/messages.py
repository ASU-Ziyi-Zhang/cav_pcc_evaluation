#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

'''File containing classes for various vehicle message types including BSM'''

### Notes
# BSM message structure?:
# https://www.tomesoftware.com/wp-content/uploads/2019/08/5-2019-B2V-Workshop-Detroit-Farid-Ahmed-Zaid-BSM-Messages.pdf

from math import nan
from enum import Enum
from datetime import datetime, timezone
from abc import ABC
import hashlib
import numpy as np
import sys
from typing import Dict, Any

from src.encoders import Encoder

def _fits_in_double(value) -> bool:
    return sys.float_info.min <= abs(value) <= sys.float_info.max

def _force_length_n(s:str, n:int=16) -> str:
    return s[:n].ljust(n, '\x00')  # Truncate if too long, pad with spaces if too short


class MSG(ABC):
    """Messaging superclass meant to allow type-checking of its children classes."""
    
    # These must be defined in child classes
    required_keys: list
    defaults: dict
    data_types: dict

    # Common parameters
    msg_n = 31 # Number of characters in msg_type string
    id_n = 31 # Number of characters in id string
    bsm_traj_n = 10 # Maximal number of points in the trajectory

    # Common methods
    @staticmethod
    def _fill_message_defaults(message, defaults):
        """
        Assigns default values to optional keys if they are not present in the message.
        This ensures a complete data structure for downstream processing.
        """
        for key, default_value in defaults.items():
            if key not in message:
                message[key] = default_value
        
        return message
    
    @staticmethod
    def _check_message_requirements(message, required_keys):
        """
        Validates that all required fields are present in the provided motion data message.
        Raises a ValueError if any required key is missing.
        """
        if not all(key in message for key in required_keys):
            raise ValueError("Missing required fields in motion data.")
    
    @staticmethod
    def _check_keys(message, defaults, required_keys):
        """
        Validates that the keys in the message only correspond to known keys.
        Raises a ValueError if an invalid key is found.
        """
        allowed_keys = defaults.keys() | required_keys
        invalids = set(message.keys()) - allowed_keys

        if any(invalids):
            raise ValueError(f"Detected unrecognized keys supplied to motion data: {invalids}")

    @staticmethod
    def get_msg_type_from_bytes(byte_array:bytes) -> str:
        message = byte_array[:MSG.msg_n].decode( Encoder.str_encoder_type )
        return message.replace(" ", "").replace("\x00", "")

    @staticmethod
    def get_msg_type_from_dict(message:Dict[str,str]) -> str:
        return message['msg_type'].replace(" ", "").replace("\x00", "")

class BSM(MSG):
    """
    The BSM (Basic Safety Message) class represents vehicle motion data using structured keys, 
    default values, and validation methods. This class is custom and has some non-standardized fields and units.

    Attributes:
        required_keys (list[str]): List of keys that must be present in a motion data message.
        defaults (dict[str, Any]): Dictionary of optional keys with default values.
        data_types (dict[str, str]): Dictionary of data types associated with each required and default key.

    Enum Classes:
        TransmissionState: Defines possible transmission states for a vehicle.
        BrakingStatus: Represents whether the brakes are applied or not.
        TurnSignalStatus: Indicates the current turn signal status.
        Error: Defines possible error states within the system.

    Methods:
        - make_bsm(...) -> dict: Creates a bsm-format message based on required input vehicle data. Optional vehicle data can be input to the function using key=value arguments.
    """
    class Trajectory:
        """ The trajectory sub-message that includes latitude, longitude, and UTC time for these positions"""

        @staticmethod
        def downsample_to_n(values:list) -> list:
            """Interpolates a long trajectory down to n elements using linear interpolation."""
            
            n = BSM.bsm_traj_n
            
            # Check inputs
            if len(values) <= n:
                return values  # If already 10 or fewer, return unchanged

            # Set up interpolation
            x_original = np.linspace(0, len(values) - 1, len(values))  # Original indices
            x_new = np.linspace(0, len(values) - 1, n)  # Target indices for interpolation

            # Perform interpolation
            interpolated_values = np.interp(x_new, x_original, values)  # Perform linear interpolation

            interpolated_values[0] = values[0]  # Ensure first value remains unchanged
            interpolated_values[-1] = values[-1]  # Ensure last value remains unchanged

            return interpolated_values.tolist()

        @staticmethod
        def make_and_add_trajectory(bsm, time, rel_long_pos, rel_lat_pos=[None], type = 'path_intentions'):
            """Make a trajectory sub-message from times and positions and add it to the provided BSM."""
            assert type == 'path_intentions' or type == 'path_history', 'Unrecognized type of trajectory passed to adder.'

            # Fill in optional values
            if not rel_lat_pos or rel_lat_pos == None or all(l is None for l in rel_lat_pos):
                rel_lat_pos = [nan for _ in rel_long_pos]

            # Force the trajectory to be a certain size - fill in missing expected elements
            if len(time) < BSM.bsm_traj_n:
                default_value = nan
                
                time += [default_value] * (BSM.bsm_traj_n - len(time))
                rel_long_pos += [default_value] * (BSM.bsm_traj_n - len(rel_long_pos))
                rel_lat_pos += [default_value] * (BSM.bsm_traj_n - len(rel_lat_pos))

            # Force the trajectory to be a certain size
            time = BSM.Trajectory.downsample_to_n(time)
            rel_long_pos = BSM.Trajectory.downsample_to_n(rel_long_pos)
            rel_lat_pos = BSM.Trajectory.downsample_to_n(rel_lat_pos)

            # Make trajectory
            trajectory = {
                type+"_n": len(time),
                type+"_time": time, # time with respect to these positions [s]
                type+"_rel_lat_pos": rel_lat_pos,
                type+"_rel_long_pos": rel_long_pos
            }
            
            # Add to BSM
            bsm.update(trajectory)
            
            return bsm
        
        @staticmethod
        def has_trajectory(bsm, type = 'path_intentions'):
            """Checks if a trajectory sub-message exists in the BSM."""
            assert type == 'path_intentions' or type == 'path_history', 'Unrecognized type of trajectory passed to adder.'

            time_traj = bsm[type+"_time"]
            pos_traj = bsm[type+"_rel_long_pos"]

            has_times = all(t is not None and t is not nan for t in time_traj)
            has_longs = all(l is not None and l is not nan for l in pos_traj)
            
            return has_times and has_longs
        
        @staticmethod
        def get_trajectory(bsm, type = 'path_intentions'):
            """Checks if a trajectory sub-message exists in the BSM and returns it."""
            assert type == 'path_intentions' or type == 'path_history', 'Unrecognized type of trajectory passed to adder.'
            
            time_traj = bsm[type+"_time"]
            pos_traj = bsm[type+"_rel_long_pos"]

            return time_traj, pos_traj

    # Helper enums for each BSM message type, defining various vehicle states
    class TransmissionState(Enum):
        """ Defines possible transmission states of a vehicle """
        PARK = -2
        REVERSE = -1
        NEUTRAL = 0
        DRIVE = 1
        LOW_GEAR = 2

    class BrakingStatus(Enum):
        """ Represents whether the brakes are applied or not """
        NOT_APPLIED = 0
        APPLIED = 1

    class TurnSignalStatus(Enum):
        """ Indicates the active turn signal status """
        OFF = 0
        LEFT = 1
        RIGHT = 2
        HAZARD = 3

    class Error(Enum):
        """ Defines error status codes related to vehicle communication """
        OK = 0
        DEFAULT_ERROR = 1
        # Additional specific error types could be added here

    @staticmethod
    def _check_enum_fields(message):
        """
        Validates that specific optional fields in the message correspond to known enum values.
        Raises a ValueError if an invalid enum value is found.
        """
        if message["transmission_state"] not in [state.value for state in BSM.TransmissionState]:
            raise ValueError(f"Invalid transmission state: {message['transmission_state']}")

        if message["braking_status"] not in [status.value for status in BSM.BrakingStatus]:
            raise ValueError(f"Invalid braking status: {message['braking_status']}")

        if message["turn_signal_status"] not in [signal.value for signal in BSM.TurnSignalStatus]:
            raise ValueError(f"Invalid turn signal status: {message['turn_signal_status']}")

    # Required Keys: These fields must be present in the motion message
    required_keys = [
        "msg_type",  # Unique n char id associated with type of message

        "id", # Unique n char id associated with agent
        "current_utc_time",  # Timestamp in UTC format

        "latitude",  # Geographic latitude
        "longitude",  # Geographic longitude
        "speed",  # Vehicle speed in meters per second
        "acceleration",  # Vehicle acceleration in meters per second squared
        "heading",  # Vehicle heading in radians
        "heading_rate_change",  # Rate of heading change in radians per second

        "error_flag"  # Error status flag
    ]
    
    # Optional Keys with Default Values: If not present in the message, default values are used
    defaults = {
        # In an XIL setting relative gaps are reported from simulation to hardware
        "rel_long_gap": nan, # Bumper to bumper gap in meters
        "rel_lat_gap": nan, # Vehicle side to vehicle side gap in meters

        # Optional status information
        "elevation": 0.0,  # Altitude above sea level in meters
        "position_accuracy": 1.5,  # Estimated position accuracy in meters (1-sigma deviation) - 1.5m is the worst-resolution allowed accuracy
        "steering_angle": 0.0,  # Steering wheel angle in radians
        "transmission_state": TransmissionState.DRIVE.value,  # Default transmission state
        "braking_status": BrakingStatus.NOT_APPLIED.value,  # Default brake status
        "turn_signal_status": TurnSignalStatus.OFF.value,  # Default turn signal status
        "length": 4.2,  # Vehicle length in meters
        "width": 1.8,  # Vehicle width in meters
        
        # History and intended path
        "path_history_n": MSG.bsm_traj_n,
        "path_history_time": [nan]*MSG.bsm_traj_n,
        "path_history_rel_long_pos": [nan]*MSG.bsm_traj_n,
        "path_history_rel_lat_pos": [nan]*MSG.bsm_traj_n,

        "path_intentions_n": MSG.bsm_traj_n,
        "path_intentions_time": [nan]*MSG.bsm_traj_n,
        "path_intentions_rel_long_pos": [nan]*MSG.bsm_traj_n,
        "path_intentions_rel_lat_pos": [nan]*MSG.bsm_traj_n
    }

    # Encoding type associated with each field
    data_types = {
        "msg_type": f"{MSG.msg_n}s",

        "id": f"{MSG.id_n}s",
        "current_utc_time": "d",

        "latitude": "d",
        "longitude": "d",
        "rel_long_gap": "d",
        "rel_lat_gap": "d",
        "speed": "d",
        "acceleration": "d",
        "heading": "d",
        "heading_rate_change": "d",

        "error_flag": "h",

        "elevation": "d",
        "position_accuracy": "d",
        "steering_angle": "d",

        "transmission_state": "h",
        "braking_status": "h",
        "turn_signal_status": "h",
        "length": "d",
        "width": "d",

        "path_history_n": "H",
        "path_history_time": f"{MSG.bsm_traj_n}d",
        "path_history_rel_long_pos": f"{MSG.bsm_traj_n}d",
        "path_history_rel_lat_pos": f"{MSG.bsm_traj_n}d",

        "path_intentions_n": "H",
        "path_intentions_time": f"{MSG.bsm_traj_n}d",
        "path_intentions_rel_long_pos": f"{MSG.bsm_traj_n}d",
        "path_intentions_rel_lat_pos": f"{MSG.bsm_traj_n}d"
    }

    @staticmethod
    def make_bsm(id:str, latitude:float, longitude:float, speed:float, acceleration:float, heading:float, heading_rate_change:float, error_flag=Error.OK.value,
        **kwargs
    ) -> Dict[str, Any]:
        """
        Creates a bsm-format message based on required input vehicle data.
        Optional vehicle data can be input to the function using key=value arguments
        """
        message = {
            # Required keys
            "msg_type": _force_length_n("bsm", MSG.msg_n),
            
            "id": _force_length_n(id, MSG.id_n),  # Unique fixed-length ID (Hopefully unique)
            "current_utc_time": BSM.get_utc_time(),  # Timestamp in UTC format

            "latitude": latitude,  # Geographic latitude
            "longitude": longitude,  # Geographic longitude
            "speed": speed,  # Vehicle speed in meters per second
            "acceleration": acceleration,  # Vehicle acceleration in meters per second squared
            "heading": heading,  # Vehicle heading in radians with respect to latitude axis
            "heading_rate_change": heading_rate_change,  # Rate of heading change in radians per second
            
            "error_flag": error_flag,  # Using enum - Error status flag
        }
    
        # Validate that all provided keys exist in defaults
        invalid_keys = [key for key in kwargs if key not in BSM.defaults]
        if invalid_keys:
            raise ValueError(f"Invalid keys detected: {invalid_keys}")

        # Include optioinal keys that were explicitly provided by the caller
        message_opt = {key: kwargs[key] for key in kwargs if key in BSM.defaults}
        message.update(message_opt)

        # Fill out and check the rest of the message
        message = BSM.fill_and_check_message(message)

        return message
    
    @staticmethod
    def get_utc_time():
        """
        Returns the current UTC (Coordinated Universal Time) time
        """
        t = datetime.now(timezone.utc).timestamp()
        assert _fits_in_double(t), 'Timestamp does not fit within double.'
        
        return t
    
    @staticmethod
    def fill_and_check_message(message):
        """
        Validates that all required fields are present in the provided motion data message.
        Returns an updated message that populates all missing, optional data fields with defaults.
        Raises a ValueError if any required key is missing, or if unrecognized keys/values are detected.
        """

        if message:
            message = MSG._fill_message_defaults(message, BSM.defaults)

            MSG._check_message_requirements(message, BSM.required_keys)
            MSG._check_keys(message, BSM.defaults, BSM.required_keys)

            BSM._check_enum_fields(message)

        return message
    


class SIM(MSG):
    """
    The SIM class represents simulation status data using structured keys, 
    default values, and validation methods.

    Attributes:
        required_keys (list[str]): List of keys that must be present in a sim data message.
        defaults (dict[str, Any]): Dictionary of optional keys with default values.
        data_types (dict[str, str]): Dictionary of data types associated with each required and default key.

    Enum Classes:
        SimStatus: Defines possible xil simulation statuses.
        Error: Defines possible error states within the system.

    Methods:
        - make_sim(message: dict) -> dict: Creates and validates required fields in the message and fills optional keys.
    """
    # Helper enums for each BSM message type, defining various vehicle states
    class SimStatus(Enum):
        """ Defines possible xil simulation statuses """
        OFFLINE = -2 # Shutdown or will shutdown and will cease communication
        NOT_READY = -1 # Communicating and preparing but not ready yet
        WAITING = 0 # Communicating and ready but waiting for confirmation to start
        RUNNING = 1 # Communicating and ready, has received confirmation to start
        RESET = -3 # Request for the current scenario to be reset, the server program will reset but not need to be re-launched manually

    class Error(Enum):
        """ Defines error status codes related to vehicle communication """
        OK = 0
        DEFAULT_ERROR = 1
        # Additional specific error types could be added here

    @staticmethod
    def _check_enum_fields(message):
        """
        Validates that specific optional fields in the message correspond to known enum values.
        Raises a ValueError if an invalid enum value is found.
        """
        if message["sim_status"] not in [state.value for state in SIM.SimStatus]:
            raise ValueError(f"Invalid simulation status: {message['sim_status']}")

    # Required Keys: These fields must be present in the motion message
    required_keys = [
        "msg_type",  # Unique n char id associated with type of message

        "id", # Unique n char id associated with agent
        "sim_status",  # Simulation status enum flag

        "error_flag"  # Error status enum flag
    ]
    
    # Optional Keys with Default Values: If not present in the message, default values are used
    defaults = {}

    # Encoding type associated with each field
    data_types = {
        "msg_type": f"{MSG.msg_n}s",

        "id": f"{MSG.id_n}s",
        "sim_status": "h",
        "error_flag": "h",
    }

    @staticmethod
    def make_sim(id:str, sim_status, error_flag=Error.OK.value
    ) -> Dict[str, Any]:
        """
        Creates a bsm-format message based on required input vehicle data.
        Optional vehicle data can be input to the function using key=value arguments
        """
        message = {
            # Required keys
            "msg_type": _force_length_n("sim", MSG.msg_n),
            
            "id": _force_length_n(id, MSG.id_n),  # Unique fixed-length ID (Hopefully unique)
            "sim_status": sim_status,
            "error_flag": error_flag,  # Using enum - Error status flag
        }

        # Fill out and check the rest of the message
        message = SIM.fill_and_check_message(message)

        return message
    
    @staticmethod
    def fill_and_check_message(message):
        """
        Validates that all required fields are present in the provided motion data message.
        Returns an updated message that populates all missing, optional data fields with defaults.
        Raises a ValueError if any required key is missing, or if unrecognized keys/values are detected.
        """

        if message:
            message = MSG._fill_message_defaults(message, SIM.defaults)

            MSG._check_message_requirements(message, SIM.required_keys)
            MSG._check_keys(message, SIM.defaults, SIM.required_keys)
            
            SIM._check_enum_fields(message)

        return message
    

class SPAT(MSG):
    class Error(Enum):
        """ Defines error status codes related to vehicle communication """
        OK = 0
        DEFAULT_ERROR = 1
        # Additional specific error types could be added here

    @staticmethod
    def _check_enum_fields(message):
        """
        Validates that specific optional fields in the message correspond to known enum values.
        Raises a ValueError if an invalid enum value is found.
        """
        pass

    required_keys = [
        "msg_type",  # Unique n char id associated with type of message

        "id", # Unique n char id associated with agent

        "error_flag"  # Error status enum flag
    ]
    
    # Optional Keys with Default Values: If not present in the message, default values are used
    defaults = {}

    # Encoding type associated with each field
    data_types = {
        "msg_type": f"{MSG.msg_n}s",

        "id": f"{MSG.id_n}s",

        "error_flag": "h",
    }

    @staticmethod
    def make_spat(id:str, error_flag=Error.OK.value
    ) -> Dict[str, Any]:
        """
        Creates a bsm-format message based on required input vehicle data.
        Optional vehicle data can be input to the function using key=value arguments
        """
        message = {
            # Required keys
            "msg_type": _force_length_n("sim", MSG.msg_n),
            
            "id": _force_length_n(id, MSG.id_n),  # Unique fixed-length ID (Hopefully unique)

            "error_flag": error_flag,  # Using enum - Error status flag
        }

        # Fill out and check the rest of the message
        message = SIM.fill_and_check_message(message)

        return message
    
    @staticmethod
    def fill_and_check_message(message):
        """
        Validates that all required fields are present in the provided motion data message.
        Returns an updated message that populates all missing, optional data fields with defaults.
        Raises a ValueError if any required key is missing, or if unrecognized keys/values are detected.
        """

        if message:
            message = MSG._fill_message_defaults(message, SPAT.defaults)

            MSG._check_message_requirements(message, SPAT.required_keys)
            MSG._check_keys(message, SPAT.defaults, SPAT.required_keys)
            
            SPAT._check_enum_fields(message)

        return message