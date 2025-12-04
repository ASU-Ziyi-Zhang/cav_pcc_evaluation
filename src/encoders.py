#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### Nick Goberville                 ###
### Transportation Education (STEP) ###
### tard(at)anl(dot)gov             ###
#######################################

import struct
import json
import pathlib
import re
from abc import ABC, abstractmethod

from typing import Dict, Any

def _flatten_list(nested_list:list) -> list:
    """Recursively unrolls nested lists into a single list."""
    flat_list = []

    for item in nested_list:
        if isinstance(item, list):
            flat_list.extend(_flatten_list(item))  # Recursively flatten nested lists
        else:
            flat_list.append(item)
    
    return flat_list

def byte_encode_strings(lst:list) -> list:
    """Iterate through a list and encode any strings to bytes."""
    return [item.replace(' ', '\x00').encode(Encoder.str_encoder_type) if isinstance(item, str) else item for item in lst]

def byte_decode_strings(d:dict) -> dict:
    """Iterate through a list and decode any bytes objects to strings."""
    for key, item in d.items():
        if isinstance(item, bytes):
            d[key] = item.replace(b'\x00', b'').decode(Encoder.str_encoder_type).rstrip() # Decode from bytes array, remove null bytes, strip off padding from right side

    return d

class Encoder(ABC):
    """
    Encoder superclass.
    """
    str_encoder_type = "ascii" # How to encode strings - "ascii", "utf-8", "utf-16"

class JsonEncoder(Encoder):
    """
    Encodes a dictionary of signals into JSON format.
    """

    def decode(self, received_data):
        return json.loads(received_data.decode(self.str_encoder_type))
    
    def encode(self, message):
        return json.dumps(message).encode(self.str_encoder_type)

class ByteEncoder(Encoder):
    """
    Encodes a dictionary of signals into bytes based on a JSON-specified format for each signal.
    """

    @staticmethod
    def _check_file_exists(file_path):
        """Check if a file exists. If not, print the full path and raise an error."""
        path = pathlib.Path(file_path).resolve()  # Get absolute path
        
        if not path.is_file():
            print(f"File not found: {path}")
            raise FileNotFoundError(f"File does not exist: {path}")
        
        return True  # File exists

    def _load_signal_formats_from_file(self, json_file_path: str) -> Dict[str, str]:
        """
        Load signal formats from a JSON file.

        Args:
            json_file_path (str): Path to the JSON file containing signal formats.

        Returns:
            dict: Dictionary where keys are signal names and values are format characters.

        Raises:
            FileNotFoundError: If the JSON file is not found.
            json.JSONDecodeError: If the JSON file is not properly formatted.
        """
        try:
            # Check for file
            self._check_file_exists(json_file_path)

            # Open and read signal formats
            with open(json_file_path, 'r') as file:
                signal_formats = json.load(file)
                if not isinstance(signal_formats, dict):
                    raise ValueError("JSON file must contain a dictionary of signal formats.")
                
                return signal_formats
            
        except FileNotFoundError:
            raise FileNotFoundError(f"JSON file not found: {json_file_path}")
        
        except json.JSONDecodeError:
            raise ValueError(f"Invalid JSON format in file: {json_file_path}")
    
    def __init__(self, msg_class = None, json_dict:dict = {}, json_file_path: str = ''):
        """
        Initialize the encoder with a JSON format that specifies signal formats.

        Args:
            msg_class (class): Class template from the messages.py file. Will check for the msg_class.data_types property

            json_dict (dict): Directly assign the signal formats from a dict.
                                Example JSON: {"speed": "d", "gear": "i", "active": "?"}

            json_file_path (str): Path to the JSON file containing signal formats.
                                  Example JSON: {"speed": "d", "gear": "i", "active": "?"}
        """
        if msg_class:
            self.signal_formats = msg_class.data_types
        
        elif json_dict:
            self.signal_formats = json_dict
        
        elif json_file_path:
            self.signal_formats = self._load_signal_formats_from_file(json_file_path)

        # Construct the full format string based on signal order
        self.format_string = ''.join(self.signal_formats.values())
        
        # Store the expected signal order for encoding
        self.signal_order = list(self.signal_formats.keys())

    def encode(self, signal_values: Dict[str, Any]) -> bytes:
        """
        Encodes signal values into bytes using the format specified for each signal.

        Args:
            signal_values (dict): Dictionary with signal names as keys and their values as values.
                                  Example: {"speed": 55.0, "gear": 3, "active": True}

        Returns:
            bytes: Encoded byte data for the signals.

        Raises:
            ValueError: If signal_values does not contain all required signals.
        """
        # Check if unexpected signals are present in the input dictionary
        unexpected_signals = [s for s in signal_values if s not in self.signal_order]
        if any(unexpected_signals):
            raise ValueError(f"Unexpected signal in inputs: {unexpected_signals}")
        
        # Ensure all expected signals are present in the input dictionary
        missing_signals = [s for s in self.signal_order if s not in signal_values]
        if any(missing_signals):
            raise ValueError(f"Missing required signals: {missing_signals}")

        # Extract the values in the correct order
        values = _flatten_list([signal_values[signal] for signal in self.signal_order])

        # Encode any strings
        values = byte_encode_strings(values)

        # Encode the values based on the constructed format string
        try:
            encoded_data = struct.pack(self.format_string, *values)
            return encoded_data
        
        except struct.error as e:
            raise ValueError(f"Error in encoding with format '{self.format_string}': {e}")
        

    def decode(self, byte_data: bytes) -> Dict[str, Any]:
        """
        Decodes bytes into signal values according to the format specified in the JSON file.

        Args:
            byte_data (bytes): The bytes to decode, matching the format specified.

        Returns:
            dict: A dictionary where keys are signal names and values are the decoded data.

        Raises:
            ValueError: If decoding fails due to a mismatch in byte_data size or format.
        """
        try:
            # Unpack the byte data according to the format string
            unpacked_data = struct.unpack(self.format_string, byte_data)

            # Map each decoded value to its signal name
            # The arrays are multi-length so must check which signals and how big they are
            value_iter = iter(unpacked_data)  # Iterator for controlled extraction
        
            decoded_data = {}
            for key, fmt in zip(self.signal_formats.keys(), self.signal_formats.values()):
                # Detect numeric indicators in field names (assuming they imply array sizes in which the next n elements are an array)
                if re.search(r"\d+", fmt) and not 's' in fmt:
                    array_size = int(re.findall(r"\d+", fmt)[0])  # Extract numeric portion
                    decoded_data[key] = [next(value_iter) for _ in range(array_size)]
                
                else:
                    decoded_data[key] = next(value_iter)

            # If there are no arrays in msg
            # decoded_data = {self.signal_order[i]: unpacked_data[i] for i in range(len(unpacked_data))}

            # Decode any strings
            decoded_data = byte_decode_strings(decoded_data)

            # Return
            return decoded_data
        
        except struct.error as e:
            raise ValueError(f"Error decoding data with format '{self.format_string}': {e}")
