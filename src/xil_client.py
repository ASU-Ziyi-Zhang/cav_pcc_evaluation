#!/usr/bin/env python3

#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import argparse
import socket
import json
import threading
import warnings
import copy 

from time import perf_counter as counter
from time import sleep

from src.messages import BSM, SIM, SPAT, MSG
from src.encoders import ByteEncoder, JsonEncoder
import parsers.client

# from contextlib import contextmanager
# @contextmanager # Maybe useful someday
# def acquire_timeout(lock, timeout):
#     result = lock.acquire(timeout=timeout)
#     try:
#         yield result
#     finally:
#         if result:
#             lock.release()

class UDPClient:
    """
    A multi-threaded UDP client for sending motion data to a server and receiving updates.

    Parameters:
    - server_ip (str): The IP address of the server to connect to. Defaults to "127.0.0.1".
    - server_port (int): The UDP port number used for communication with the server. Defaults to 12345.
    - framerate (int): The update rate (frames per second) for sending motion data. Defaults to 5.

    Attributes:
    - update_interval (float): The time (seconds) between sending updates.
    - time_at_last_update (float): Timestamp of the last update sent to the server.
    - send_socket (socket.socket): UDP socket used for sending and receiving data.
    - bsm_data (dict): Stores received data from the server.
    - send_lock (threading.Lock): Ensures thread-safe access to client data.
    - read_lock (threading.Lock): Ensures thread-safe access to server data.

    Methods:
    - stop(): Gracefully stops the client, terminating threads and closing the socket.
    - send_update_data(update_data): Updates and sends motion data to the server.
    - get_bsm_data(): Retrieves the latest data received from the server.
    - rate(): Retrieves the latest data received from the server.
    - is_running(): Returns True/False if the client is currently running.
    - is_connected(): Returns True/False if the client is connected to the server.
    """

    def __init__(self, server_ip="127.0.0.1", server_port=12345, client_ip="127.0.0.1", framerate=20):
        """Initializes the UDP client."""
        ### Client Settings
        self.server_ip = server_ip
        self.server_port = server_port
        self.framerate = framerate
        self.client_ip = client_ip

        self.encoder_method = "byte" # Encoder/decoder method used - "json", "byte"
        
        if self.encoder_method == "byte":
            self.encoders = {
                'bsm': ByteEncoder(BSM),
                'sim': ByteEncoder(SIM),
                'spat': ByteEncoder(SPAT),
            }
            
            self.decoders = copy.deepcopy(self.encoders)
            
            self.num_recv_bytes = 1024
        
        elif self.encoder_method == "json":
            self.json_encoder = JsonEncoder()
            self.json_decoder = copy.deepcopy(self.json_encoder)

            self.num_recv_bytes = 1024*8

        else:
            raise ValueError(f"Encoder method unhandled. {self.encoder_method}")

        # Updates
        self.update_interval = 1./framerate  # Time between sending updates
        self.time_at_last_update = counter()  # The time that the last update was sent - will not send a new one until the update interval has passed
        self.time_at_start = counter()
        self.update_iter = 0 # Number of rate updates done

        # Initialize socket
        n_sockets = 3 # Number of sockets to open - corresponds with number of message types used
        
        self.send_sockets = {
            'bsm': socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
            'sim': socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
            'spat': socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
        }
        for i,send_socket in enumerate(self.send_sockets.values()):
            port = 10000 + n_sockets*i  
            send_socket.bind((self.client_ip, port))

        self.recv_sockets = {
            'bsm': socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
            'sim': socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
            'spat': socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
        }
        for i,recv_socket in enumerate(self.recv_sockets.values()):
            port = 10001 + n_sockets*i # Hard-code the listen ports based on server expectation
            recv_socket.bind((self.client_ip, port))

        self.recv_sockets_value_iter = iter( self.recv_sockets.values() )

        assert len(self.send_sockets) == len(self.recv_sockets), 'Send and received sockets are mismatched in length.'
        assert len(self.send_sockets) == n_sockets, 'Send and received sockets are mismatched in expected number of sockets.'

        # Data to/from stream 
        self.bsm_data = {}  # Shared data storage
        self.sim_data = {}
        self.spat_data = {}

        # Threading
        self.running = threading.Event()  # Event flag to stop sending threads

        self.send_lock = threading.Lock()  # Thread safety lock
        self.read_lock = threading.Lock()  # Thread safety lock

        ### Start
        self.connected = threading.Event() # Event Flag to indicate if connected to server
        self.simming = threading.Event() # Event Flag to indicate if the server is simming
        
        self.start()

        # Report
        print(f'Sending to server on {self.server_ip}:{self.server_port}')
    
    def __del__(self):
        self.stop()

    def stop(self):
        """Stops the client gracefully."""
        if self.running.is_set():
            # Print
            print("Client is shutting down...")

            # Close threads and close sockets
            self.running.clear()

            for listen_thread in self.listen_threads:
                listen_thread.join()

            for send_socket, recv_socket in zip(self.send_sockets.values(), self.recv_sockets.values()):
                send_socket.close() 
                recv_socket.close()
            
    def start(self):
        """
        Starts a separate thread to listen for motion data server responses.
        """
        if not self.running.is_set():
            self.running.set()

            self.listen_threads = [
                threading.Thread(target=self.listen_for_replies, args=(), daemon=True),
                threading.Thread(target=self.listen_for_replies, args=(), daemon=True),
                threading.Thread(target=self.listen_for_replies, args=(), daemon=True),
            ]

            for listen_thread in self.listen_threads:
                listen_thread.start()
    
    def send_update_data(self, update_data):
        """Sends serialized data to the server with status enums."""
        if self.running.is_set():
            try:
                # Safely update shared data
                with self.send_lock:
                    serialized_message, msg_type = self.encode_msg(update_data)
                
                    # Send to server
                    send_socket = self.send_sockets[msg_type]
                    send_socket.sendto(serialized_message, (self.server_ip, self.server_port))

            except json.JSONDecodeError:
                print("Error: Invalid JSON format")
            
            except socket.error as e:
                print(f"Socket error: {str(e)}")
            
            except Exception as e:
                print(f"Unexpected client error: {str(e)}")
    
    def decode_msg(self, received_data) -> tuple[dict, str]:
        """Decode the received message data."""
        msg = {}
        msg_type = ''

        if self.encoder_method == "json":
            msg = self.json_decoder.decode(received_data)
        
        elif self.encoder_method == "byte":
            msg_type = MSG.get_msg_type_from_bytes(received_data)

            if msg_type not in self.decoders:
                raise ValueError(f"Unhandled message type during decode. '{msg_type}'")
            
            decoder = self.decoders[msg_type]

            msg = decoder.decode(received_data)

        else:
            raise ValueError("Unrecognized encoder/decoder style.")
        
        return msg, msg_type
    
    def encode_msg(self, msg_data) -> tuple[bytes, str]:
        """Encode the message data."""
        msg = bytes()
        msg_type = ''

        if self.encoder_method == "json":
            msg = self.json_encoder.encode(msg_data)
        
        elif self.encoder_method == "byte":
            msg_type = MSG.get_msg_type_from_dict(msg_data)

            if msg_type not in self.encoders:
                raise ValueError(f"Unhandled message type during encode. '{msg_type}'")
            
            encoder = self.encoders[msg_type]

            msg = encoder.encode(msg_data)

        else:
            raise ValueError("Unrecognized encoder/decoder style.")
        
        return msg, msg_type

    def get_data(self, msg_type):
        """Gets the most recently streamed data from the server based on type."""
        with self.read_lock:
            if 'bsm' in msg_type:
                data = self.bsm_data
            elif 'sim' in msg_type:
                data = self.sim_data
            elif 'spat' in msg_type:
                data = self.spat_data
            else:
                raise ValueError(f'Unrecognized msg_type in get_data. {msg_type}')
            
            return data.copy()
        
    def remove_data(self, msg_type):
        """Remove the stored data that was received from the server."""
        with self.read_lock:
            if 'bsm' in msg_type:
                data = self.bsm_data
            elif 'sim' in msg_type:
                data = self.sim_data
            elif 'spat' in msg_type:
                data = self.spat_data
            else:
                raise ValueError(f'Unrecognized msg_type. {msg_type}')
            
            data.clear()

    def remove_all_data(self):
        """Removes the message data streamed from server."""
        with self.read_lock:
            self.bsm_data.clear()
            self.sim_data.clear()
            self.spat_data.clear()

    def listen_for_replies(self):
        """
        Asynchronously listens for server replies in a separate thread.
        """
        
        recv_socket = next( self.recv_sockets_value_iter )
        print(f'Listener thread started on {recv_socket.getsockname()}')

        while self.running.is_set():
            try:
                # Get data from server
                recv_socket.settimeout(1)  # Prevent blocking indefinitely

                received_data, _ = recv_socket.recvfrom(self.num_recv_bytes)
                
                # Process and save data
                message, msg_type = self.decode_msg(received_data)

                # Check message type
                if 'sim' in msg_type:
                    # Handle sim
                    self.remove_data('sim')
                    message = SIM.fill_and_check_message(message)

                    id = message['id']
                    self.sim_data[id] = message

                    # Handle heartbeat
                    if message['sim_status'] == SIM.SimStatus.WAITING.value:
                        pass

                    elif message['sim_status'] == SIM.SimStatus.OFFLINE.value:
                        if self.simming.is_set():
                            print(f'Simulation {id} reported as OFFLINE.')
                            self.simming.clear()

                    elif message['sim_status'] == SIM.SimStatus.RUNNING.value:
                        if not self.simming.is_set():
                            print(f'Simulation {id} reported as RUNNING.')
                            self.simming.set()

                elif 'bsm' in msg_type:
                    # Handle BSM
                    self.remove_data('bsm') # Reset so that only most recent update is used
                    message = BSM.fill_and_check_message(message)
                    
                    id = message['id']
                    self.bsm_data[id] = message

                elif 'spat' in msg_type:
                    # Handle spat
                    self.remove_data('spat')
                    message = SPAT.fill_and_check_message(message)

                    id = message['id']
                    self.spat_data[id] = message

                else:
                    raise ValueError(f'Unrecognized msg_type during listen. "{msg_type}"')

                if not self.is_connected(): # If not already flagged that we are connected to the server
                    self.connected.set()

            except socket.timeout:
                pass  # No response received, continue listening
            
            except ConnectionResetError as e:
                if self.connected.is_set():
                    print("Connection reset by server.")

                    self.remove_all_data()
                    self.connected.clear()
                
            except ConnectionRefusedError as e:
                print("Connected refused by server.")
            
                self.remove_all_data()
                self.connected.clear()
            
            except socket.error as e:
                print(f"Unhandled socket error occurred: {e}")

                self.remove_all_data()
                self.connected.clear()
            
            except Exception as e:
                print(f"Unhandled client error: {str(e)}")
                
                self.remove_all_data()
                self.connected.clear()

    def is_running(self):
        '''Returns True/False if the client is currently running.'''
        return self.running.is_set()

    def is_connected(self):
        '''Returns True/False if the client is connected/not connected to the server.'''
        return self.connected.is_set()
    
    def is_simming(self):
        '''Returns True/False if the server has responded that the sim is beginning.'''
        return self.simming.is_set()

    def rate(self):
        '''Starts a timer to pause until next update interval'''
        # Calculate Sleep duration and sleep
        self.update_iter += 1
        
        sleep_duration = self.update_interval*self.update_iter - (counter()-self.time_at_start)
        if sleep_duration > 0:
            sleep(sleep_duration) # Control broadcast update rate

        # Error check
        if sleep_duration < 1e-3:
            warnings.warn("Broadcast update too slow to maintain desired update rate")

        # The time when the last update was made
        self.time_at_last_update = counter()


class UDPClientOld:
    """
    A multi-threaded UDP client for sending motion data to a server and receiving updates.

    Parameters:
    - server_ip (str): The IP address of the server to connect to. Defaults to "127.0.0.1".
    - server_port (int): The UDP port number used for communication with the server. Defaults to 12345.
    - framerate (int): The update rate (frames per second) for sending motion data. Defaults to 5.

    Attributes:
    - update_interval (float): The time (seconds) between sending updates.
    - time_at_last_update (float): Timestamp of the last update sent to the server.
    - send_socket (socket.socket): UDP socket used for sending and receiving data.
    - bsm_data (dict): Stores received data from the server.
    - send_lock (threading.Lock): Ensures thread-safe access to client data.
    - read_lock (threading.Lock): Ensures thread-safe access to server data.

    Methods:
    - stop(): Gracefully stops the client, terminating threads and closing the socket.
    - send_update_data(update_data): Updates and sends motion data to the server.
    - get_bsm_data(): Retrieves the latest data received from the server.
    - rate(): Retrieves the latest data received from the server.
    - is_running(): Returns True/False if the client is currently running.
    - is_connected(): Returns True/False if the client is connected to the server.
    """

    def __init__(self, server_ip="127.0.0.1", server_port=12345, framerate=20):
        """Initializes the UDP client."""
        ### Client Settings
        self.server_ip = server_ip
        self.server_port = server_port
        self.framerate = framerate

        self.encoder_method = "byte" # Encoder/decoder method used - "json", "byte"
        
        if self.encoder_method == "byte":
            self.bsm_encoder = ByteEncoder(BSM)
            self.bsm_decoder = copy.deepcopy(self.bsm_encoder)

            self.sim_encoder = ByteEncoder(SIM)
            self.sim_decoder = copy.deepcopy(self.sim_encoder)

            self.spat_encoder = ByteEncoder(SPAT)
            self.spat_decoder = copy.deepcopy(self.spat_encoder)

            self.num_recv_bytes = 1024
        
        elif self.encoder_method == "json":
            self.json_encoder = JsonEncoder()
            self.json_decoder = copy.deepcopy(self.json_encoder)

            self.num_recv_bytes = 1024*8

        else:
            raise ValueError(f"Encoder method unhandled. {self.encoder_method}")

        # Updates
        self.update_interval = 1./framerate  # Time between sending updates
        self.time_at_last_update = counter()  # The time that the last update was sent - will not send a new one until the update interval has passed
        self.time_at_start = counter()
        self.update_iter = 0 # Number of rate updates done

        # Initialize socket
        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Data to/from stream 
        self.bsm_data = {}  # Shared data storage
        self.sim_data = {}
        self.spat_data = {}

        # Threading
        self.running = threading.Event()  # Event flag to stop sending threads

        self.send_lock = threading.Lock()  # Thread safety lock
        self.read_lock = threading.Lock()  # Thread safety lock

        ### Start
        self.connected = threading.Event() # Event Flag to indicate if connected to server
        self.simming = threading.Event() # Event Flag to indicate if the server is simming
        
        self.start()

        print(f'Client started on {self.server_ip}:{self.server_port}')
    
    def __del__(self):
        self.stop()

    def stop(self):
        """Stops the client gracefully."""
        if self.running.is_set():
            # Print
            print("Client is shutting down...")

            # Close threads and close socket
            self.running.clear()

            self.listen_thread.join()

            self.send_socket.close()

    def start(self):
        """
        Starts a separate thread to listen for motion data server responses.
        """
        if not self.running.is_set():
            self.running.set()

            self.listen_thread = threading.Thread(target=self.listen_for_replies, daemon=True)
            self.listen_thread.start()
    
    def send_update_data(self, update_data):
        """Sends serialized data to the server with status enums."""
        if self.running.is_set():
            try:
                # Safely update shared data
                with self.send_lock:
                    serialized_message = self.encode_msg(update_data)
                
                    # Send to server
                    self.send_socket.sendto(serialized_message, (self.server_ip, self.server_port))

            except json.JSONDecodeError:
                print("Error: Invalid JSON format")
            
            except socket.error as e:
                print(f"Socket error: {str(e)}")
            
            except Exception as e:
                print(f"Unexpected client error: {str(e)}")
    
    def decode_msg(self, received_data) -> dict:
        """Decode the received message data."""
        msg = {}

        if self.encoder_method == "json":
            msg = self.json_decoder.decode(received_data)
        
        elif self.encoder_method == "byte":
            msg_type = MSG.get_msg_type_from_bytes(received_data)

            if "bsm" in msg_type:
                decoder = self.bsm_decoder

            elif "sim" in msg_type:
                decoder = self.sim_decoder
            
            elif "spat" in msg_type:
                decoder = self.spat_decoder
            
            else:
                raise ValueError(f"Unhandled message type during decode. '{msg_type}'")

            msg = decoder.decode(received_data)

        else:
            raise ValueError("Unrecognized encoder/decoder style.")
        
        return msg
    
    def encode_msg(self, msg_data) -> bytes:
        """Encode the message data."""
        msg = bytes()

        if self.encoder_method == "json":
            msg = self.json_encoder.encode(msg_data)
        
        elif self.encoder_method == "byte":
            msg_type = MSG.get_msg_type_from_dict(msg_data)

            if "bsm" in msg_type:
                encoder = self.bsm_encoder

            elif "sim" in msg_type:
                encoder = self.sim_encoder
            
            elif "spat" in msg_type:
                encoder = self.spat_encoder

            else:
                raise ValueError(f"Unhandled message type during encode. '{msg_type}'")

            msg = encoder.encode(msg_data)

        else:
            raise ValueError("Unrecognized encoder/decoder style.")
        
        return msg

    def get_data(self, msg_type):
        """Gets the most recently streamed data from the server based on type."""
        with self.read_lock:
            if 'bsm' in msg_type:
                data = self.bsm_data
            elif 'sim' in msg_type:
                data = self.sim_data
            elif 'spat' in msg_type:
                data = self.spat_data
            else:
                raise ValueError(f'Unrecognized msg_type in get_data. {msg_type}')
            
            return data.copy()
        
    def remove_data(self, msg_type):
        """Remove the stored data that was received from the server."""
        with self.read_lock:
            if 'bsm' in msg_type:
                data = self.bsm_data
            elif 'sim' in msg_type:
                data = self.sim_data
            elif 'spat' in msg_type:
                data = self.spat_data
            else:
                raise ValueError(f'Unrecognized msg_type. {msg_type}')
            
            data.clear()

    def remove_all_data(self):
        """Removes the message data streamed from server."""
        with self.read_lock:
            self.bsm_data.clear()
            self.sim_data.clear()
            self.spat_data.clear()

    def listen_for_replies(self):
        """
        Asynchronously listens for server replies in a separate thread.
        """
        while self.running.is_set():
            try:
                # Get data from server
                self.send_socket.settimeout(1)  # Prevent blocking indefinitely

                received_data, _ = self.send_socket.recvfrom(self.num_recv_bytes)
                
                # Process and save data
                message = self.decode_msg(received_data)

                # Check the message for required fields and fill in appropriate optional data
                msg_type = MSG.get_msg_type_from_dict(message)

                # Check message type
                if 'sim' in msg_type:
                    # Handle sim
                    self.remove_data('sim')
                    message = SIM.fill_and_check_message(message)

                    id = message['id']
                    self.sim_data[id] = message

                    # Handle heartbeat
                    if message['sim_status'] == SIM.SimStatus.WAITING.value:
                        pass

                    elif message['sim_status'] == SIM.SimStatus.OFFLINE.value:
                        if self.simming.is_set():
                            print(f'Simulation {id} reported as OFFLINE.')
                            self.simming.clear()

                    elif message['sim_status'] == SIM.SimStatus.RUNNING.value:
                        if not self.simming.is_set():
                            print(f'Simulation {id} reported as RUNNING.')
                            self.simming.set()

                elif 'bsm' in msg_type:
                    # Handle BSM
                    self.remove_data('bsm') # Reset so that only most recent update is used
                    message = BSM.fill_and_check_message(message)
                    
                    id = message['id']
                    self.bsm_data[id] = message

                elif 'spat' in msg_type:
                    # Handle spat
                    pass

                else:
                    raise ValueError(f'Unrecognized msg_type during listen. -{msg_type}-')

                if not self.is_connected(): # If not already flagged that we are connected to the server
                    self.connected.set()

            except socket.timeout:
                pass  # No response received, continue listening
            
            except ConnectionResetError as e:
                if self.connected.is_set():
                    print("Connection reset by server.")

                    self.remove_all_data()
                    self.connected.clear()
                
            except ConnectionRefusedError as e:
                print("Connected refused by server.")
            
                self.remove_all_data()
                self.connected.clear()
            
            except socket.error as e:
                print(f"Unhandled socket error occurred: {e}")

                self.remove_all_data()
                self.connected.clear()
            
            except Exception as e:
                print(f"Unhandled client error: {str(e)}")
                
                self.remove_all_data()
                self.connected.clear()

    def is_running(self):
        '''Returns True/False if the client is currently running.'''
        return self.running.is_set()

    def is_connected(self):
        '''Returns True/False if the client is connected/not connected to the server.'''
        return self.connected.is_set()
    
    def is_simming(self):
        '''Returns True/False if the server has responded that the sim is beginning.'''
        return self.simming.is_set()

    def rate(self):
        '''Starts a timer to pause until next update interval'''
        # Calculate Sleep duration and sleep
        self.update_iter += 1
        
        sleep_duration = self.update_interval*self.update_iter - (counter()-self.time_at_start)
        if sleep_duration > 0:
            sleep(sleep_duration) # Control broadcast update rate

        # Error check
        if sleep_duration < 1e-3:
            warnings.warn("Broadcast update too slow to maintain desired update rate")

        # The time when the last update was made
        self.time_at_last_update = counter()

### Run the client
if __name__ == "__main__":
    # Handle input arguments
    parser = argparse.ArgumentParser(description="UDP Client options for sending and Receiving Motion Data")
    parsers.client.register_parser(parser)

    args = parser.parse_args()
    
    # Run client to send/listen for server messages
    client = UDPClient(server_ip=args.server_ip, server_port=args.server_port, framerate=args.framerate)

    try:
        t_start = counter()
        
        while counter() - t_start < 20:
            # Pack data
            client_data = BSM.make_bsm(str(1), 10.5, 20.3, 0.0, 0.0, 0.0, 0.0, BSM.Error.OK.value,
                        length=4.2, width=1.8, braking_status=BSM.BrakingStatus.NOT_APPLIED.value)

            # Send data
            client.send_update_data(client_data)

            # Get any server data
            bsm_data = client.get_data('bsm')
            if bsm_data:
                for bsm in bsm_data.values():
                    print(bsm['id'], bsm['current_utc_time'], bsm['rel_long_gap'], bsm['speed'])

            # Waiting til sending next needed update
            client.rate()

    except KeyboardInterrupt:
        pass

    # Cleanup
    client.stop()