#!/usr/bin/env python3

#######################################
### Tyler Ard                       ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

import socket
import json
import threading
import argparse
import logging
import os
import sys
from math import pi
from time import perf_counter as counter
from time import sleep
import copy

import parsers.server
from src.messages import BSM, SPAT, MSG, SIM
from src.encoders import ByteEncoder, JsonEncoder

def _is_dict_of_dicts(d: dict) -> bool:
    return isinstance(d, dict) and all(isinstance(v, dict) for v in d.values())

class UDPServer:
    """
    A multi-threaded UDP server for handling motion data streaming and client management.

    Parameters:
    - server_ip (str): The IP address the server binds to. Defaults to "127.0.0.1".
    - server_port (int): The port on which the server listens for UDP packets. Defaults to 12345.
    - framerate (int): The update rate (frames per second) for broadcasting data. Defaults to 20.

    Attributes:
    - update_interval (float): The time (seconds) between sending updates.
    - time_at_last_update (float): Timestamp of the last broadcast update.
    - inactivity_timeout (float): Maximum allowed inactivity duration before removing a client.
    - inactivity_interval (float): Time interval for checking inactive clients.
    - time_at_last_inactivity_check (float): Timestamp of the last inactivity check.
    - server_socket (socket.socket): UDP socket for communication.
    - all_bsm_data (dict): Stores received motion data indexed by client addresses.
    - client_last_active_time (dict): Tracks last active timestamps for each client.
    - running (threading.Event): Flag to indicate if the server is running.
    - client_lock (threading.Lock): Ensures thread-safe client management.
    - message_lock (threading.Lock): Ensures thread-safe message storage.

    Methods:
    - stop(): Gracefully shuts down the server, stopping threads and closing the socket.
    - start_listening(): Starts threads for handling incoming data and checking client activity.
    - add_data(message, client_ip, type): Stores validated motion data from a client.
    - get_data(client_ip, type): Retrieves the most recent motion data from a specific client.
    - get_all_data(type): Retrieves all stored motion data from active clients.
    - remove_data(client_ip, type): Removes stored motion data from a specific client.
    - broadcast_update_to_all(server_data): Sends updates to all currently active clients.
    """

    def __init__(self, server_ip="127.0.0.1", server_port=12345, framerate=20, is_debugging=False):
        """Initializes the UDP server with logging enabled."""
        ### Server Settings
        self.server_ip = server_ip
        self.server_port = server_port
        self.framerate = framerate

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
            self.num_recv_bytes = 1024*8

        # Msg Type send ports
        # The server will expect to only send certain message types across a particular port for each unique IP address
        self.msg_type_send_port = {
            'bsm': 10001,
            'sim': 10004,
            'spat': 10007,
        }

        # Updates
        self.update_interval = 1./framerate  # [s] Time between sending updates
        self.time_at_last_update = counter()  # [s] The timestamp that the last update was sent - will not send a new one until the update interval has passed
        self.time_at_start = counter()
        self.update_iter = 0 # Number of rate updates done

        # Client timeouts
        self.inactivity_timeout = 3. # [s] Duration allowed between client updates before removing them
        self.inactivity_interval = 1. # [s] Time between checking for inactivity
        self.time_at_last_inactivity_check = counter()  # The timestamp that the last inactivity purge was checked - 

        ### Initialize socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.bind((self.server_ip, self.server_port))

        # Datas from stream
        self.all_bsm_data = {}  # Shared data storage
        self.all_sim_data = {}  # Shared data storage
        self.all_spat_data = {}  # Shared data storage

        # Set client activity checker and server running flag
        self.client_last_active_time = {}
        self.running = threading.Event()
        
        self.client_lock = threading.Lock() # For thread safety
        self.message_lock = threading.Lock() # For thread safety

        self.num_clients = 0 # Number of active clients - updated in inactivity check

        ### Debugging
        self.is_debugging = is_debugging

        ### Start
        self.start()
        
        ### Configure logging
        self._set_logger()

    def _set_logger(self):
        # Make logging directory
        log_directory = 'logs'
        os.makedirs(log_directory, exist_ok=True) # Create the directory if it doesn't exist

        # Set up logger
        level = logging.INFO if not self.is_debugging else logging.DEBUG
        
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel( level )
        file_handler = logging.FileHandler("logs/udp_server.log")
        file_handler.setLevel( level )
        
        logging.basicConfig(
            level=level,
            format="%(asctime)s - %(levelname)s - %(message)s",
            handlers=(file_handler, console_handler)
        )

        # Report successful server startup
        logging.info(f"Server started on {self.server_ip}:{self.server_port}")


    def __del__(self):
        """Stops the server gracefully."""
        self.stop()

    def stop(self):
        """Stops the server gracefully."""
        if self.running.is_set():
            # Log
            logging.info("Server is shutting down...")

            # Close threads and close socket
            self.running.clear()
            
            self.listen_thread.join()
            self.inactivity_thread.join()

            self.server_socket.close()

    def start(self):
        """
        Starts the multi-threaded motion data streaming with asynchronous reply handling.
        """
        if not self.running.is_set():
            # Start threads
            self.running.set()

            self.listen_thread = threading.Thread(target=self.handle_clients, daemon=True)
            self.listen_thread.start()

            self.inactivity_thread = threading.Thread(target=self.check_for_inactive, daemon=True)
            self.inactivity_thread.start()

    def handle_clients(self):
        """Handles incoming messages and starts threads for concurrent clients."""
        # Run
        while self.running.is_set():
            try:
                self.server_socket.settimeout(1)

                received_data, recv_address = self.server_socket.recvfrom(self.num_recv_bytes)
                
                threading.Thread(target=self.process_message, args=(received_data, recv_address)).start()
                
            except socket.timeout:
                pass

            except ConnectionResetError as e:
                logging.info("Connection reset by peer.")

            except ConnectionRefusedError as e:
                logging.warning("Connected refused by peer.")

            except socket.error as e:
                logging.error(f"Unhandled socket error occurred: {e}")

            except Exception as e:
                logging.error(f"Unhandled error: {str(e)}")

    def process_message(self, received_data, recv_address):
        """Processes received motion data, validates it, applies defaults and saves the data."""
        ### Try to process message
        try:
            logging.debug(f"Received {len(received_data)} bytes from {recv_address}.")

            # Separate into IP and Port
            client_ip = recv_address[0]
            client_port = recv_address[1]

            # Write new activity time
            with self.client_lock:
                self.client_last_active_time[client_ip] = counter()
            
            # Process and save data
            message, msg_type = self.decode_msg(received_data)

            # Check message type
            if 'sim' in msg_type:
                # Handle heartbeat
                message = SIM.fill_and_check_message(message)

                self.add_data(message, client_ip, 'sim')

            elif 'bsm' in msg_type:
                # Handle BSM
                message = BSM.fill_and_check_message(message)

                self.add_data(message, client_ip, 'bsm')

            elif 'spat' in msg_type:
                # Handle spat
                message = SPAT.fill_and_check_message(message)
                
                self.add_data(message, client_ip, 'spat')

            else:
                raise ValueError(f'Unrecognized msg_type during listen. "{msg_type}"')

        except json.JSONDecodeError:
            logging.warning(f"JSON decode error from {recv_address}")

        except ValueError as e:
            logging.warning(f"Validation error from {recv_address}: {str(e)}")

        except Exception as e:
            logging.error(f"Unexpected error processing message from {recv_address}: {str(e)}")

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
    
    def add_data(self, message, client_ip, msg_type):
        """Adds the received and processed data to storage based on type."""
        with self.message_lock:
            if 'bsm' in msg_type:
                data = self.all_bsm_data
            elif 'sim' in msg_type:
                data = self.all_sim_data
            elif 'spat' in msg_type:
                data = self.all_spat_data
            else:
                raise ValueError(f'Unrecognized msg_type. "{msg_type}"')
            
            data[client_ip] = message

    def get_data(self, client_ip, msg_type) -> dict:
        """Gets the most recently received data from client based on type."""
        with self.message_lock:
            if 'bsm' in msg_type:
                data = self.all_bsm_data
            elif 'sim' in msg_type:
                data = self.all_sim_data
            elif 'spat' in msg_type:
                data = self.all_spat_data
            else:
                raise ValueError(f'Unrecognized msg_type. "{msg_type}"')
            
            return data[client_ip] if client_ip in data else {}

    def get_all_data(self, msg_type) -> dict:
        """Gets the most recently received data from all clients based on type."""
        with self.message_lock:
            if 'bsm' in msg_type:
                data = self.all_bsm_data
            elif 'sim' in msg_type:
                data = self.all_sim_data
            elif 'spat' in msg_type:
                data = self.all_spat_data
            else:
                raise ValueError(f'Unrecognized msg_type. "{msg_type}"')
            
            return data if len(data)>0 else {}
        
    def remove_data(self, client_ip):
        """Removes the message data from client."""
        with self.message_lock:
            if client_ip in self.all_bsm_data:
                del self.all_bsm_data[client_ip]
            elif client_ip in self.all_sim_data:
                del self.all_sim_data[client_ip]
            elif client_ip in self.all_spat_data:
                del self.all_spat_data[client_ip]
    
    def get_num_clients(self):
        """Gets the current number of active clients."""
        with self.client_lock:
            return self.num_clients
        
    def make_client_inactive(self, client_ip):
        """Flags the client as inactive."""
        if client_ip is None:
            return
        
        with self.client_lock:
            logging.warning("  Marking client as inactive.")
            self.client_last_active_time[client_ip] = -1
        
    def check_for_inactive(self):
        """Checks for inactive clients, logs removals, and removes the client from set."""
        while self.running.is_set():
            with self.client_lock:
                self.num_clients = 0

                for client_ip, last_active in list(self.client_last_active_time.items()):
                    if self.time_at_last_update - last_active > self.inactivity_timeout:
                        # Remove client_ip data which may be in one of several locations
                        del self.client_last_active_time[client_ip]
                        self.remove_data(client_ip)

                        logging.warning(f"Client {client_ip} has been detected as inactive and is removed")
                    
                    else:
                        self.num_clients += 1

            # Pause til next check
            self.start_inactive_timer()

    def start_inactive_timer(self):
        '''Starts a timer to pause until next inactivity check'''
        # Calculate Sleep duration and sleep
        broadcast_duration = counter() - self.time_at_last_inactivity_check # How long the update broadcast took
        sleep_duration = max(0, self.inactivity_interval - broadcast_duration)

        sleep(sleep_duration)  # Control broadcast update rate

        # Error check
        if sleep_duration < 1e-3:
            logging.warning("Inactivity check was too slow to maintain desired update rate")

        # The time when the last update was made
        self.time_at_last_inactivity_check = counter()

    def set_client_port(self, msg_type, client_ip):
        """Modifies the client port based on msg_type."""
        # Check input arguments
        if not isinstance(client_ip, str):
            raise ValueError(f"Expected type of client passed to set client port to be a str. {client_ip}")
        
        if msg_type not in self.msg_type_send_port:
            raise ValueError(f"Expected msg_type to be found in msg_type_send_port dictionary. '{msg_type}' passed.")

        # Make tuple for send_addr
        port = self.msg_type_send_port[msg_type]
        
        send_addr = (client_ip, port)
        
        return send_addr
    
    def broadcast_update_to_all(self, server_data):
        """Sends an update message to all currently active clients."""
        # Check inputs
        if _is_dict_of_dicts(server_data):
            raise ValueError('Server data message contains a dict in one of its value fields. Is this following a format from messages.py?')
        
        # Send
        if self.running.is_set():
            # Get current active clients
            with self.client_lock:
                active_clients = [client_ip for client_ip, last_active in self.client_last_active_time.items()
                                  if self.time_at_last_update - last_active <= self.inactivity_timeout]

            if active_clients:
                # Serialize and send
                serialized_message, msg_type = self.encode_msg(server_data)

                for client_ip in active_clients:
                    send_addr = self.set_client_port(msg_type, client_ip)
                    sent_bytes = self.server_socket.sendto(serialized_message, send_addr)

                    logging.debug(f"  Sent {sent_bytes} bytes {msg_type} update to {send_addr}")

    def send_update(self, server_data, client_ip):
        """Sends an update message to selected active client_ip."""
        # Check inputs
        if _is_dict_of_dicts(server_data):
            raise ValueError('Server data message contains a dict in one of its value fields. Is this following a format from messages.py?')
        
        # Send
        if self.running.is_set():
            is_active = False

            # Check client activity
            with self.client_lock:
                if client_ip in self.client_last_active_time:
                    last_active = self.client_last_active_time[client_ip]
                    is_active = self.time_at_last_update - last_active <= self.inactivity_timeout

            # Serialize and send
            serialized_message, msg_type = self.encode_msg(server_data)

            if is_active:
                send_addr = self.set_client_port(msg_type, client_ip)
                sent_bytes = self.server_socket.sendto(serialized_message, send_addr)

                logging.debug(f"  Sent {sent_bytes} bytes {msg_type} update to {send_addr}")
                
            else:
                logging.debug(f"  Did not send {msg_type} update to {client_ip} - inactivity detected")

    def rate(self):
        '''Starts a timer to pause until next update interval'''
        # Calculate Sleep duration and sleep
        self.update_iter += 1
        
        sleep_duration = self.update_interval*self.update_iter - (counter()-self.time_at_start)
        if sleep_duration > 0:
            sleep(sleep_duration) # Control broadcast update rate

        # Error check
        if sleep_duration < 1e-3:
            logging.warning("Broadcast update too slow to maintain desired update rate")

        # The time when the last update was made
        self.time_at_last_update = counter()

    def is_running(self):
        '''Returns True/False if the server is currently running.'''
        return self.running.is_set()
    
    def is_connected(self, expected_num_clients):
        '''Returns True/False if the server is currently fully connected with the expected number of clients and the clients are all ready.'''
    
        # Number of clients check
        if expected_num_clients > 0:
            n = self.get_num_clients()
            return n == expected_num_clients
        else:
            return True

    def is_simming(self):
        """Checks the sim message data from every client and if any client reports as NOT_READY or OFFLINE then should not be simming currently."""
        sim_data = self.get_all_data('sim')

        if sim_data:
            for client_ip, data in sim_data.items():
                sim_status = data['sim_status']
                if sim_status == SIM.SimStatus.NOT_READY.value or sim_status == SIM.SimStatus.OFFLINE.value or sim_status == SIM.SimStatus.NOT_READY.value or sim_status == SIM.SimStatus.RESET.value:
                    return False
            
        else:
            return False
                
        return True
    
    def is_resetting(self):
        """Checks the sim message data from every client and if any client reports as RESET then should begin a reset routine."""
        sim_data = self.get_all_data('sim')

        if sim_data:
            for client_ip, data in sim_data.items():
                sim_status = data['sim_status']
                if sim_status == SIM.SimStatus.RESET.value:
                    return True
            
        else:
            return False
                
        return False

### Run the server
if __name__ == "__main__":
    # Handle input arguments
    parser = argparse.ArgumentParser(description="Multi-Client UDP Server for Motion Data with Multiple Response Entries")
    parsers.server.register_parser(parser)

    args = parser.parse_args()

    # Run server to listen for multiple client messages
    server = UDPServer(server_ip=args.server_ip, server_port=args.server_port, framerate=args.framerate)
    
    # Send asynchronous updates
    try:
        t_start = counter()
        while counter()-t_start < 15:
            # Get client data
            bsm_data = server.get_all_data('bsm')
            if bsm_data:
                for bsm in bsm_data.values():
                    print(bsm['current_utc_time'])
                    print(bsm['speed'])
            
            # Get server data update for clients
            server_data = {str(i): BSM.make_bsm(str(i), 10.5, 20.3, 5.6, 0.8, 0.5*pi, 0.0, 
                    rel_long_gap=10., rel_lat_gap=0)
                for i in range(0, 2) # Surrounding vehicles
            }

            for id, message in server_data.items():
                server.broadcast_update_to_all(message)

            # Pause til next update
            server.rate()

    except KeyboardInterrupt:
        pass

    # Cleanup
    server.stop()