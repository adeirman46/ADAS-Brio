
# import serial
# import time
# import csv
# import struct
# import socket
# from typing import Optional, Tuple
# from read_can import CANSpeedHandler
# from threading import Thread, Event

# class MIMOSpeedController:
#     def __init__(self, serial_port: str = '/dev/ttyACM0', baud_rate: int = 9600):
#         """Initialize MIMO speed controller with serial connection and CAN interface."""
#         self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
#         time.sleep(2)  # Wait for Arduino to initialize
        
#         self.can_handler = CANSpeedHandler()
#         self.start_time = time.time()
#         self.sample_time = 0.02  # 20ms (50Hz) sampling rate
        
#         # Initialize variables
#         self.current_distance = float('inf')
#         self.stop_event = Event()
#         self.last_control_time = time.time()
#         self.log_file = None
#         self.csv_writer = None
#         self.test_counter = 1
        
#         # Initialize sockets
#         self.init_sockets()
        
#         # Initialize logging
#         self.init_logging()
        
#         # Start receiver thread
#         self.distance_thread = Thread(target=self._receive_distance)
#         self.distance_thread.daemon = True
#         self.distance_thread.start()

#     def init_sockets(self):
#         """Initialize all UDP sockets for communication."""
#         try:
#             # Socket for receiving distance data
#             self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#             self.socket.bind(('localhost', 12345))
#             self.socket.settimeout(0.1)

#             # Socket for sending data to GUI
#             self.gui_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#             self.control_address = ('localhost', 12346)  # Speed/brake data

#         except Exception as e:
#             print(f"Error initializing sockets: {e}")
#             raise

#     def init_logging(self):
#         """Initialize log file for the entire session."""
#         try:
#             session_timestamp = time.strftime("%Y%m%d-%H%M%S")
#             self.log_file = open(f'speed_control_session_{session_timestamp}.csv', 'w', newline='')
#             self.csv_writer = csv.writer(self.log_file)
#             self.csv_writer.writerow(['Session_Time', 'Test_Time', 'Desired_Speed', 'Actual_Speed', 
#                                     'Throttle_Output', 'Brake_Output', 'Distance', 'Test_Number'])
#             print(f"Logging initialized with file: speed_control_session_{session_timestamp}.csv")
#         except Exception as e:
#             print(f"Error initializing logging: {e}")
#             raise

#     def _receive_distance(self):
#         """Background thread to receive distance data from vision detector."""
#         while not self.stop_event.is_set():
#             try:
#                 data, _ = self.socket.recvfrom(1024)
#                 self.current_distance = struct.unpack('f', data)[0]
#             except socket.timeout:
#                 continue
#             except Exception as e:
#                 print(f"Error receiving distance: {e}")

#     def send_to_gui(self, actual_speed: float, brake_percentage: float):
#         """Send control data to GUI."""
#         try:
#             data = struct.pack('ff', actual_speed, brake_percentage)
#             self.gui_socket.sendto(data, self.control_address)
#         except Exception as e:
#             print(f"Error sending data to GUI: {e}")

#     def log_data(self, desired_speed: float, actual_speed: float, 
#                  control_outputs: Optional[Tuple[float, float]], test_start_time: float):
#         """Log time, speeds, control outputs, and distance to CSV."""
#         try:
#             session_time = time.time() - self.start_time
#             test_time = time.time() - test_start_time
            
#             row = [
#                 f"{session_time:.3f}",
#                 f"{test_time:.3f}",
#                 f"{desired_speed:.2f}",
#                 f"{actual_speed:.2f}"
#             ]
            
#             if control_outputs:
#                 row.extend([f"{control_outputs[0]:.2f}", f"{control_outputs[1]:.2f}"])
#             else:
#                 row.extend(["NA", "NA"])
                
#             row.extend([f"{self.current_distance:.2f}", self.test_counter])
#             self.csv_writer.writerow(row)
#             self.log_file.flush()
#         except Exception as e:
#             print(f"Error logging data: {e}")

#     def get_safe_distances(self, speed: float) -> Tuple[float, float]:
#         """Calculate safe distances based on current speed."""
#         speed_table = [
#             (0, 1.0, 2.0),     # speed, min distance, safe distance
#             (5, 1.25, 2.5),
#             (10, 2.5, 5.0),
#             (15, 3.75, 7.5)    # Maximum speed 15 km/h
#         ]
        
#         if speed <= speed_table[0][0]:
#             return speed_table[0][1], speed_table[0][2]
#         if speed >= speed_table[-1][0]:
#             return speed_table[-1][1], speed_table[-1][2]
        
#         for i in range(len(speed_table)-1):
#             speed1, min1, safe1 = speed_table[i]
#             speed2, min2, safe2 = speed_table[i+1]
            
#             if speed1 <= speed <= speed2:
#                 ratio = (speed - speed1) / (speed2 - speed1)
#                 min_dist = min1 + ratio * (min2 - min1)
#                 safe_dist = safe1 + ratio * (safe2 - safe1)
#                 return min_dist, safe_dist
                
#         return speed_table[-1][1], speed_table[-1][2]

#     def send_speeds(self, desired_speed: float, actual_speed: float, brake_command: float) -> Optional[Tuple[float, float]]:
#         """Send speeds and brake command to Arduino and get control outputs."""
#         try:
#             self.ser.write(struct.pack('f', desired_speed))
#             self.ser.write(struct.pack('f', actual_speed))
#             self.ser.write(struct.pack('f', brake_command))
            
#             throttle_data = self.ser.read(4)
#             brake_data = self.ser.read(4)
            
#             if len(throttle_data) == 4 and len(brake_data) == 4:
#                 throttle = struct.unpack('f', throttle_data)[0]
#                 brake = struct.unpack('f', brake_data)[0]
#                 return throttle, brake
#             return None
#         except Exception as e:
#             print(f"Serial communication error: {e}")
#             return None

#     def run_continuous(self, cruise_speed: float = 15.0):
#         """Run continuous speed control with distance-based safety."""
#         test_start_time = time.time()
#         next_sample_time = time.time()
        
#         MAX_SPEED = 15.0  # Maximum allowed speed
#         MAX_DETECTION_RANGE = 20.0  # Maximum detection range
#         cruise_speed = min(cruise_speed, MAX_SPEED)
        
#         try:
#             print(f"\nRunning ADAS speed control (cruise speed: {cruise_speed} kph)")
#             print("Time(s) | Target(kph) | Actual(kph) | Throttle(%) | Brake(%) | Distance(m)")
            
#             while True:
#                 current_time = time.time()
                
#                 if current_time < next_sample_time:
#                     continue
                
#                 actual_speed = self.can_handler.read_speed()
#                 if actual_speed is None:
#                     continue
                
#                 # Get safety distances
#                 min_distance, safe_distance = self.get_safe_distances(actual_speed)
#                 safe_distance = min(safe_distance, MAX_DETECTION_RANGE)
                
#                 # Initialize control commands
#                 desired_speed = cruise_speed
#                 brake_command = 0.0
                
#                 # Determine control based on distance
#                 if self.current_distance > MAX_DETECTION_RANGE:
#                     desired_speed = cruise_speed
#                 elif self.current_distance <= min_distance:
#                     desired_speed = 0.0
#                     brake_command = 100.0
#                 elif self.current_distance <= safe_distance:
#                     distance_ratio = (self.current_distance - min_distance) / (safe_distance - min_distance)
#                     desired_speed = cruise_speed * distance_ratio
#                     brake_command = 70.0 * (1 - distance_ratio)
                
#                 # Apply control
#                 control_outputs = self.send_speeds(desired_speed, actual_speed, brake_command)
                
#                 if control_outputs:
#                     throttle, brake = control_outputs
#                     self.send_to_gui(actual_speed, brake)
#                     self.log_data(desired_speed, actual_speed, control_outputs, test_start_time)
                    
#                     # Print status
#                     print(f"\r{current_time - test_start_time:.1f} | {desired_speed:.1f} | "
#                           f"{actual_speed:.1f} | {throttle:.1f} | {brake:.1f} | "
#                           f"{self.current_distance:.1f}", end="")
                
#                 next_sample_time = current_time + self.sample_time
                
#         except KeyboardInterrupt:
#             print("\nTest interrupted by user")
#         except Exception as e:
#             print(f"\nError during test: {e}")
#             raise

#     def cleanup(self):
#         """Clean up resources."""
#         self.stop_event.set()
#         if self.distance_thread.is_alive():
#             self.distance_thread.join()
#         if self.log_file:
#             self.log_file.close()
#         self.ser.close()
#         self.can_handler.close()
#         self.socket.close()
#         self.gui_socket.close()

# if __name__ == "__main__":
#     try:
#         print("Distance-Aware MIMO Speed Controller")
#         print("-----------------------------------")
#         print("Press Ctrl+C to stop the controller")
        
#         controller = MIMOSpeedController()
#         controller.run_continuous()
#     except Exception as e:
#         print(f"Error: {e}")
#     finally:
#         if 'controller' in locals():
#             controller.cleanup()
#         print("Cleanup completed.")


import serial
import time
import csv
import struct
import socket
from typing import Optional, Tuple
from read_can import CANSpeedHandler
from threading import Thread, Event

class MIMOSpeedController:
    def __init__(self, serial_port: str = '/dev/ttyACM0', baud_rate: int = 9600):
        """Initialize MIMO speed controller with serial connection and CAN interface."""
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        
        self.can_handler = CANSpeedHandler()
        self.start_time = time.time()
        self.sample_time = 0.02  # 20ms (50Hz) sampling rate
        self.log_file = None
        self.csv_writer = None
        
        # Distance-related variables
        self.current_distance = float('inf')
        self.stop_event = Event()
        
        # UDP socket for receiving distance data
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('localhost', 12345))
        self.socket.settimeout(0.1)  # 100ms timeout
        # Add GUI communication socket
        self.gui_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gui_address = ('localhost', 12346)  # New port for GUI
        
        # Initialize logging with session timestamp
        self.init_logging()
        
        # Start distance receiver thread
        self.distance_thread = Thread(target=self._receive_distance)
        self.distance_thread.daemon = True
        self.distance_thread.start()

    def send_to_gui(self, actual_speed: float, brake_percentage: float):
        """Send control data to GUI."""
        try:
            data = struct.pack('ff', actual_speed, brake_percentage)
            self.gui_socket.sendto(data, self.gui_address)
        except Exception as e:
            print(f"Error sending data to GUI: {e}")

    def _receive_distance(self):
        """Background thread to receive distance data from vision detector."""
        while not self.stop_event.is_set():
            try:
                data, _ = self.socket.recvfrom(1024)
                self.current_distance = struct.unpack('f', data)[0]
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving distance: {e}")

    def init_logging(self):
        """Initialize log file for the entire session."""
        if not self.log_file:
            session_timestamp = time.strftime("%Y%m%d-%H%M%S")
            self.log_file = open(f'speed_control_session_{session_timestamp}.csv', 'w', newline='')
            self.csv_writer = csv.writer(self.log_file)
            self.csv_writer.writerow(['Session_Time', 'Test_Time', 'Desired_Speed', 'Actual_Speed', 
                                    'Throttle_Output', 'Brake_Output', 'Distance', 'Test_Number'])
            print(f"Logging initialized with file: speed_control_session_{session_timestamp}.csv")
            self.test_counter = 1

    def log_data(self, desired_speed: float, actual_speed: float, 
                 control_outputs: Optional[Tuple[float, float]], test_start_time: float):
        """Log time, speeds, control outputs, and distance to CSV."""
        session_time = time.time() - self.start_time
        test_time = time.time() - test_start_time
        
        row = [
            f"{session_time:.3f}",
            f"{test_time:.3f}",
            f"{desired_speed:.2f}",
            f"{actual_speed:.2f}"
        ]
        
        if control_outputs:
            row.extend([f"{control_outputs[0]:.2f}", f"{control_outputs[1]:.2f}"])
        else:
            row.extend(["NA", "NA"])
            
        row.extend([f"{self.current_distance:.2f}", self.test_counter])
        self.csv_writer.writerow(row)
        self.log_file.flush()

    def send_speeds(self, desired_speed: float, actual_speed: float) -> Optional[Tuple[float, float]]:
        """Send desired and actual speeds to Arduino and get control outputs."""
        try:
            # Send both speeds
            self.ser.write(struct.pack('f', desired_speed))
            self.ser.write(struct.pack('f', actual_speed))
            
            # Read throttle and brake outputs
            throttle_data = self.ser.read(4)
            brake_data = self.ser.read(4)
            
            if len(throttle_data) == 4 and len(brake_data) == 4:
                throttle = struct.unpack('f', throttle_data)[0]
                brake = struct.unpack('f', brake_data)[0]
                return throttle, brake
            return None
        except Exception as e:
            print(f"Serial communication error: {e}")
            return None
        

    # def run_continuous(self, cruise_speed: float = 15.0):
    #     """
    #     Run continuous speed control with industry-standard ADAS distance management.
    #     Implements Time Headway (THW) based approach with progressive deceleration.
    #     """
    #     test_start_time = time.time()
    #     next_sample_time = time.time()
        
    #     # ADAS control parameters
    #     MIN_FOLLOWING_DISTANCE = 2.0  # meters - absolute minimum safe distance
    #     CRITICAL_TIME_HEADWAY = 1.5   # seconds - critical following time
    #     SAFE_TIME_HEADWAY = 2.0       # seconds - comfortable following time
    #     COMFORT_DECEL = 2.0           # m/s² - comfortable deceleration
    #     EMERGENCY_DECEL = 3.5         # m/s² - emergency deceleration
        
    #     try:
    #         print(f"\nRunning ADAS speed control (cruise speed: {cruise_speed} kph)")
    #         print("Time(s) | Target(kph) | Actual(kph) | Throttle(%) | Brake(%) | Distance(m)")
            
    #         while True:
    #             current_time = time.time()
                
    #             if current_time < next_sample_time:
    #                 continue
                    
    #             actual_speed = self.can_handler.read_speed()
    #             if actual_speed is None:
    #                 continue
                
    #             # Convert speed to m/s for calculations
    #             speed_ms = actual_speed / 3.6
                
    #             # Calculate time headway (THW)
    #             time_headway = (self.current_distance / speed_ms) if speed_ms > 0 else float('inf')
                
    #             # Calculate desired speed based on time headway and distance
    #             if self.current_distance <= MIN_FOLLOWING_DISTANCE:
    #                 # Emergency stop condition
    #                 desired_speed = 0
                
    #             elif time_headway < CRITICAL_TIME_HEADWAY:
    #                 # Critical following distance - progressive deceleration
    #                 decel_factor = max(0, time_headway / CRITICAL_TIME_HEADWAY)
    #                 desired_speed = actual_speed * decel_factor
                
    #             elif time_headway < SAFE_TIME_HEADWAY:
    #                 # Maintain safe following distance with smooth deceleration
    #                 decel_factor = (time_headway - CRITICAL_TIME_HEADWAY) / (SAFE_TIME_HEADWAY - CRITICAL_TIME_HEADWAY)
    #                 target_reduction = (1 - decel_factor) * COMFORT_DECEL
    #                 desired_speed = max(0, actual_speed - target_reduction * 3.6)  # Convert back to km/h
                
    #             else:
    #                 # Safe following distance - maintain cruise speed
    #                 desired_speed = min(cruise_speed, 15.0)  # Limited to 30 km/h for testing
                
    #             # Apply control outputs
    #             control_outputs = self.send_speeds(desired_speed, actual_speed)
                
    #             if control_outputs:
    #                 throttle, brake = control_outputs
    #                 self.send_to_gui(actual_speed, brake)
                
    #             self.log_data(desired_speed, actual_speed, control_outputs, test_start_time)
                
    #             if int(current_time * 2) > int((current_time - self.sample_time) * 2):
    #                 print(f"\r{current_time - test_start_time:.1f} | {desired_speed:.1f} | "
    #                     f"{actual_speed:.1f} | "
    #                     f"{control_outputs[0]:.1f} | {control_outputs[1]:.1f} | "
    #                     f"{self.current_distance:.1f}" if control_outputs else "NA", end="")
                
    #             next_sample_time = current_time + self.sample_time
                
    #     except KeyboardInterrupt:
    #         print("\nTest interrupted by user")
    #     except Exception as e:
    #         print(f"\nError during test: {e}")

    # def run_continuous(self, cruise_speed: float = 15.0):
    #     """
    #     Run continuous speed control with simplified distance-based management.
    #     Speed limits and safe distances scale with current speed.
    #     """
    #     test_start_time = time.time()
    #     next_sample_time = time.time()

    #     # Control parameters
    #     BASE_MIN_DISTANCE = 2.0     # Base minimum distance in meters
    #     BASE_SAFE_DISTANCE = 4.0    # Base safe distance in meters
    #     SPEED_FACTOR = 0.3         # Additional distance per km/h of speed

    #     try:
    #         print(f"\nRunning speed control (cruise speed: {cruise_speed} kph)")
    #         print("Time(s) | Target(kph) | Actual(kph) | Throttle(%) | Brake(%) | Distance(m)")

    #         while True:
    #             current_time = time.time()

    #             if current_time < next_sample_time:
    #                 continue

    #             actual_speed = self.can_handler.read_speed()
    #             if actual_speed is None:
    #                 continue

    #             # Calculate dynamic distance thresholds based on current speed
    #             min_distance = BASE_MIN_DISTANCE + (actual_speed * SPEED_FACTOR)
    #             safe_distance = BASE_SAFE_DISTANCE + (actual_speed * SPEED_FACTOR)

    #             # Determine desired speed based on current distance
    #             if self.current_distance <= min_distance:
    #                 # Emergency stop if too close
    #                 desired_speed = 0
                
    #             elif self.current_distance <= safe_distance:
    #                 # Linear speed reduction between min and safe distance
    #                 distance_factor = (self.current_distance - min_distance) / (safe_distance - min_distance)
    #                 desired_speed = actual_speed * distance_factor
                
    #             else:
    #                 # Maintain cruise speed when distance is safe
    #                 desired_speed = min(cruise_speed, 15.0)

    #             # Apply control outputs
    #             control_outputs = self.send_speeds(desired_speed, actual_speed)

    #             if control_outputs:
    #                 throttle, brake = control_outputs
    #                 self.send_to_gui(actual_speed, brake)

    #             self.log_data(desired_speed, actual_speed, control_outputs, test_start_time)

    #             if int(current_time * 2) > int((current_time - self.sample_time) * 2):
    #                 print(f"\r{current_time - test_start_time:.1f} | {desired_speed:.1f} | "
    #                     f"{actual_speed:.1f} | "
    #                     f"{control_outputs[0]:.1f} | {control_outputs[1]:.1f} | "
    #                     f"{self.current_distance:.1f}" if control_outputs else "NA", end="")

    #             next_sample_time = current_time + self.sample_time

    #     except KeyboardInterrupt:
    #         print("\nTest interrupted by user")
    #     except Exception as e:
    #         print(f"\nError during test: {e}")

    def validate_distance(current: float, history: list, max_change: float = 1.0) -> float:
        """
        Validate distance measurement against history.
        Args:
            current: Current distance measurement
            history: List of previous valid measurements
            max_change: Maximum allowed change between measurements
        Returns:
            Validated distance measurement
        """
        if not history:
            return current
        last_valid = history[-1]
        if abs(current - last_valid) > max_change:
            return last_valid
        return current

    def run_continuous(self, cruise_speed: float = 15.0):
        """
        DTC-based speed control system. Sends desired speeds to Arduino
        which handles the actual throttle/brake control.
        """
        test_start_time = time.time()
        next_sample_time = time.time()

        # DTC Zones (in meters)
        CRITICAL_DTC = 2.0 + 1.0     # Emergency braking zone
        WARNING_DTC = 3.0 + 1.0      # Strong deceleration zone
        PREPARE_DTC = 4.0 + 1.0     # Initial deceleration zone
        ACC_DTC = 6.0 + 1.0        # ACC control zone
        
        # Validation parameters
        distance_buffer = []    # Rolling buffer for distance validation
        BUFFER_SIZE = 5        # Number of measurements to keep
        VALID_CHANGE = 1.0     # Maximum valid change between measurements
        
        # Emergency state tracking
        emergency_active = False
        emergency_start_time = None
        consistent_detections = 0
        REQUIRED_DETECTIONS = 3  # Number of consistent detections needed
        MIN_EMERGENCY_TIME = 2.0 # Minimum time to hold emergency state

        try:
            print("\nRunning DTC-based speed control")
            print("Time(s) | Target(km/h) | Current(km/h) | DTC(m) | State")

            while True:
                current_time = time.time()
                if current_time < next_sample_time:
                    continue

                actual_speed = self.can_handler.read_speed()
                if actual_speed is None:
                    continue

                # Validate distance measurement
                current_distance = self.validate_distance(self.current_distance, distance_buffer, VALID_CHANGE)
                
                # Update distance history
                distance_buffer.append(current_distance)
                if len(distance_buffer) > BUFFER_SIZE:
                    distance_buffer.pop(0)

                # Emergency detection logic
                if current_distance <= CRITICAL_DTC:
                    consistent_detections += 1
                    if consistent_detections >= REQUIRED_DETECTIONS and not emergency_active:
                        emergency_active = True
                        emergency_start_time = current_time
                else:
                    consistent_detections = max(0, consistent_detections - 1)

                # Emergency state handling
                if emergency_active:
                    time_in_emergency = current_time - emergency_start_time
                    # Check if we can exit emergency state
                    if (time_in_emergency >= MIN_EMERGENCY_TIME and 
                        current_distance > PREPARE_DTC and 
                        actual_speed < 2.0):
                        emergency_active = False
                        emergency_start_time = None

                # Determine desired speed based on DTC zones
                if emergency_active:
                    desired_speed = 0.0  # Emergency - request full stop
                elif current_distance <= WARNING_DTC:
                    # Progressive deceleration in warning zone
                    factor = (current_distance - CRITICAL_DTC) / (WARNING_DTC - CRITICAL_DTC)
                    desired_speed = actual_speed * max(0, min(factor, 1)) * 0.5  # 50% speed reduction
                elif current_distance <= PREPARE_DTC:
                    # Mild deceleration in preparation zone
                    factor = (current_distance - WARNING_DTC) / (PREPARE_DTC - WARNING_DTC)
                    desired_speed = actual_speed * max(0.5, min(factor, 1))
                elif current_distance <= ACC_DTC:
                    # ACC control - maintain safe following speed
                    factor = (current_distance - PREPARE_DTC) / (ACC_DTC - PREPARE_DTC)
                    desired_speed = cruise_speed * max(0, min(factor, 1))
                else:
                    # Normal cruise control
                    desired_speed = min(cruise_speed, 15.0)

                # Send speed commands to Arduino
                control_outputs = self.send_speeds(desired_speed, actual_speed)

                # Update GUI and log data
                if control_outputs:
                    self.send_to_gui(actual_speed, 100 if emergency_active else 0)
                    self.log_data(desired_speed, actual_speed, control_outputs, test_start_time)

                    state = ("EMERGENCY" if emergency_active else
                            "WARNING" if current_distance <= WARNING_DTC else
                            "PREPARE" if current_distance <= PREPARE_DTC else
                            "ACC" if current_distance <= ACC_DTC else "CRUISE")

                    print(f"\r{current_time - test_start_time:.1f} | {desired_speed:.1f} | "
                        f"{actual_speed:.1f} | {current_distance:.1f} | {state}", end="")

                next_sample_time = current_time + self.sample_time

        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        except Exception as e:
            print(f"\nError during test: {e}")

    def _get_status_message(self, speed_stable: bool, speed_error: float, distance: float) -> str:
        """Generate status message based on current system state."""
        if distance <= 2.0:
            return "EMERGENCY STOP"
        elif distance <= 5.0:
            return "CRITICAL DISTANCE"
        elif speed_stable:
            return "STABLE"
        elif speed_error > 5.0:
            return "ADJUSTING SPEED"
        else:
            return "NORMAL"

    def cleanup(self):
        """Clean up resources."""
        self.stop_event.set()
        if self.distance_thread.is_alive():
            self.distance_thread.join()
        if self.log_file:
            self.log_file.close()
        self.ser.close()
        self.can_handler.close()
        self.socket.close()
        self.gui_socket.close()  # Close GUI socket

if __name__ == "__main__":
    try:
        print("Distance-Aware MIMO Speed Controller")
        print("-----------------------------------")
        print("Press Ctrl+C to stop the controller")
        
        controller = MIMOSpeedController()
        controller.run_continuous()  # Run with default 15 kph cruise speed
            
    except KeyboardInterrupt:
        print("\nExiting program...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.cleanup()
        print("Cleanup completed.")