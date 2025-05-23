import serial
import time
import csv
import struct
import socket
from typing import Optional, Tuple
from read_can import CANSpeedHandler
from threading import Thread, Event
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class MIMOSpeedController:
    def __init__(self, serial_port: str = '/dev/ttyACM0', baud_rate: int = 9600):
        """Initialize MIMO speed controller with serial connection and CAN interface."""
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
        time.sleep(2)
        
        self.can_handler = CANSpeedHandler()
        self.start_time = time.time()
        self.sample_time = 0.01  # Align with Arduino's 10ms loop
        self.log_file = None
        self.csv_writer = None
        
        self.current_distance = float('inf')
        self.current_delta = 0.0  # Initialize delta_m (delta_steer)
        self.stop_event = Event()
        
        # Socket for receiving distance
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('localhost', 12345))
        self.socket.settimeout(0.1)
        
        # Socket for sending data to GUI
        self.gui_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gui_address = ('localhost', 12346)
        
        # Socket for receiving mode
        self.mode_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mode_socket.bind(('localhost', 12360))
        self.mode_socket.settimeout(0.1)
        
        # Socket for receiving delta_m
        self.delta_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.delta_socket.bind(('localhost', 12361))
        self.delta_socket.settimeout(0.1)
        
        self.mode = 0  # Default mode: 0 (ACC), 1 (AEB), 2 (LKAS)
        
        # Initialize fuzzy logic system for ACC
        self.distance = ctrl.Antecedent(np.arange(0, 20.1, 0.1), 'distance')
        self.speed = ctrl.Consequent(np.arange(0, 30.1, 0.1), 'speed')

        # Define membership functions for distance
        self.distance['close'] = fuzz.trapmf(self.distance.universe, [0, 0, 2.5, 5])
        self.distance['near'] = fuzz.trimf(self.distance.universe, [2.5, 7.5, 10])
        self.distance['medium'] = fuzz.trimf(self.distance.universe, [7.5, 12.5, 15])
        self.distance['far'] = fuzz.trapmf(self.distance.universe, [12.5, 15, 20, 20])

        # Define membership functions for speed
        self.speed['stop'] = fuzz.trapmf(self.speed.universe, [0, 0, 2.5, 5])
        self.speed['slow'] = fuzz.trimf(self.speed.universe, [2.5, 10, 17.5])
        self.speed['medium'] = fuzz.trimf(self.speed.universe, [12.5, 20, 27.5])
        self.speed['fast'] = fuzz.trapmf(self.speed.universe, [25, 30, 30, 30])

        # Define fuzzy rules
        rule1 = ctrl.Rule(self.distance['close'], self.speed['stop'])
        rule2 = ctrl.Rule(self.distance['near'], self.speed['slow'])
        rule3 = ctrl.Rule(self.distance['medium'], self.speed['medium'])
        rule4 = ctrl.Rule(self.distance['far'], self.speed['fast'])

        # Create control system
        self.speed_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
        self.speed_sim = ctrl.ControlSystemSimulation(self.speed_ctrl)
        
        self.init_logging()
        
        # Thread for receiving distance
        self.distance_thread = Thread(target=self._receive_distance)
        self.distance_thread.daemon = True
        self.distance_thread.start()
        
        # Thread for receiving mode
        self.mode_thread = Thread(target=self._receive_mode)
        self.mode_thread.daemon = True
        self.mode_thread.start()
        
        # Thread for receiving delta_m
        self.delta_thread = Thread(target=self._receive_delta)
        self.delta_thread.daemon = True
        self.delta_thread.start()

    def compute_desired_speed(self, dist: float) -> float:
        """Compute desired speed using fuzzy logic for ACC mode."""
        dist = min(max(dist, 0), 20)  # Clamp distance to fuzzy system's universe (0–20m)
        self.speed_sim.input['distance'] = dist
        self.speed_sim.compute()
        return self.speed_sim.output['speed']

    def send_to_gui(self, actual_speed: float, brake_percentage: float):
        try:
            data = struct.pack('ff', actual_speed, brake_percentage)
            self.gui_socket.sendto(data, self.gui_address)
        except Exception as e:
            print(f"Error sending data to GUI: {e}")

    def _receive_distance(self):
        while not self.stop_event.is_set():
            try:
                data, _ = self.socket.recvfrom(1024)
                self.current_distance = struct.unpack('f', data)[0]
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving distance: {e}")

    def _receive_mode(self):
        while not self.stop_event.is_set():
            try:
                data, _ = self.mode_socket.recvfrom(4)
                self.mode = struct.unpack('i', data)[0]
                print(f"Received mode: {self.mode}")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving mode: {e}")

    def _receive_delta(self):
        while not self.stop_event.is_set():
            try:
                data, _ = self.delta_socket.recvfrom(4)
                delta = struct.unpack('f', data)[0]
                self.current_delta = delta  # Removed clamping
                if abs(self.current_delta) > 10.0:  # Reasonable threshold for warning
                    print(f"Warning: Large delta_m received: {self.current_delta:.2f} meters")
                print(f"Received delta_m: {self.current_delta}")
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving delta_m: {e}")

    def init_logging(self):
        if not self.log_file:
            session_timestamp = time.strftime("%Y%m%d-%H%M%S")
            self.log_file = open(f'speed_control_session_{session_timestamp}.csv', 'w', newline='')
            self.csv_writer = csv.writer(self.log_file)
            self.csv_writer.writerow(['Session_Time', 'Test_Time', 'Desired_Speed', 'Actual_Speed', 
                                    'Throttle_Output', 'Brake_Output', 'Distance', 
                                    'Delta_Steer', 'Desired_Delta_Steer', 'Test_Number'])
            print(f"Logging initialized with file: speed_control_session_{session_timestamp}.csv")
            self.test_counter = 1

    def log_data(self, desired_speed: float, actual_speed: float, 
                 control_outputs: Optional[Tuple[float, float]], test_start_time: float,
                 delta_steer: float, desired_delta_steer: float):
        session_time = time.time() - self.start_time
        test_time = time.time() - test_start_time
        
        row = [
            f"{session_time:.3f}",
            f"{test_time:.3f}",
            f"{desired_speed:.2f}",
            f"{actual_speed:.2f}"
        ]
        
        if control_outputs:
            throttle, brake = control_outputs
            # Clamp throttle and brake to reasonable range for logging
            throttle = max(min(throttle, 100.0), 0.0)
            brake = max(min(brake, 100.0), 0.0)
            row.extend([f"{throttle:.2f}", f"{brake:.2f}"])
        else:
            row.extend(["0.00", "0.00"])
            
        row.extend([f"{self.current_distance:.2f}", f"{delta_steer:.2f}", f"{desired_delta_steer:.2f}"])
        row.append(self.test_counter)
        self.csv_writer.writerow(row)
        self.log_file.flush()

    def send_speeds(self, desired_speed: float, actual_speed: float, 
                    delta_steer: float, desired_delta_steer: float) -> Optional[Tuple[float, float]]:
        try:
            # Flush serial buffer to clear stale data
            self.ser.flushInput()
            self.ser.flushOutput()
            
            # Send synchronization byte 'S' followed by four floats
            self.ser.write(b'S')
            self.ser.write(struct.pack('f', desired_speed))
            self.ser.write(struct.pack('f', actual_speed))
            self.ser.write(struct.pack('f', delta_steer))
            self.ser.write(struct.pack('f', desired_delta_steer))
            
            # Small delay to allow Arduino to process and respond
            time.sleep(0.005)
            
            # Retry reading up to 3 times
            for _ in range(3):
                throttle_data = self.ser.read(4)
                brake_data = self.ser.read(4)
                
                if len(throttle_data) == 4 and len(brake_data) == 4:
                    throttle = struct.unpack('f', throttle_data)[0]
                    brake = struct.unpack('f', brake_data)[0]
                    # Validate throttle and brake values
                    if not (np.isfinite(throttle) and np.isfinite(brake)):
                        print("Invalid throttle/brake values received, retrying")
                        continue
                    return throttle, brake
                print("Incomplete serial data received, retrying")
                time.sleep(0.005)
            
            print("Failed to receive valid serial data after retries")
            return None
        except Exception as e:
            print(f"Serial communication error: {e}")
            return None

    def run_continuous(self, cruise_speed: float = 10.0):
        test_start_time = time.time()
        next_sample_time = time.time()

        try:
            print(f"\nRunning speed control (cruise speed: {cruise_speed} kph)")
            print("Time(s) | Target(kph) | Actual(kph) | Throttle(%) | Brake(%) | Distance(m) | Delta(m)")

            while True:
                current_time = time.time()

                if current_time < next_sample_time:
                    continue

                actual_speed = self.can_handler.read_speed()
                if actual_speed is None:
                    continue

                delta_steer = 0.0
                desired_delta_steer = 0.0

                if self.mode == 2:  # LKAS
                    desired_speed = 10.0  # Constant speed of 10 kph for LKAS
                    delta_steer = self.current_delta  # Use the received delta_m as delta_steer
                    desired_delta_steer = 0.0  # Desired delta_steer is always 0
                elif self.mode == 1:  # AEB
                    if self.current_distance < 2.0:
                        desired_speed = 0.0
                    else:
                        desired_speed = cruise_speed
                else:  # ACC (mode 0)
                    desired_speed = self.compute_desired_speed(self.current_distance)
                    desired_speed = min(desired_speed, cruise_speed, 15.0)

                # Send speeds and steering data
                control_outputs = self.send_speeds(desired_speed, actual_speed, delta_steer, desired_delta_steer)

                if control_outputs:
                    throttle, brake = control_outputs
                    # Clamp throttle and brake for GUI and display
                    throttle = max(min(throttle, 100.0), 0.0)
                    brake = max(min(brake, 100.0), 0.0)
                    self.send_to_gui(actual_speed, brake)
                else:
                    throttle, brake = 0.0, 0.0  # Default values for logging/display

                # Log data
                self.log_data(desired_speed, actual_speed, control_outputs, test_start_time, 
                            delta_steer, desired_delta_steer)

                if int(current_time * 100) > int((current_time - self.sample_time) * 100):
                    print(f"\r{current_time - test_start_time:.1f} | {desired_speed:.1f} | "
                        f"{actual_speed:.1f} | {throttle:.1f} | {brake:.1f} | "
                        f"{self.current_distance:.1f} | {delta_steer:.1f}", end="")

                next_sample_time = current_time + self.sample_time

        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        except Exception as e:
            print(f"\nError during test: {e}")

    def _get_status_message(self, speed_stable: bool, speed_error: float, distance: float) -> str:
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
        self.stop_event.set()
        if self.distance_thread.is_alive():
            self.distance_thread.join()
        if self.mode_thread.is_alive():
            self.mode_thread.join()
        if self.delta_thread.is_alive():
            self.delta_thread.join()
        if self.log_file:
            self.log_file.close()
        self.ser.close()
        self.can_handler.close()
        self.socket.close()
        self.gui_socket.close()
        self.mode_socket.close()
        self.delta_socket.close()

if __name__ == "__main__":
    try:
        print("Distance-Aware MIMO Speed Controller with LKAS Support")
        print("-----------------------------------")
        print("Press Ctrl+C to stop the controller")
        
        controller = MIMOSpeedController()
        controller.run_continuous()
            
    except KeyboardInterrupt:
        print("\nExiting program...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.cleanup()
        print("Cleanup completed.")