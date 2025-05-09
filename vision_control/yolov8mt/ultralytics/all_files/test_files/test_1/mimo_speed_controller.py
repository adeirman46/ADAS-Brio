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
        
        # Initialize logging with session timestamp
        self.init_logging()
        
        # Start distance receiver thread
        self.distance_thread = Thread(target=self._receive_distance)
        self.distance_thread.daemon = True
        self.distance_thread.start()

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

    def run_continuous(self, cruise_speed: float = 15.0):
        """Run the speed control loop continuously with distance-based speed adjustment."""
        test_start_time = time.time()
        next_sample_time = time.time()
        
        try:
            print(f"\nRunning distance-aware MIMO speed control (cruise speed: {cruise_speed} kph)")
            print("Time(s) | Target(kph) | Actual(kph) | Throttle(%) | Brake(%) | Distance(m)")
            
            while True:
                current_time = time.time()
                
                if current_time < next_sample_time:
                    continue
                
                # Determine desired speed based on distance
                desired_speed = cruise_speed if self.current_distance > 10.0 else 0.0
                
                # Get actual speed from CAN
                actual_speed = self.can_handler.read_speed()
                if actual_speed is None:
                    continue
                
                # Send speeds and get control outputs
                control_outputs = self.send_speeds(desired_speed, actual_speed)
                
                # Log data with test start time
                self.log_data(desired_speed, actual_speed, control_outputs, test_start_time)
                
                # Print real-time data (update display at 2Hz)
                if int(current_time * 2) > int((current_time - self.sample_time) * 2):
                    print(f"\r{current_time - test_start_time:.1f} | {desired_speed:.1f} | "
                          f"{actual_speed:.1f} | "
                          f"{control_outputs[0]:.1f} | {control_outputs[1]:.1f} | "
                          f"{self.current_distance:.1f}" if control_outputs else "NA", end="")
                
                next_sample_time = current_time + self.sample_time
                
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        except Exception as e:
            print(f"\nError during test: {e}")

    def cleanup(self):
        """Clean up resources."""
        self.stop_event.set()  # Signal distance thread to stop
        if self.distance_thread.is_alive():
            self.distance_thread.join()
        if self.log_file:
            self.log_file.close()
        self.ser.close()
        self.can_handler.close()
        self.socket.close()

if __name__ == "__main__":
    try:
        print("Distance-Aware MIMO Speed Controller")
        print("-----------------------------------")
        print("Press Ctrl+C to stop the controller")
        
        controller = MIMOSpeedController()
        controller.run_continuous(cruise_speed=15)  # Run with default 15 kph cruise speed
            
    except KeyboardInterrupt:
        print("\nExiting program...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.cleanup()
        print("Cleanup completed.")