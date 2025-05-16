import serial
import time
import csv
import struct
from typing import Optional, Tuple
from read_can import CANSpeedHandler

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
        
        # Initialize logging with session timestamp
        self.init_logging()

    def init_logging(self):
        """Initialize log file for the entire session."""
        if not self.log_file:  # Only create if not already exists
            session_timestamp = time.strftime("%Y%m%d-%H%M%S")
            self.log_file = open(f'speed_control_session_{session_timestamp}.csv', 'w', newline='')
            self.csv_writer = csv.writer(self.log_file)
            self.csv_writer.writerow(['Session_Time', 'Test_Time', 'Desired_Speed', 'Actual_Speed', 
                                    'Throttle_Output', 'Brake_Output', 'Test_Number'])
            print(f"Logging initialized with file: speed_control_session_{session_timestamp}.csv")
            self.test_counter = 1

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

    def log_data(self, desired_speed: float, actual_speed: float, 
                 control_outputs: Optional[Tuple[float, float]], test_start_time: float):
        """Log time, speeds, and control outputs to CSV."""
        session_time = time.time() - self.start_time  # Time since session start
        test_time = time.time() - test_start_time    # Time since current test start
        
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
            
        row.append(self.test_counter)  # Add test number
        self.csv_writer.writerow(row)
        self.log_file.flush()  # Ensure data is written to disk

    def run(self, desired_speed: float, duration: float = 60):
        """Run the speed control loop for specified duration."""
        test_start_time = time.time()
        end_time = test_start_time + duration
        next_sample_time = time.time()
        
        try:
            print(f"\nRunning MIMO speed control test #{self.test_counter} for {duration} seconds...")
            print("Time(s) | Desired(kph) | Actual(kph) | Throttle(%) | Brake(%)")
            
            while time.time() < end_time:
                current_time = time.time()
                
                # Wait until next sample time
                if current_time < next_sample_time:
                    continue
                
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
                          f"{control_outputs[0]:.1f} | {control_outputs[1]:.1f}" 
                          if control_outputs else "NA", end="")
                
                # Calculate next sample time
                next_sample_time = current_time + self.sample_time
                
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
            return True  # Signal that interruption was by user
        except Exception as e:
            print(f"\nError during test: {e}")
            return False
        else:
            print(f"\nTest #{self.test_counter} completed successfully")
            self.test_counter += 1  # Increment test counter for next test
            return True

    def cleanup(self):
        """Clean up resources."""
        if self.log_file:
            self.log_file.close()
        self.ser.close()
        self.can_handler.close()

if __name__ == "__main__":
    try:
        print("MIMO Speed Controller")
        print("--------------------")
        print("Enter 'q' for speed or duration to quit")
        
        controller = MIMOSpeedController()
        
        while True:
            # Get user input for desired speed
            while True:
                speed_input = input("\nEnter desired speed (kph): ")
                if speed_input.lower() == 'q':
                    raise KeyboardInterrupt  # Clean exit
                try:
                    desired_speed = float(speed_input)
                    if 0 <= desired_speed <= 100:  # Assuming max speed is 100 kph
                        break
                    print("Speed must be between 0 and 100 kph")
                except ValueError:
                    print("Please enter a valid number")

            # Get user input for duration
            while True:
                duration_input = input("Enter test duration (seconds): ")
                if duration_input.lower() == 'q':
                    raise KeyboardInterrupt  # Clean exit
                try:
                    duration = float(duration_input)
                    if duration > 0:
                        break
                    print("Duration must be greater than 0")
                except ValueError:
                    print("Please enter a valid number")

            # Run the test
            if not controller.run(desired_speed, duration):
                print("Test failed, continuing with same test number...")
            
            print("\nReady for next test...")
            
    except KeyboardInterrupt:
        print("\nExiting program...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.cleanup()
        print("Cleanup completed.")