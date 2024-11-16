import serial
import time
import csv
import struct
from typing import Optional
from read_can import CANSpeedHandler

class SpeedController:
    def __init__(self, serial_port: str = '/dev/ttyUSB0', baud_rate: int = 9600):  # Changed to 9600 baud rate
        """Initialize speed controller with serial connection and CAN interface."""
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        
        self.can_handler = CANSpeedHandler()
        
        # Initialize logging
        self.log_file = open('speed_data.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['Time', 'Desired_Speed', 'Actual_Speed', 'Control_Output'])
        self.start_time = time.time()
        
        # Timing
        self.sample_time = 0.02  # 20ms (50Hz) sampling rate

    def send_speeds(self, desired_speed: float, actual_speed: float) -> Optional[float]:
        """Send desired and actual speeds to Arduino and get control output."""
        try:
            # Send desired speed
            self.ser.write(struct.pack('f', desired_speed))
            
            # Wait for acknowledgment
            if self.ser.read() != b'A':
                print("Failed to get acknowledgment")
                return None
            
            # Send actual speed
            self.ser.write(struct.pack('f', actual_speed))
            
            # Read control output
            control_data = self.ser.read(4)  # Read 4 bytes (float)
            if len(control_data) == 4:
                return struct.unpack('f', control_data)[0]
            return None
        except Exception as e:
            print(f"Serial communication error: {e}")
            return None

    def log_data(self, desired_speed: float, actual_speed: float, control_output: Optional[float]):
        """Log time, speeds, and control output to CSV."""
        current_time = time.time() - self.start_time
        self.csv_writer.writerow([
            f"{current_time:.3f}",
            f"{desired_speed:.2f}",
            f"{actual_speed:.2f}",
            f"{control_output:.2f}" if control_output is not None else "NA"
        ])
        self.log_file.flush()  # Ensure data is written to file

    def run(self, desired_speed: float, duration: float = 60):
        """Run the speed control loop for specified duration."""
        end_time = time.time() + duration
        next_sample_time = time.time()
        
        try:
            print(f"Running speed control test for {duration} seconds...")
            print("Time(s) | Desired(kph) | Actual(kph) | Control(%)")
            
            while time.time() < end_time:
                current_time = time.time()
                
                # Wait until next sample time
                if current_time < next_sample_time:
                    continue
                
                # Get actual speed from CAN
                actual_speed = self.can_handler.read_speed()
                if actual_speed is None:
                    continue
                
                # Send speeds and get control output
                control_output = self.send_speeds(desired_speed, actual_speed)
                
                # Log data
                self.log_data(desired_speed, actual_speed, control_output)
                
                # Print real-time data (update display at 2Hz to avoid flooding console)
                if int(current_time * 2) > int((current_time - self.sample_time) * 2):
                    print(f"\r{current_time - self.start_time:.1f} | {desired_speed:.1f} | "
                          f"{actual_speed:.1f} | {control_output:.1f}" if control_output is not None else "NA", 
                          end="")
                
                # Calculate next sample time
                next_sample_time = current_time + self.sample_time
                
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources."""
        self.log_file.close()
        self.ser.close()
        self.can_handler.close()

if __name__ == "__main__":
    # Example usage
    try:
        controller = SpeedController()
        
        # Run test with desired speed of 30 kph for 60 seconds
        desired_speed = 30.0  # kph
        controller.run(desired_speed, duration=60)
        
    except Exception as e:
        print(f"Error: {e}")