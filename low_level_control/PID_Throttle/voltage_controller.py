import serial
import time
import csv
import struct
from datetime import datetime
from typing import Optional
from read_can import CANSpeedHandler

class VoltageController:
    def __init__(self, port: str = '/dev/ttyACM0', baud_rate: int = 9600):
        """Initialize voltage controller with serial connection and CAN interface."""
        # Initialize serial connection
        self.ser = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        
        # Initialize CAN interface same as MIMO controller
        self.can_handler = CANSpeedHandler()
        self.sample_time = 0.02  # 20ms (50Hz) sampling rate
        
        # Create log file with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = open(f'voltage_log_{timestamp}.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['Timestamp', 'Voltage_Out2', 'Voltage_Out1', 'Speed_KPH'])
        
        self.current_voltage = 0.5  # Track current voltage
        
    def set_voltage(self, voltage: float) -> bool:
        try:
            voltage = max(0.5, min(1.0, voltage))
            voltage_bytes = struct.pack('f', voltage)
            self.ser.write(voltage_bytes)
            
            confirmation = self.ser.read(4)
            if len(confirmation) == 4:
                self.current_voltage = voltage
                return True
            return False
        except Exception as e:
            return False
    
    def log_data(self):
        try:
            speed = self.can_handler.read_speed()
            if speed is None:
                return None
            
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            self.csv_writer.writerow([
                timestamp, 
                self.current_voltage, 
                self.current_voltage * 2.0, 
                speed
            ])
            self.log_file.flush()
            return speed
            
        except Exception as e:
            return None
    
    def cleanup(self):
        self.log_file.close()
        self.ser.close()
        self.can_handler.close()

def main():
    controller = VoltageController()
    next_sample_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            
            if current_time >= next_sample_time:
                speed = controller.log_data()
                if speed is not None:
                    next_sample_time = current_time + controller.sample_time
            
            if input_available():
                cmd = input("\nEnter voltage (0.5-1.0) or q to quit: ").strip().lower()
                if cmd == 'q':
                    break
                try:
                    voltage = float(cmd)
                    if 0.5 <= voltage <= 1.0:
                        controller.set_voltage(voltage)
                except ValueError:
                    pass
            
            time.sleep(0.001)
                
    except KeyboardInterrupt:
        pass
    finally:
        controller.cleanup()

def input_available():
    import sys
    import select
    
    if sys.platform == 'win32':
        import msvcrt
        return msvcrt.kbhit()
    else:
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return bool(r)

if __name__ == "__main__":
    main()