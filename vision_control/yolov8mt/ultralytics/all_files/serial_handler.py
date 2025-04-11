import serial
import threading
import time
import struct
from read_can import CANSpeedHandler
from ctypes import sizeof

class SerialHandler:
    MAX_BUFF_LEN = 255
    
    def __init__(self, port, baudrate):
        """Initialize serial communication and CAN interface"""
        self.port = serial.Serial(port, baudrate, timeout=0.1)
        self.speed = 0
        self.actual_brake = 0
        self.desired_velocity = 0
        self.desired_brake = 0
        self.brake_state = 0
        self.obstacle_distance = float('inf')
        self.running = False
        self.lock = threading.Lock()
        
        # Initialize CAN speed handler
        try:
            self.can_handler = CANSpeedHandler()
            self.can_connected = True
            print("CAN speed handler initialized successfully")
        except Exception as e:
            print(f"Failed to initialize CAN handler: {e}")
            self.can_connected = False

    def write_ser(self, desired_velocity, current_velocity):
        """Write desired and current velocity to Arduino for PID control"""
        try:
            # Pack float values into bytes
            self.port.write(struct.pack('f', desired_velocity))
            self.port.write(struct.pack('f', current_velocity))
        except serial.SerialException as e:
            print(f"Failed to write to serial port: {e}")

    def read_ser(self):
        """Read throttle and brake outputs from Arduino"""
        try:
            if self.port.in_waiting >= 2 * sizeof(float):
                throttle_data = self.port.read(4)  # Read 4 bytes for float
                brake_data = self.port.read(4)     # Read 4 bytes for float
                
                if len(throttle_data) == 4 and len(brake_data) == 4:
                    throttle_percent = struct.unpack('f', throttle_data)[0]
                    brake_percent = struct.unpack('f', brake_data)[0]
                    return throttle_percent, brake_percent
        except Exception as e:
            print(f"Error reading from serial port: {e}")
        return None, None

    def update_control(self):
        """Update control outputs based on obstacle distance"""
        with self.lock:
            obstacle_distance = self.obstacle_distance
            
            # Simple distance-based speed control
            if obstacle_distance <= 3.0:  # Emergency stop within 3m
                self.desired_velocity = 0
            else:
                self.desired_velocity = 15
            
            # Read actual speed from CAN
            if self.can_connected:
                speed = self.can_handler.read_speed()
                if speed is not None:
                    self.speed = float(speed)  # Ensure proper type conversion

            # Send desired and current speed to Arduino for PID control
            self.write_ser(self.desired_velocity, self.speed)
            
            # Read control outputs from Arduino
            throttle, brake = self.read_ser()
            if throttle is not None and brake is not None:
                self.desired_brake = brake
                self.actual_brake = brake  # For now, assuming actual matches desired

    def control_thread(self):
        """Main control loop"""
        print("Control thread started")
        while self.running:
            self.update_control()
            time.sleep(0.02)  # 50Hz to match Arduino's sample time
        print("Control thread stopped")

    def start(self):
        """Start the control thread"""
        print("Starting SerialHandler thread")
        self.running = True
        self.control_thread = threading.Thread(target=self.control_thread)
        self.control_thread.start()

    def stop(self):
        """Stop the control thread and cleanup"""
        print("Stopping SerialHandler thread")
        self.running = False
        self.control_thread.join()
        self.port.close()
        if self.can_connected:
            self.can_handler.close()
        print("SerialHandler thread stopped")

    def update_obstacle_distance(self, distance):
        """Update the current obstacle distance"""
        with self.lock:
            self.obstacle_distance = distance

    def get_status(self):
        """Get current status of all variables"""
        with self.lock:
            return {
                "speed": self.speed,
                "actual_brake": self.actual_brake,
                "desired_velocity": self.desired_velocity,
                "desired_brake": self.desired_brake,
                "obstacle_distance": self.obstacle_distance
            }