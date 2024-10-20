import serial
import can
import threading
import time
from fuzzy_braking import FuzzyBrakingSystem

class SerialHandler:
    MAX_BUFF_LEN = 255
    MESSAGE_ID_RPM = 0x47
    MESSAGE_ID_SPEED = 0x55
    MESSAGE_ID_BRAKE = 0x21

    def __init__(self, port, baudrate):
        self.port = serial.Serial(port, baudrate, timeout=0.1)
        self.speed = 0
        self.actual_brake = 0
        self.desired_velocity = 0
        self.desired_brake = 0
        self.brake_state = 0
        self.obstacle_distance = float('inf')
        self.latitude = 0.0
        self.longitude = 0.0
        self.running = False
        self.lock = threading.Lock()
        
        self.fuzzy_braking = FuzzyBrakingSystem()
        
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan', receive_own_messages=False)
            self.can_connected = True
            print("CAN bus connected successfully")
        except (can.CanError, OSError) as e:
            print(f"Failed to connect to CAN bus. Error: {e}. Running without CAN.")
            self.can_connected = False

    def write_ser(self, desired_velocity, desired_brake, actual_velocity, brake_state):
        cmd = f'{desired_velocity},{desired_brake},{actual_velocity},{brake_state}\n'
        try:
            self.port.write(cmd.encode())
        except serial.SerialException as e:
            print(f"Failed to write to serial port: {e}")

    def read_ser(self):
        try:
            if self.port.in_waiting:
                line = self.port.readline().decode().strip()
                data = line.split(', ')
                if len(data) >= 7:
                    actual_brake = float(data[3].split(': ')[1])
                    lat = float(data[5].split('=')[1])
                    lon = float(data[6].split('=')[1])
                    return actual_brake, lat, lon
        except (ValueError, UnicodeDecodeError, IndexError, serial.SerialException) as e:
            print(f"Error reading from serial port: {e}")
        return None, None, None

    def process_can_message(self, msg):
        if msg.arbitration_id == self.MESSAGE_ID_SPEED:
            third_byte_speed = msg.data[3]
            with self.lock:
                self.speed = int(0.2829 * third_byte_speed + 0.973)
        elif msg.arbitration_id == self.MESSAGE_ID_BRAKE:
            brake_flag = msg.data[0]
            with self.lock:
                self.brake_state = 1 if brake_flag == 0x60 else 0

    def update_control(self):
        with self.lock:
            obstacle_distance = self.obstacle_distance
            speed = self.speed

        braking_signal = self.fuzzy_braking.get_braking_value(obstacle_distance, speed)
        self.desired_brake = int(braking_signal * 200)

        max_velocity = 40
        min_velocity = 0
        safe_distance = 10

        if obstacle_distance > safe_distance:
            self.desired_velocity = max_velocity
        elif obstacle_distance > 0:
            self.desired_velocity = int(min_velocity + (max_velocity - min_velocity) * (obstacle_distance / safe_distance))
        else:
            self.desired_velocity = min_velocity

        self.write_ser(self.desired_velocity, self.desired_brake, speed, self.brake_state)

        new_actual_brake, new_lat, new_lon = self.read_ser()
        if new_actual_brake is not None:
            self.actual_brake = new_actual_brake
        if new_lat is not None and new_lon is not None:
            self.latitude = new_lat
            self.longitude = new_lon

    def can_thread(self):
        print("CAN thread started")
        while self.running and self.can_connected:
            try:
                message = self.bus.recv(timeout=0.1)
                if message:
                    self.process_can_message(message)
            except can.CanError as e:
                print(f"CAN error: {e}")
        print("CAN thread stopped")

    def control_thread(self):
        print("Control thread started")
        while self.running:
            self.update_control()
            time.sleep(0.01)  # 10ms control loop
        print("Control thread stopped")

    def start(self):
        print("Starting SerialHandler threads")
        self.running = True
        if self.can_connected:
            self.can_thread = threading.Thread(target=self.can_thread)
            self.can_thread.start()
        self.control_thread = threading.Thread(target=self.control_thread)
        self.control_thread.start()

    def stop(self):
        print("Stopping SerialHandler threads")
        self.running = False
        if hasattr(self, 'can_thread'):
            self.can_thread.join()
        self.control_thread.join()
        self.port.close()
        if self.can_connected:
            self.bus.shutdown()
        print("SerialHandler threads stopped")

    def update_obstacle_distance(self, distance):
        with self.lock:
            self.obstacle_distance = distance

    def get_status(self):
        with self.lock:
            return {
                "speed": self.speed,
                "actual_brake": self.actual_brake,
                "desired_velocity": self.desired_velocity,
                "desired_brake": self.desired_brake,
                "brake_state": self.brake_state,
                "obstacle_distance": self.obstacle_distance,
                "latitude": self.latitude,
                "longitude": self.longitude
            }

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Test SerialHandler")
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='Serial port for communication')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baudrate for serial communication')
    args = parser.parse_args()

    handler = SerialHandler(args.port, args.baudrate)
    
    try:
        handler.start()
        print("SerialHandler started. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
            status = handler.get_status()
            print(f"Current status: {status}")
    except KeyboardInterrupt:
        print("Stopping SerialHandler...")
    finally:
        handler.stop()
        print("SerialHandler stopped.")