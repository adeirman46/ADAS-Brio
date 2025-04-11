#!/usr/bin/env python3

try:
    import can
except ImportError:
    print("python-can package not found. Installing...")
    import subprocess
    subprocess.check_call(["pip3", "install", "python-can"])
    import can

import time
import sys
import os
from typing import Optional

class CANSpeedHandler:
    def __init__(self, bitrate: int = 500000):
        """Initialize CAN bus connection for CANable."""
        try:
            print("Initializing CAN interface for CANable...")
            self.bus = can.Bus(interface='socketcan', channel='slcan0', bitrate=bitrate)
            print("CAN interface initialized successfully")
        except Exception as e:
            print(f"Failed to initialize CAN interface: {e}")
            raise
        
        # Polynomial regression coefficients for speed conversion
        self.coef_a = -0.00000016  # coefficient for v1²
        self.coef_b = 0.00650007   # coefficient for v1
        self.coef_c = -1.15230758  # constant term

    def convert_to_kph(self, v1: int) -> float:
        """Convert raw CAN value to speed in KPH using polynomial regression."""
        return (self.coef_a * v1**2) + (self.coef_b * v1) + self.coef_c

    def read_speed(self) -> Optional[float]:
        """Read speed message from CAN bus and convert to KPH."""
        try:
            msg = self.bus.recv(timeout=1.0)
            if msg and msg.arbitration_id == 0x1D0:
                v1 = int.from_bytes(msg.data[0:2], byteorder='big')
                speed_kph = self.convert_to_kph(v1)
                return max(0, speed_kph)  # Ensure non-negative speed
            return None
        except Exception as e:
            print(f"Error reading CAN message: {e}")
            return None

    def start_continuous_reading(self, callback):
        """Start continuous reading of speed messages."""
        try:
            notifier = can.Notifier(self.bus, [lambda msg: self._handle_message(msg, callback)])
            return notifier
        except Exception as e:
            print(f"Error starting continuous reading: {e}")
            raise

    def _handle_message(self, msg, callback):
        """Process received CAN message if it's a speed message."""
        try:
            if msg.arbitration_id == 0x1D0:
                v1 = int.from_bytes(msg.data[0:2], byteorder='big')
                speed_kph = self.convert_to_kph(v1)
                callback(max(0, speed_kph))  # Ensure non-negative speed
        except Exception as e:
            print(f"Error handling message: {e}")

    def close(self):
        """Close the CAN bus connection."""
        try:
            self.bus.shutdown()
            print("CAN interface closed successfully")
        except Exception as e:
            print(f"Error closing CAN interface: {e}")

if __name__ == "__main__":
    def print_speed(speed):
        print(f"Current speed: {speed:.2f} KPH")
    
    # Create handler instance
    try:
        handler = CANSpeedHandler()
        print("Starting continuous CAN reading. Press Ctrl+C to stop.")
        print("Waiting for messages with ID 0x1D0...")
        
        # Start continuous reading
        notifier = handler.start_continuous_reading(print_speed)
        
        # Keep program running
        while True:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping CAN reading...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'handler' in locals():
            handler.close()
        # Clean up the interface
        os.system("sudo ip link set slcan0 down 2>/dev/null")
