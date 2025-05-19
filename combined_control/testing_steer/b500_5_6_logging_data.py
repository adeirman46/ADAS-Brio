#!/usr/bin/env python3
import serial
import csv
import time
import os
import sys
import select
import platform

# Cross-platform non-blocking keyboard input
if platform.system() == "Windows":
    import msvcrt
else:
    import termios
    import tty

# Set the correct port for your Arduino
serial_port = '/dev/ttyACM0'  # Adjust to your port (Windows: 'COMx', Mac/Linux: '/dev/ttyUSB0')
baud_rate = 9600
output_file = 'steering_data_with_voltage2.csv'

# Initialize serial connection
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print("Serial connection established")
except Exception as e:
    print(f"Failed to initialize serial connection: {e}")
    exit(1)

# Wait for the serial connection to establish
time.sleep(2)

# Function for non-blocking keyboard input
def get_key():
    if platform.system() == "Windows":
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8')
        return None
    else:
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

# Set terminal to non-blocking mode for Unix-like systems
if platform.system() != "Windows":
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

# Create and open CSV file
with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time (ms)', 'Voltage1 (V)', 'Voltage2 (V)', 'Mode', 'Raw Value', 'Filtered Value', 'Steering Angle (deg)'])
    
    print("Logging steering and voltage data to CSV...")
    print("Press 'm' to toggle Manual/Keyboard mode, 'a' to steer left (Keyboard mode), 'd' to steer right (Keyboard mode), or Ctrl+C to stop.")
    
    try:
        while True:
            # Check for key press
            key = get_key()
            if key == 'm':
                ser.write(b'm')
                print("Sent 'm' to Arduino (toggle Manual/Keyboard mode)")
            elif key == 'a':
                ser.write(b'a')
                print("Sent 'a' to Arduino (steer left in Keyboard mode)")
            elif key == 'd':
                ser.write(b'd')
                print("Sent 'd' to Arduino (steer right in Keyboard mode)")
            
            if ser.in_waiting > 0:
                # Read a line of data from Arduino
                line = ser.readline().decode('utf-8').strip()
                
                # If the line is not empty, attempt to process it
                if line:
                    # Skip status messages from Arduino
                    if not (line.startswith("Mode:") or line.startswith("Steering Control")):
                        data = line.split(',')
                        # Check if the data has exactly 7 elements
                        if len(data) == 7:
                            try:
                                time_ms = float(data[0])  # Ensure time is a number
                                voltage1 = float(data[1])  # Ensure voltage1 is a number
                                voltage2 = float(data[2])  # Ensure voltage2 is a number
                                mode = data[3]  # Mode is a string (Manual/Keyboard)
                                raw_value = float(data[4])  # Ensure raw value is a number
                                filtered_value = float(data[5])  # Ensure filtered value is a number
                                steering_angle = float(data[6])  # Ensure steering angle is a number
                                
                                # Write the data to CSV
                                writer.writerow([time_ms, voltage1, voltage2, mode, raw_value, filtered_value, steering_angle])
                                print(f"Time: {time_ms} ms, Voltage1: {voltage1:.2f} V, Voltage2: {voltage2:.2f} V, "
                                      f"Mode: {mode}, Raw Value: {raw_value:.2f}, Filtered Value: {filtered_value:.2f}, "
                                      f"Steering Angle: {steering_angle:.1f} deg")
                            except ValueError:
                                # Skip lines where numeric data cannot be converted to float
                                print(f"Skipping malformed data: {line}")
                                continue
                        else:
                            # Skip lines that don't have exactly 7 elements
                            print(f"Skipping invalid data format: {line}")
                            continue
                    else:
                        # Print status messages from Arduino
                        print(line)
                    
    except KeyboardInterrupt:
        print("Logging stopped by user.")
    finally:
        # Restore terminal settings for Unix-like systems
        if platform.system() != "Windows":
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        ser.close()