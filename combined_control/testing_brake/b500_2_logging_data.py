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

# Set the correct port for your Arduino (check the COM port in your Arduino IDE)
serial_port = '/dev/ttyACM0'  # Adjust this to your port (Windows: 'COMx', Mac/Linux: '/dev/ttyUSB0')
baud_rate = 9600
output_file = 'brake_data_with_speed40.csv'

# Initialize serial connection
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print("Serial connection established")
except Exception as e:
    print(f"Failed to initialize serial connection: {e}")
    exit(1)

# Initialize CAN speed handler
try:
    from testing_throttle.read_can import CANSpeedHandler
    can_handler = CANSpeedHandler()
    print("CAN speed handler initialized successfully")
except Exception as e:
    print(f"Failed to initialize CAN speed handler: {e}")
    ser.close()
    exit(1)

# Variable to store the latest speed
current_speed = None

def update_speed(speed):
    """Callback to update the current speed."""
    global current_speed
    current_speed = speed
    print(f"Updated speed: {speed:.2f} KPH")

# Start continuous CAN reading
try:
    notifier = can_handler.start_continuous_reading(update_speed)
except Exception as e:
    print(f"Failed to start CAN reading: {e}")
    ser.close()
    can_handler.close()
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
    writer.writerow(['Time (ms)', 'Raw Distance (mm)', 'Filtered Distance (mm)', 'Brake Position (%)', 'Speed (KPH)', 'Brake State'])
    
    print("Logging brake data with speed to CSV...")
    print("Press 'b' to fully press brake, 'n' to release brake, or Ctrl+C to stop.")
    
    try:
        while True:
            # Check for key press
            key = get_key()
            if key == 'b':
                ser.write(b'b')
                print("Sent 'b' to Arduino (fully press brake)")
            elif key == 'n':
                ser.write(b'n')
                print("Sent 'n' to Arduino (release brake)")
            
            if ser.in_waiting > 0:
                # Read a line of data from Arduino
                line = ser.readline().decode('utf-8').strip()
                
                # If the line is not empty, attempt to split and process it
                if line:
                    # Skip status messages from Arduino
                    if not line.startswith("Brake set to") and not line.startswith("Brake Control"):
                        data = line.split(',')
                        # Check if the data has 4 or 5 elements
                        if len(data) in [4, 5]:
                            try:
                                time_ms = float(data[0])  # Ensure time is a number
                                raw_distance_mm = float(data[1])  # Ensure raw distance is a number
                                filtered_distance_mm = float(data[2])  # Ensure filtered distance is a number
                                brake_percent = float(data[3])  # Ensure percentage is a number
                                
                                # Handle the brake state (if present)
                                brake_state = data[4] if len(data) == 5 else 'N/A'
                                
                                # Write the data to CSV, including the current speed and brake state
                                writer.writerow([time_ms, raw_distance_mm, filtered_distance_mm, brake_percent, 
                                               current_speed if current_speed is not None else 'N/A', brake_state])
                                print(f"Time: {time_ms} ms, Raw Distance: {raw_distance_mm} mm, "
                                      f"Filtered Distance: {filtered_distance_mm} mm, Brake Position: {brake_percent} %, "
                                      f"Speed: {current_speed if current_speed is not None else 'N/A'} KPH, "
                                      f"Brake State: {brake_state}")
                            except ValueError:
                                # Skip lines where data cannot be converted to float
                                print(f"Skipping malformed data: {line}")
                                continue
                        else:
                            # Skip lines that don't have 4 or 5 elements
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
        can_handler.close()
        # Clean up the CAN interface
        os.system("sudo ip link set slcan0 down 2>/dev/null")