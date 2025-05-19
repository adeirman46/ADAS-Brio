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
output_file = 'throttle_data_with_speed_ex6.csv'

# Initialize serial connection
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print("Serial connection established")
except Exception as e:
    print(f"Failed to initialize serial connection: {e}")
    exit(1)

# Initialize CAN speed handler
try:
    from read_can import CANSpeedHandler
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
    writer.writerow(['Time (ms)', 'Out1 (V)', 'Out2 (V)', 'Mode', 'Raw Value', 'Filtered Value', 'Pedal Percentage', 'Speed (KPH)'])
    
    print("Logging throttle data with speed to CSV...")
    print("Press 'm' to toggle Manual/ADAS mode, 'w' to increase throttle (ADAS), 's' to decrease throttle (ADAS), or Ctrl+C to stop.")
    
    try:
        while True:
            # Check for key press
            key = get_key()
            if key == 'm':
                ser.write(b'm')
                print("Sent 'm' to Arduino (toggle Manual/ADAS mode)")
            elif key == 'w':
                ser.write(b'w')
                print("Sent 'w' to Arduino (increase throttle in ADAS mode)")
            elif key == 's':
                ser.write(b's')
                print("Sent 's' to Arduino (decrease throttle in ADAS mode)")
            
            if ser.in_waiting > 0:
                # Read a line of data from Arduino
                line = ser.readline().decode('utf-8').strip()
                
                # If the line is not empty, attempt to process it
                if line:
                    # Skip status messages from Arduino
                    if not (line.startswith("Mode:") or line.startswith("Increased - Out") or 
                            line.startswith("Decreased - Out") or line.startswith("Maximum voltage") or 
                            line.startswith("Minimum voltage") or line.startswith("Throttle Control")):
                        data = line.split(',')
                        # Check if the data has exactly 7 elements
                        if len(data) == 7:
                            try:
                                time_ms = float(data[0])
                                out1 = float(data[1])
                                out2 = float(data[2])
                                mode = data[3]
                                raw_value = float(data[4])
                                filtered_value = float(data[5])
                                pedal_percentage = float(data[6])
                                
                                # Write the data to CSV
                                writer.writerow([time_ms, out1, out2, mode, raw_value, filtered_value, 
                                               pedal_percentage, current_speed if current_speed is not None else 'N/A'])
                                print(f"Time: {time_ms} ms, Out1: {out1:.2f} V, Out2: {out2:.2f} V, "
                                      f"Mode: {mode}, Raw Value: {raw_value:.2f}, Filtered Value: {filtered_value:.2f}, "
                                      f"Pedal Percentage: {pedal_percentage:.1f}%, "
                                      f"Speed: {current_speed if current_speed is not None else 'N/A'} KPH")
                            except ValueError:
                                print(f"Skipping malformed data: {line}")
                                continue
                        else:
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