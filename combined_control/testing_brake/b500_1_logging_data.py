import serial
import csv
import time
import sys
import select

# Set the correct port for your Arduino (check the COM port in your Arduino IDE)
serial_port = '/dev/ttyACM0'  # Adjust this to your port (Windows: 'COMx', Mac/Linux: '/dev/ttyUSB0')
baud_rate = 9600
output_file = 'brake_data_stop.csv'

# Initialize serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Wait for the serial connection to establish
time.sleep(2)

# Create and open CSV file
with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time (ms)', 'Raw Distance (mm)', 'Filtered Distance (mm)', 'Brake Position (%)', 'Brake Mode'])
    
    print("Logging brake data to CSV...")
    print("Press 'b' to fully press brake, 'n' to release brake, or Ctrl+C to stop.")
    
    try:
        while True:
            # Check for key press without blocking
            if select.select([sys.stdin], [], [], 0.0)[0]:
                key = sys.stdin.read(1)
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
                    # Check if the line is a status message (e.g., "Brake set to...")
                    if not line.startswith("Brake set to") and not line.startswith("Brake Control"):
                        data = line.split(',')
                        # Check if the data has exactly 5 elements (time, raw distance, filtered distance, percentage, mode)
                        if len(data) == 5:
                            try:
                                time_ms = float(data[0])  # Ensure time is a number
                                raw_distance_mm = float(data[1])  # Ensure raw distance is a number
                                filtered_distance_mm = float(data[2])  # Ensure filtered distance is a number
                                brake_percent = float(data[3])  # Ensure percentage is a number
                                brake_mode = data[4]  # Brake mode (FULL or RELEASED)
                                
                                # Write the data to CSV
                                writer.writerow([time_ms, raw_distance_mm, filtered_distance_mm, brake_percent, brake_mode])
                                print(f"Time: {time_ms} ms, Raw Distance: {raw_distance_mm} mm, Filtered Distance: {filtered_distance_mm} mm, Brake Position: {brake_percent} %, Mode: {brake_mode}")
                            except ValueError:
                                # Skip lines where data cannot be converted to float
                                print(f"Skipping malformed data: {line}")
                                continue
                        else:
                            # Skip lines that don't have exactly 5 elements
                            print(f"Skipping invalid data format: {line}")
                            continue
                    else:
                        # Print status messages from Arduino
                        print(line)
                    
    except KeyboardInterrupt:
        print("Logging stopped by user.")
        ser.close()