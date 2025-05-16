import serial
import csv
import time

# Set the correct port for your Arduino (check the COM port in your Arduino IDE)
serial_port = '/dev/ttyACM0'  # Adjust this to your port (Windows: 'COMx', Mac/Linux: '/dev/ttyUSB0')
baud_rate = 9600
output_file = 'brake_data_with_time_iir_1st_order.csv'

# Initialize serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Wait for the serial connection to establish
time.sleep(2)

# Create and open CSV file
with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Time (ms)', 'Distance (mm)', 'Brake Position (%)'])  # Write header
    
    print("Logging brake data with timestamp to CSV...")
    
    try:
        while True:
            if ser.in_waiting > 0:
                # Read a line of data from Arduino
                line = ser.readline().decode('utf-8').strip()
                
                # If the line is not empty, attempt to split and process it
                if line:
                    data = line.split(', ')
                    # Check if the data has exactly 3 elements (time, distance, percentage)
                    if len(data) == 3:
                        try:
                            time_ms = float(data[0])  # Ensure time is a number
                            distance_mm = float(data[1])  # Ensure distance is a number
                            brake_percent = float(data[2])  # Ensure percentage is a number
                            
                            # Write the data to CSV
                            writer.writerow([time_ms, distance_mm, brake_percent])
                            print(f"Time: {time_ms} ms, Distance: {distance_mm} mm, Brake Position: {brake_percent} %")
                        except ValueError:
                            # Skip lines where data cannot be converted to float
                            print(f"Skipping malformed data: {line}")
                            continue
                    else:
                        # Skip lines that don't have exactly 3 elements
                        print(f"Skipping invalid data format: {line}")
                        continue
                    
    except KeyboardInterrupt:
        print("Logging stopped by user.")
        ser.close()