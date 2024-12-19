import sys
import serial
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import QTimer, QUrl
from PyQt6.QtWebEngineWidgets import QWebEngineView

class MapWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Live GPS Coordinate Plotter")
        self.setGeometry(100, 100, 800, 600)

        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # Create web view for map
        self.web_view = QWebEngineView()
        self.init_map()

        # Status label for coordinates
        self.status_label = QLabel("Waiting for GPS data...")
        
        # Add widgets to main layout
        layout.addWidget(self.web_view)
        layout.addWidget(self.status_label)

        # Initialize serial connection
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            print("Serial port opened successfully")
        except serial.SerialException as e:
            self.status_label.setText(f"Error opening serial port: {e}")
            print(f"Error opening serial port: {e}")
            return

        # Timer for reading serial data
        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial)
        self.timer.start(500)  # Read every 500ms to match Arduino delay

    def init_map(self):
        """Initialize the OpenStreetMap with Leaflet."""
        html_content = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>Live GPS Tracker</title>
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css"/>
            <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
            <style>
                #map { height: 100%; width: 100%; }
                body { margin: 0; padding: 0; height: 100vh; }
            </style>
        </head>
        <body>
            <div id="map"></div>
            <script>
                // Initialize map centered on Bandung, Indonesia
                var map = L.map('map').setView([-6.917464, 107.619125], 13);
                
                // Add OpenStreetMap tiles
                L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                    attribution: '© OpenStreetMap contributors'
                }).addTo(map);

                // Create marker variable
                var marker;

                // Function to update marker
                function updateMarker(lat, lon) {
                    // Remove existing marker if it exists
                    if (marker) {
                        map.removeLayer(marker);
                    }
                    
                    // Add new marker
                    marker = L.marker([lat, lon]).addTo(map);
                    
                    // Center map on marker
                    map.setView([lat, lon], 15);
                }
            </script>
        </body>
        </html>
        """
        self.web_view.setHtml(html_content)

    def parse_gps_data(self, line):
        """Parse the GPS data from the serial line."""
        try:
            # Split the line by comma and get lat/lon values
            parts = line.split(", ")
            lat = float(parts[0].split(":")[1])
            lon = float(parts[1].split(":")[1])
            
            # Basic validation
            if -90 <= lat <= 90 and -180 <= lon <= 180:
                return lat, lon
            return None, None
            
        except (IndexError, ValueError):
            return None, None

    def read_serial(self):
        """Read and process data from serial port."""
        try:
            if self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    lat, lon = self.parse_gps_data(line)
                    if lat is not None and lon is not None:
                        # Update the map marker
                        js_code = f"updateMarker({lat}, {lon});"
                        self.web_view.page().runJavaScript(js_code)
                        
                        # Update status label
                        self.status_label.setText(f"Current Position - Latitude: {lat}, Longitude: {lon}")
                    
        except (serial.SerialException, UnicodeDecodeError) as e:
            self.status_label.setText(f"Error reading serial data: {e}")

    def closeEvent(self, event):
        """Clean up when closing the application."""
        if hasattr(self, 'serial_port'):
            self.serial_port.close()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = MapWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()