from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSizePolicy, QFrame
from PyQt6.QtGui import QImage, QPixmap, QFont, QColor
from PyQt6.QtCore import Qt, QUrl, QTimer
from PyQt6.QtWebEngineWidgets import QWebEngineView
from plane_visualization import PlaneVisualizationWidget
from threading import Thread
from PyQt6.QtCore import pyqtSlot, QMetaObject, Q_ARG
import cv2
import numpy as np
import struct
import socket
from typing import Optional
import time

class FPSCounter:
    def __init__(self):
        self.prev_time = time.time()
        self.fps = 0
        self.alpha = 0.1  # Smoothing factor for moving average

    def update(self):
        current_time = time.time()
        self.fps = (1 - self.alpha) * self.fps + self.alpha * (1.0 / (current_time - self.prev_time))
        self.prev_time = current_time
        return self.fps


class VideoStreamGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.warning_visible = False
        self.blink_timer = QTimer(self)
        self.blink_timer.timeout.connect(self.toggle_warning)
        self.fps_counter = FPSCounter()  # Add FPS counter

        # Initialize UDP sockets for receiving data
        self.init_sockets()
        
        # Start reception threads
        self.start_reception_threads()
    
    def init_sockets(self):
        """Initialize UDP sockets for receiving data from all components."""
        # Socket for control data (speed and brake)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.control_socket.bind(('localhost', 12346))
        self.control_socket.settimeout(0.1)
        
        # Socket for video stream
        self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_socket.bind(('localhost', 12347))
        self.video_socket.settimeout(0.1)
        
        # Socket for distance data
        self.distance_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.distance_socket.bind(('localhost', 12348))
        self.distance_socket.settimeout(0.1)
        
        # Flag for thread control
        self.running = True

    def initUI(self):
        self.setWindowTitle("Advanced Driver Assistance System")
        self.setStyleSheet("background-color: #e6e9ed; color: #4676fa;")
        self.resize(1280, 720)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Left side (video and info panel)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        main_layout.addWidget(left_widget, 7)  # Allocate 70% of the width

        # Video display
        self.video_container = QFrame()
        self.video_container.setLineWidth(2)
        self.video_container.setMidLineWidth(1)
        self.video_container.setStyleSheet("background-color: #e6e9ed;")
        self.video_container.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        left_layout.addWidget(self.video_container, 9)  # Allocate 90% of the height

        self.video_label = QLabel(self.video_container)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background-color: transparent;")

        # Warning label (overlay on video)
        self.warning_label = QLabel("WARNING: OBSTACLE AHEAD", self.video_container)
        self.warning_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.warning_label.setStyleSheet("""
            background-color: rgba(255, 0, 0, 0.7);
            color: white;
            font-size: 24px;
            font-weight: bold;
            padding: 10px;
            border: 2px solid white;
            border-radius: 5px;
        """)
        self.warning_label.hide()

        # Info panel with brake display
        info_panel = QWidget()
        info_layout = QHBoxLayout(info_panel)
        left_layout.addWidget(info_panel, 1)  # Allocate 10% of the height

        # Create status labels with consistent styling
        self.distance_label = QLabel("Distance: N/A")
        self.speed_label = QLabel("Speed: N/A")
        self.brake_label = QLabel("Brake: N/A")
        self.adas_label = QLabel("ADAS: Activated")
        
        # Set consistent styling for all status labels
        status_style = "font-size: 16px; padding: 5px; background-color: #c0cffa; border-radius: 5px;"
        for label in (self.distance_label, self.speed_label, self.brake_label):
            label.setStyleSheet(status_style)
            info_layout.addWidget(label)
        
        # ADAS label with special styling
        self.adas_label.setStyleSheet("font-size: 16px; padding: 5px; background-color: #008000; color: white; border-radius: 5px;")
        info_layout.addWidget(self.adas_label)

        # Right side (Google Maps and 2D plane visualization)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        main_layout.addWidget(right_widget, 3)  # Allocate 30% of the width

        # Google Maps
        self.map_view = QWebEngineView()
        self.map_view.setUrl(QUrl("https://maps.google.com"))
        self.map_loading_label = QLabel('Loading Google Maps...')
        self.map_loading_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.map_loading_label.setFont(QFont('Arial', 16))
        self.map_loading_label.setStyleSheet("color: #4676fa; background-color: #c0cffa;")
        right_layout.addWidget(self.map_loading_label)
        right_layout.addWidget(self.map_view, 5)
        self.map_view.hide()
        self.map_view.loadFinished.connect(self.on_map_loaded)

        # 2D plane visualization
        self.plane_widget = PlaneVisualizationWidget()
        self.plane_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.plane_widget.setStyleSheet("background-color: #c0cffa; border-radius: 5px;")
        right_layout.addWidget(self.plane_widget, 5)

        self.showMaximized()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_video_layout()

    def update_video_layout(self):
        # Update video label size
        self.video_label.setGeometry(self.video_container.rect())
        
        # Update warning label size and position
        warning_width = self.video_container.width() // 2  # Half the width of the container
        warning_height = 50
        self.warning_label.setFixedSize(warning_width, warning_height)
        warning_x = (self.video_container.width() - warning_width) // 2  # Center horizontally
        warning_y = 20  # 20 pixels from the top
        self.warning_label.move(warning_x, warning_y)

    def on_map_loaded(self):
        self.map_loading_label.hide()
        self.map_view.show()
    
    def start_reception_threads(self):
        """Start threads for receiving different types of data."""
        # Thread for control data
        self.control_thread = Thread(target=self.receive_control_data)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        # Thread for video stream
        self.video_thread = Thread(target=self.receive_video_stream)
        self.video_thread.daemon = True
        self.video_thread.start()
        
        # Thread for distance data
        self.distance_thread = Thread(target=self.receive_distance_data)
        self.distance_thread.daemon = True
        self.distance_thread.start()

    def receive_control_data(self):
        """Receive speed and brake data from MIMO controller."""
        while self.running:
            try:
                data, _ = self.control_socket.recvfrom(1024)
                speed, brake = struct.unpack('ff', data)
                # Update GUI elements in thread-safe way
                QMetaObject.invokeMethod(self, "update_control_display",
                                       Qt.ConnectionType.QueuedConnection,
                                       Q_ARG(float, speed),
                                       Q_ARG(float, brake))
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving control data: {e}")

    def update_brake_status(self, brake_value):
        """Update brake label color and text based on brake value."""
        self.brake_label.setText(f"Brake: {brake_value:.1f}%")
        if brake_value > 80:
            self.brake_label.setStyleSheet("font-size: 16px; padding: 5px; background-color: #ff4d4d; color: white; border-radius: 5px;")
        elif brake_value > 50:
            self.brake_label.setStyleSheet("font-size: 16px; padding: 5px; background-color: #ffa64d; color: white; border-radius: 5px;")
        else:
            self.brake_label.setStyleSheet("font-size: 16px; padding: 5px; background-color: #c0cffa; border-radius: 5px;")

    @pyqtSlot(float, float)
    def update_control_display(self, speed, brake):
        """Update speed and brake displays."""
        self.speed_label.setText(f"Speed: {speed:.1f} km/h")
        self.update_brake_status(brake)

    @pyqtSlot(object)
    def update_video_display(self, frame):
        """Update video display with FPS counter."""
        if frame is not None:
            # Update FPS
            fps = self.fps_counter.update()
            
            # Draw FPS on frame
            cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.video_label.setPixmap(pixmap.scaled(self.video_container.size(), 
                                                Qt.AspectRatioMode.KeepAspectRatio))
            

    def receive_video_stream(self):
        """Receive video stream from vision detector."""
        while self.running:
            try:
                # Receive frame size first
                size_data, _ = self.video_socket.recvfrom(1024)
                frame_size = struct.unpack('I', size_data)[0]
                
                # Receive frame data
                frame_data = b''
                while len(frame_data) < frame_size:
                    chunk, _ = self.video_socket.recvfrom(8192)
                    frame_data += chunk
                
                # Decode frame
                nparr = np.frombuffer(frame_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                # Update GUI in thread-safe way
                QMetaObject.invokeMethod(self, "update_video_display",
                                       Qt.ConnectionType.QueuedConnection,
                                       Q_ARG(object, frame))
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving video stream: {e}")

    def receive_distance_data(self):
        """Receive distance data from vision detector."""
        while self.running:
            try:
                data, _ = self.distance_socket.recvfrom(1024)
                distance = struct.unpack('f', data)[0]
                # Update GUI in thread-safe way
                QMetaObject.invokeMethod(self, "update_distance_display",
                                       Qt.ConnectionType.QueuedConnection,
                                       Q_ARG(float, distance))
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving distance data: {e}")
    
    @pyqtSlot(float)
    def update_distance_display(self, distance):
        """Update distance display and warning."""
        self.distance_label.setText(f"Distance: {distance:.2f}m")
        if distance < 2:
            if not self.blink_timer.isActive():
                self.blink_timer.start(200)  # Blink every 200ms
        else:
            self.blink_timer.stop()
            self.warning_label.hide()

    def toggle_warning(self):
        """Toggle warning label visibility."""
        self.warning_visible = not self.warning_visible
        self.warning_label.setVisible(self.warning_visible)

    def update_plane(self, tracked_objects, depth_map):
        """Update plane visualization."""
        self.plane_widget.update_data(tracked_objects, depth_map)

    def update_map(self, latitude, longitude):
        """Update the map view to center on the given coordinates."""
        url = f"https://www.google.com/maps/search/?api=1&query={latitude},{longitude}"
        self.map_view.setUrl(QUrl(url))

    def closeEvent(self, event):
        """Clean up resources when closing."""
        self.running = False
        self.control_socket.close()
        self.video_socket.close()
        self.distance_socket.close()
        self.blink_timer.stop()
        event.accept()



if __name__ == "__main__":
    import sys
    
    app = QApplication(sys.argv)
    gui = VideoStreamGUI()
    gui.show()
    sys.exit(app.exec())