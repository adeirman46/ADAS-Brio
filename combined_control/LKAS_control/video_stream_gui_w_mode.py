from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QSizePolicy, QFrame, QRadioButton, QButtonGroup, QAbstractButton)
from PyQt6.QtGui import QImage, QPixmap, QFont, QColor
from PyQt6.QtCore import Qt, QTimer, pyqtSlot, QMetaObject, Q_ARG, QSize
from plane_visualization import PlaneVisualizationWidget
from threading import Thread
import cv2
import numpy as np
import struct
import socket
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
        self.fps_counter = FPSCounter()

        # Initialize sockets and start threads
        self.init_sockets()
        self.start_reception_threads()

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
        main_layout.addWidget(left_widget, 7)

        # Video container with main feed and overlaid DMS
        self.video_container = QFrame()
        self.video_container.setLineWidth(2)
        self.video_container.setMidLineWidth(1)
        self.video_container.setStyleSheet("background-color: #e6e9ed;")
        self.video_container.setSizePolicy(QSizePolicy.Policy.Expanding, 
                                        QSizePolicy.Policy.Expanding)
        left_layout.addWidget(self.video_container, 9)

        # Create layout for video container
        video_layout = QHBoxLayout(self.video_container)
        video_layout.setContentsMargins(0, 0, 0, 0)
        video_layout.setSpacing(0)

        # Create a widget to hold both video label and overlays
        self.video_stack = QWidget()
        self.video_stack.setLayout(QVBoxLayout())
        self.video_stack.layout().setContentsMargins(0, 0, 0, 0)
        video_layout.addWidget(self.video_stack)

        # Main video label
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("background-color: transparent;")
        self.video_stack.layout().addWidget(self.video_label)

        # DMS overlay (positioned in top-right)
        self.dms_label = QLabel(self.video_label)
        self.dms_label.setFixedSize(320, 240)
        self.dms_label.setStyleSheet("""
            background-color: rgba(0, 0, 0, 50);
            border: 2px solid white;
            border-radius: 5px;
        """)
        
        # Warning label (positioned at bottom)
        self.warning_label = QLabel("WARNING: OBSTACLE AHEAD", self.video_label)
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
        self.warning_label.setFixedSize(400, 50)
        self.warning_label.hide()

        # Info panel
        info_panel = QWidget()
        info_layout = QHBoxLayout(info_panel)
        left_layout.addWidget(info_panel, 1)

        # Status labels with styling
        labels = {
            'gps': "GPS: Waiting for data...",
            'distance': "Distance: N/A",
            'speed': "Speed: N/A",
            'brake': "Brake: N/A",
            'adas': "ADAS: Activated"
        }

        status_style = "font-size: 16px; padding: 5px; background-color: #c0cffa; border-radius: 5px;"
        adas_style = "font-size: 16px; padding: 5px; background-color: #008000; color: white; border-radius: 5px;"

        for key, text in labels.items():
            label = QLabel(text)
            setattr(self, f"{key}_label", label)
            label.setStyleSheet(adas_style if key == 'adas' else status_style)
            info_layout.addWidget(label)

        # Right side (Plane visualization and mode selection)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        main_layout.addWidget(right_widget, 3)

        # 2D plane visualization
        self.plane_widget = PlaneVisualizationWidget()
        self.plane_widget.setSizePolicy(QSizePolicy.Policy.Expanding, 
                                    QSizePolicy.Policy.Expanding)
        self.plane_widget.setStyleSheet("background-color: #c0cffa; border-radius: 5px;")
        right_layout.addWidget(self.plane_widget, 7)

        # Mode selection panel
        mode_panel = QWidget()
        mode_layout = QVBoxLayout(mode_panel)
        mode_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        mode_panel.setStyleSheet("background-color: #c0cffa; border-radius: 5px; padding: 10px;")
        
        mode_label = QLabel("Select Mode:")
        mode_label.setStyleSheet("font-size: 16px; font-weight: bold; color: black;")
        mode_layout.addWidget(mode_label)

        self.mode_group = QButtonGroup(self)
        modes = [("ACC", 0), ("AEB", 1), ("LKAS", 2)]
        for mode_name, mode_id in modes:
            radio = QRadioButton(mode_name)
            radio.setStyleSheet("font-size: 14px; color: black;")
            self.mode_group.addButton(radio, mode_id)
            mode_layout.addWidget(radio)
            if mode_id == 0:  # Default to ACC
                radio.setChecked(True)

        right_layout.addWidget(mode_panel, 3)

        # Connect mode change signal
        self.mode_group.buttonToggled[QAbstractButton, bool].connect(self.send_mode)

        self.showMaximized()

    def resizeEvent(self, event):
        """Handle resize events for the main window."""
        super().resizeEvent(event)
        self.update_overlay_positions()

    def update_overlay_positions(self):
        """Update positions of DMS and warning overlays."""
        if hasattr(self, 'video_label') and hasattr(self, 'dms_label') and hasattr(self, 'warning_label'):
            # Position DMS in top-right corner with padding
            self.dms_label.move(
                self.video_label.width() - self.dms_label.width() - 20,
                20
            )
            # Position warning at bottom with padding
            self.warning_label.move(
                (self.video_label.width() - self.warning_label.width()) // 2,
                self.video_label.height() - self.warning_label.height() - 20
            )

    def init_sockets(self):
        """Initialize all UDP sockets."""
        try:
            socket_configs = [
                ('video_socket', 12347),      # Vision detector video stream
                ('dms_socket', 12350),        # DMS video stream
                ('control_socket', 12346),    # Speed and brake data
                ('distance_socket', 12348),   # Distance data
                ('tracked_objects_socket', 12349), # Object tracking
                ('gps_socket', 12351),        # GPS data from MIMO controller
                ('mode_socket', 12362)        # Mode selection
            ]

            for socket_name, port in socket_configs:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                if socket_name == 'mode_socket':
                    # Mode socket is for sending, not binding
                    pass
                else:
                    sock.bind(('localhost', port))
                    sock.settimeout(0.5)
                setattr(self, socket_name, sock)

        except Exception as e:
            print(f"Error initializing sockets: {e}")
            raise

    def start_reception_threads(self):
        """Start all data reception threads."""
        self.running = True
        
        thread_targets = [
            self.receive_video_stream,
            self.receive_dms_stream,
            self.receive_control_data,
            self.receive_distance_data,
            self.receive_tracked_objects,
            self.receive_gps_data
        ]

        for target in thread_targets:
            thread = Thread(target=target)
            thread.daemon = True
            thread.start()

    def receive_frame(self, socket_obj):
        """Generic frame receiver for both video streams."""
        try:
            # Receive frame size
            size_data, _ = socket_obj.recvfrom(1024)
            frame_size = struct.unpack('I', size_data)[0]
            
            # Receive frame data
            frame_data = b''
            while len(frame_data) < frame_size and self.running:
                chunk, _ = socket_obj.recvfrom(8192)
                frame_data += chunk
            
            if not self.running:
                return None
                
            nparr = np.frombuffer(frame_data, np.uint8)
            return cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
        except socket.timeout:
            return None
        except Exception as e:
            print(f"Error receiving frame: {e}")
            return None

    def receive_video_stream(self):
        """Receive video stream and FPS from vision detector."""
        while self.running:
            try:
                # Receive FPS first
                fps_data, _ = self.video_socket.recvfrom(1024)
                fps = struct.unpack('f', fps_data)[0]
                
                # Then receive frame
                frame = self.receive_frame(self.video_socket)
                if frame is not None:
                    QMetaObject.invokeMethod(self, "update_video_display",
                                        Qt.ConnectionType.QueuedConnection,
                                        Q_ARG(object, frame),
                                        Q_ARG(float, fps))
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving video data: {e}")

    @pyqtSlot(object, float)
    def update_video_display(self, frame, fps):
        """Update main video display with detector FPS."""
        if frame is not None:
            # Add FPS text
            cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(self.video_label.size(), 
                                        Qt.AspectRatioMode.KeepAspectRatio,
                                        Qt.TransformationMode.SmoothTransformation)
            self.video_label.setPixmap(scaled_pixmap)

    def receive_dms_stream(self):
        """Receive DMS video stream."""
        while self.running:
            frame = self.receive_frame(self.dms_socket)
            if frame is not None:
                QMetaObject.invokeMethod(self, "update_dms_display",
                                       Qt.ConnectionType.QueuedConnection,
                                       Q_ARG(object, frame))

    def receive_control_data(self):
        """Receive speed and brake data."""
        while self.running:
            try:
                data, _ = self.control_socket.recvfrom(1024)
                speed, brake = struct.unpack('ff', data)
                QMetaObject.invokeMethod(self, "update_control_display",
                                       Qt.ConnectionType.QueuedConnection,
                                       Q_ARG(float, speed),
                                       Q_ARG(float, brake))
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving control data: {e}")

    def receive_distance_data(self):
        """Receive distance data."""
        while self.running:
            try:
                data, _ = self.distance_socket.recvfrom(1024)
                distance = struct.unpack('f', data)[0]
                QMetaObject.invokeMethod(self, "update_distance_display",
                                       Qt.ConnectionType.QueuedConnection,
                                       Q_ARG(float, distance))
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving distance data: {e}")

    def receive_tracked_objects(self):
        """Receive tracked objects data."""
        while self.running:
            try:
                size_data, _ = self.tracked_objects_socket.recvfrom(1024)
                data_size = struct.unpack('I', size_data)[0]

                data = b''
                while len(data) < data_size and self.running:
                    chunk, _ = self.tracked_objects_socket.recvfrom(8192)
                    data += chunk

                if not self.running:
                    break

                tracked_objects = np.frombuffer(data, dtype=np.float32)
                tracked_objects = tracked_objects.reshape(-1, 4)

                QMetaObject.invokeMethod(self, "update_plane_visualization",
                                       Qt.ConnectionType.QueuedConnection,
                                       Q_ARG(object, tracked_objects))
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving tracked objects: {e}")

    def receive_gps_data(self):
        """Receive GPS data from MIMO controller."""
        while self.running:
            try:
                data, _ = self.gps_socket.recvfrom(1024)
                lat, lon = struct.unpack('ff', data)
                
                QMetaObject.invokeMethod(self, "update_gps_display",
                                       Qt.ConnectionType.QueuedConnection,
                                       Q_ARG(float, lat),
                                       Q_ARG(float, lon))
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving GPS data: {e}")

    @pyqtSlot(object)
    def update_dms_display(self, frame):
        """Update DMS video display."""
        if frame is not None:
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(self.dms_label.size(), 
                                        Qt.AspectRatioMode.KeepAspectRatio,
                                        Qt.TransformationMode.SmoothTransformation)
            self.dms_label.setPixmap(scaled_pixmap)

    @pyqtSlot(float, float)
    def update_control_display(self, speed, brake):
        """Update speed and brake displays."""
        self.speed_label.setText(f"Speed: {speed:.1f} km/h")
        self.update_brake_status(brake)

    def update_brake_status(self, brake_value):
        """Update brake label color and text based on brake value."""
        self.brake_label.setText(f"Brake: {brake_value:.1f}%")
        if brake_value > 80:
            color = "#ff4d4d"  # Red
        elif brake_value > 50:
            color = "#ffa64d"  # Orange
        else:
            color = "#c0cffa"  # Default blue
            
        self.brake_label.setStyleSheet(
            f"font-size: 16px; padding: 5px; background-color: {color}; "
            f"color: {'white' if brake_value > 50 else 'black'}; border-radius: 5px;"
        )

    @pyqtSlot(float)
    def update_distance_display(self, distance):
        """Update distance display and warning."""
        self.distance_label.setText(f"Distance: {distance:.2f}m")
        
        # Show warning for close distances
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

    @pyqtSlot(object)
    def update_plane_visualization(self, tracked_objects):
        """Update plane visualization with tracked objects."""
        if hasattr(self, 'plane_widget'):
            self.plane_widget.update_tracked_objects(tracked_objects)

    @pyqtSlot(float, float)
    def update_gps_display(self, lat, lon):
        """Update GPS display."""
        if -90 <= lat <= 90 and -180 <= lon <= 180:
            self.gps_label.setText(f"GPS: {lat:.6f}, {lon:.6f}")

    @pyqtSlot(QAbstractButton, bool)
    def send_mode(self, button, checked):
        """Send selected mode to vision detector."""
        if checked:
            mode_id = self.mode_group.id(button)
            try:
                mode_bytes = struct.pack('i', mode_id)
                self.mode_socket.sendto(mode_bytes, ('localhost', 12361))
                print(f"Sent mode: {mode_id} ({button.text()})")
            except Exception as e:
                print(f"Error sending mode: {e}")

    def closeEvent(self, event):
        """Clean up resources when closing."""
        self.running = False
        
        # Close all sockets
        sockets = [
            'video_socket', 'dms_socket', 'control_socket', 
            'distance_socket', 'tracked_objects_socket', 'gps_socket',
            'mode_socket'
        ]
        for socket_name in sockets:
            if hasattr(self, socket_name):
                getattr(self, socket_name).close()
        
        # Stop timers
        self.blink_timer.stop()
        
        event.accept()

if __name__ == "__main__":
    import sys
    
    app = QApplication(sys.argv)
    gui = VideoStreamGUI()
    gui.show()
    sys.exit(app.exec())