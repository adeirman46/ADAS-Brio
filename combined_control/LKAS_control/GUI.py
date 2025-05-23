from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QSizePolicy, QFrame, QProgressBar, QComboBox)
from PyQt6.QtGui import QImage, QPixmap, QFont, QColor, QPainter, QRadialGradient, QBrush
from PyQt6.QtCore import Qt, QTimer, pyqtSlot, QMetaObject, Q_ARG, QSize, QRectF, QPropertyAnimation, QEasingCurve
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

class CircularGauge(QLabel):
    def __init__(self, label_text, max_value, parent=None):
        super().__init__(parent)
        self.label_text = label_text
        self.max_value = max_value
        self.value = 0
        self.setFixedSize(150, 150)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet("background-color: transparent; color: #00ffcc; font-size: 14px; font-weight: bold;")

    def setValue(self, value):
        self.value = max(0, min(value, self.max_value))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        if not painter.isActive():
            return

        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Draw background circle
        painter.setPen(Qt.PenStyle.NoPen)  # Fixed: Use Qt.PenStyle.NoPen instead of Qt.NoPen
        painter.setBrush(QColor(30, 30, 50))
        painter.drawEllipse(5, 5, 140, 140)

        # Draw gauge arc
        angle = (self.value / self.max_value) * 270  # 270 degrees for 3/4 circle
        gradient = QRadialGradient(75, 75, 70)
        gradient.setColorAt(0, QColor(0, 255, 204))
        gradient.setColorAt(1, QColor(0, 128, 255))
        painter.setPen(Qt.PenStyle.NoPen)  # Fixed: Use Qt.PenStyle.NoPen
        painter.setBrush(gradient)
        painter.drawPie(15, 15, 120, 120, 90 * 16, -angle * 16)

        # Draw inner circle
        painter.setBrush(QColor(20, 20, 40))
        painter.drawEllipse(25, 25, 100, 100)

        # Draw text
        painter.setPen(QColor(0, 255, 204))
        painter.setFont(QFont("Arial", 12, QFont.Weight.Bold))
        painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, 
                        f"{self.label_text}\n{self.value:.1f}")

        painter.end()

class VideoStreamGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        # Initialize current_mode before calling initUI
        self.current_mode = "ACC"
        self.initUI()
        self.warning_visible = False
        self.blink_timer = QTimer(self)
        self.blink_timer.timeout.connect(self.toggle_warning)
        self.fps_counter = FPSCounter()

        # Initialize sockets and start threads
        self.init_sockets()
        self.start_reception_threads()

        # Socket for sending mode to vision_detector.py
        self.mode_send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mode_send_address = ('localhost', 12360)

    def initUI(self):
        self.setWindowTitle("Advanced Driver Assistance System")
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1a1a2e;
                color: #00ffcc;
            }
            QLabel {
                color: #00ffcc;
                font-family: 'Arial';
            }
        """)
        self.resize(1280, 720)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # Left side (video and info panel)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setSpacing(5)
        main_layout.addWidget(left_widget, 8)

        # Video container with main feed and overlaid DMS
        self.video_container = QFrame()
        self.video_container.setLineWidth(3)
        self.video_container.setStyleSheet("""
            background-color: #2a2a4e;
            border: 3px solid transparent;
            border-radius: 10px;
            border-image: linear-gradient(to right, #00ffcc, #ff00cc) 1;
        """)
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

        # DMS overlay (floating effect with border instead of box-shadow)
        self.dms_label = QLabel(self.video_label)
        self.dms_label.setFixedSize(320, 240)
        self.dms_label.setStyleSheet("""
            background-color: rgba(0, 0, 0, 80);
            border: 3px solid rgba(0, 255, 204, 0.5);
            border-radius: 10px;
        """)

        # Warning label (bottom center with neon glow using border)
        self.warning_label = QLabel("WARNING: OBSTACLE AHEAD", self.video_label)
        self.warning_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.warning_label.setStyleSheet("""
            background-color: rgba(255, 0, 0, 0.8);
            color: white;
            font-size: 24px;
            font-weight: bold;
            padding: 10px;
            border: 3px solid rgba(255, 51, 51, 0.7);
            border-radius: 5px;
        """)
        self.warning_label.setFixedSize(400, 50)
        self.warning_label.hide()

        # Info panel
        info_panel = QWidget()
        info_layout = QHBoxLayout(info_panel)
        info_layout.setSpacing(10)
        left_layout.addWidget(info_panel, 1)

        # Status labels with modern styling
        labels = {
            'gps': "GPS: Waiting for data...",
            'distance': "Distance: N/A",
            'adas': "ADAS: Activated"
        }

        status_style = """
            font-size: 16px; 
            padding: 5px; 
            background-color: #2a2a4e; 
            border: 2px solid rgba(0, 255, 204, 0.5); 
            border-radius: 5px;
        """
        adas_style = """
            font-size: 16px; 
            padding: 5px; 
            background-color: #008000; 
            color: white; 
            border: 2px solid rgba(0, 255, 0, 0.5); 
            border-radius: 5px;
        """

        for key, text in labels.items():
            label = QLabel(text)
            setattr(self, f"{key}_label", label)
            label.setStyleSheet(adas_style if key == 'adas' else status_style)
            info_layout.addWidget(label)

        # Right side (Enhanced status panel and plane visualization)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(10)
        main_layout.addWidget(right_widget, 4)

        # Mode selection dropdown
        mode_widget = QWidget()
        mode_layout = QHBoxLayout(mode_widget)
        mode_label = QLabel("Select Mode:")
        mode_label.setStyleSheet("""
            font-size: 18px; 
            font-weight: bold; 
            padding: 5px;
        """)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["ACC", "AEB", "LKAS"])
        self.mode_combo.setStyleSheet("""
            QComboBox {
                background-color: #2a2a4e;
                color: #00ffcc;
                border: 2px solid #ff00cc;
                border-radius: 8px;
                padding: 5px;
                font-size: 16px;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox QAbstractItemView {
                background-color: #2a2a4e;
                color: #00ffcc;
                selection-background-color: #ff00cc;
                selection-color: white;
            }
        """)
        self.mode_combo.currentIndexChanged.connect(self.on_mode_changed)
        mode_layout.addWidget(mode_label)
        mode_layout.addWidget(self.mode_combo)
        right_layout.addWidget(mode_widget)

        # Mode indicator
        self.mode_label = QLabel(f"Mode: {self.current_mode}")
        self.mode_label.setStyleSheet("""
            font-size: 18px; 
            font-weight: bold; 
            padding: 10px; 
            background-color: #2a2a4e; 
            border: 3px solid rgba(255, 0, 204, 0.5); 
            border-radius: 8px;
        """)
        right_layout.addWidget(self.mode_label)

        # Status panel with gauges
        status_panel = QWidget()
        status_layout = QHBoxLayout(status_panel)
        status_layout.setSpacing(10)
        right_layout.addWidget(status_panel, 3)

        # Speed gauge
        self.speed_gauge = CircularGauge("Speed (km/h)", 100)
        status_layout.addWidget(self.speed_gauge)

        # Brake gauge
        self.brake_gauge = CircularGauge("Brake (%)", 100)
        status_layout.addWidget(self.brake_gauge)

        # Distance progress bar
        distance_widget = QWidget()
        distance_layout = QVBoxLayout(distance_widget)
        distance_layout.setSpacing(5)
        self.distance_bar = QProgressBar()
        self.distance_bar.setMaximum(20)  # Max distance 20m
        self.distance_bar.setOrientation(Qt.Orientation.Vertical)
        self.distance_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #00ffcc;
                border-radius: 5px;
                background-color: #2a2a4e;
                text-align: center;
                color: white;
                font-size: 12px;
            }
            QProgressBar::chunk {
                background-color: qlineargradient(x1:0, y1:0, x2:0, y2:1, 
                    stop:0 #ff3333, stop:0.5 #ffa500, stop:1 #00ffcc);
                border-radius: 5px;
            }
        """)
        distance_layout.addWidget(self.distance_bar)
        status_layout.addWidget(distance_widget)

        # 2D plane visualization with enhanced styling
        self.plane_widget = PlaneVisualizationWidget()
        self.plane_widget.setSizePolicy(QSizePolicy.Policy.Expanding, 
                                    QSizePolicy.Policy.Expanding)
        self.plane_widget.setStyleSheet("""
            background-color: #2a2a4e; 
            border: 3px solid rgba(0, 255, 204, 0.5); 
            border-radius: 10px;
        """)
        right_layout.addWidget(self.plane_widget, 7)

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
                ('gps_socket', 12351)         # GPS data from MIMO controller
            ]

            for socket_name, port in socket_configs:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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
            # Add FPS text with neon glow
            cv2.putText(frame, f'FPS: {fps:.1f}', (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 204), 2, 
                    lineType=cv2.LINE_AA)
            
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
        """Update speed and brake gauges."""
        self.speed_gauge.setValue(speed)
        self.brake_gauge.setValue(brake)

        # Update mode label based on brake value (for display purposes)
        if brake > 80:
            self.current_mode = "AEB"
        elif speed > 0:
            self.current_mode = self.mode_combo.currentText()  # Use selected mode
        else:
            self.current_mode = "ACC"
        
        self.mode_label.setText(f"Mode: {self.current_mode}")
        # Add a pulsing animation to the mode label
        anim = QPropertyAnimation(self.mode_label, b"geometry")
        anim.setDuration(300)
        anim.setStartValue(self.mode_label.geometry())
        anim.setEndValue(self.mode_label.geometry().adjusted(0, 0, 0, 0))
        anim.setEasingCurve(QEasingCurve.Type.InOutQuad)
        anim.start()

    @pyqtSlot(float)
    def update_distance_display(self, distance):
        """Update distance display and warning."""
        self.distance_label.setText(f"Distance: {distance:.2f}m")
        self.distance_bar.setValue(distance)
        
        # Show warning for close distances
        if distance < 2:
            if not self.blink_timer.isActive():
                self.blink_timer.start(200)  # Blink every 200ms
        else:
            self.blink_timer.stop()
            self.warning_label.hide()

    def toggle_warning(self):
        """Toggle warning label visibility with a pulsing effect."""
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

    def on_mode_changed(self, index):
        """Handle mode selection change and send to vision_detector.py."""
        mode_map = {
            "ACC": 0,
            "AEB": 1,
            "LKAS": 2
        }
        selected_mode = self.mode_combo.currentText()
        mode_value = mode_map[selected_mode]
        try:
            mode_bytes = struct.pack('i', mode_value)
            self.mode_send_socket.sendto(mode_bytes, self.mode_send_address)
            print(f"Sent mode to vision_detector.py: {selected_mode} ({mode_value})")
        except Exception as e:
            print(f"Error sending mode: {e}")

    def closeEvent(self, event):
        """Clean up resources when closing."""
        self.running = False
        
        # Close all sockets
        sockets = [
            'video_socket', 'dms_socket', 'control_socket', 
            'distance_socket', 'tracked_objects_socket', 'gps_socket',
            'mode_send_socket'
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