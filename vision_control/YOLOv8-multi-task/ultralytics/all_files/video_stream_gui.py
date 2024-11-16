from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSizePolicy, QFrame
from PyQt6.QtGui import QImage, QPixmap, QFont, QColor
from PyQt6.QtCore import Qt, QUrl, QTimer
from PyQt6.QtWebEngineWidgets import QWebEngineView
from plane_visualization import PlaneVisualizationWidget

class VideoStreamGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.warning_visible = False
        self.blink_timer = QTimer(self)
        self.blink_timer.timeout.connect(self.toggle_warning)

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
        self.warning_label = QLabel("WARNING", self.video_container)
        self.warning_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.warning_label.setStyleSheet("background-color: yellow; color: black; font-size: 24px; font-weight: bold; padding: 10px; border: 2px solid black;")
        self.warning_label.hide()

        # Info panel with brake display
        info_panel = QWidget()
        info_layout = QHBoxLayout(info_panel)
        left_layout.addWidget(info_panel, 1)  # Allocate 10% of the height

        # Create status labels with consistent styling
        self.distance_label = QLabel("Distance: N/A")
        self.speed_label = QLabel("Speed: N/A")
        self.brake_label = QLabel("Brake: N/A")  # New brake label
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
        right_layout.addWidget(self.map_view, 5)  # Allocate 50% of the height
        self.map_view.hide()
        self.map_view.loadFinished.connect(self.on_map_loaded)

        # 2D plane visualization
        self.plane_widget = PlaneVisualizationWidget()
        self.plane_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.plane_widget.setStyleSheet("background-color: #c0cffa; border-radius: 5px;")
        right_layout.addWidget(self.plane_widget, 5)  # Allocate 50% of the height

        self.showMaximized()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_video_layout()

    def update_video_layout(self):
        # Update video label size
        self.video_label.setGeometry(self.video_container.rect())
        
        # Update warning label size and position
        warning_width = self.video_container.width()
        warning_height = 50  # Fixed height
        self.warning_label.setFixedSize(warning_width, warning_height)
        warning_x = 0  # Align to the left edge of the video container
        warning_y = 10  # 20 pixels from the top
        self.warning_label.move(warning_x, warning_y)

    def on_map_loaded(self):
        self.map_loading_label.hide()
        self.map_view.show()

    def update_frame(self, frame, distance=None, velocity=None, brake=None, adas_active=True):
        # Update video frame
        if frame is not None:
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.video_label.setPixmap(pixmap.scaled(self.video_container.size(), Qt.AspectRatioMode.KeepAspectRatio))

        # Update info panel
        if distance is not None:
            self.distance_label.setText(f"Distance: {distance:.2f}m")
            # Update warning display for close distances
            if distance < 2:
                if not self.blink_timer.isActive():
                    self.blink_timer.start(200)
            else:
                self.blink_timer.stop()
                self.warning_label.hide()
                
        if velocity is not None:
            self.speed_label.setText(f"Speed: {velocity:.2f} km/h")
            
        if brake is not None:
            self.brake_label.setText(f"Brake: {brake:.2f}%")
            # Add color coding for brake status
            if brake > 80:
                self.brake_label.setStyleSheet("font-size: 16px; padding: 5px; background-color: #ff4d4d; color: white; border-radius: 5px;")
            elif brake > 50:
                self.brake_label.setStyleSheet("font-size: 16px; padding: 5px; background-color: #ffa64d; color: white; border-radius: 5px;")
            else:
                self.brake_label.setStyleSheet("font-size: 16px; padding: 5px; background-color: #c0cffa; border-radius: 5px;")
        
        # Update ADAS status
        if adas_active:
            self.adas_label.setText("ADAS: Activated")
            self.adas_label.setStyleSheet("font-size: 16px; padding: 5px; background-color: #008000; color: white; border-radius: 5px;")
        else:
            self.adas_label.setText("ADAS: Deactivated")
            self.adas_label.setStyleSheet("font-size: 16px; padding: 5px; background-color: #FF0000; color: white; border-radius: 5px;")

    def toggle_warning(self):
        self.warning_visible = not self.warning_visible
        self.warning_label.setVisible(self.warning_visible)

    def update_plane(self, tracked_objects, depth_map):
        self.plane_widget.update_data(tracked_objects, depth_map)

    def update_map(self, latitude, longitude):
        # Update the map view to center on the given coordinates
        url = f"https://www.google.com/maps/search/?api=1&query={latitude},{longitude}"
        self.map_view.setUrl(QUrl(url))

    def closeEvent(self, event):
        self.blink_timer.stop()
        event.accept()

if __name__ == "__main__":
    import sys
    
    app = QApplication(sys.argv)
    gui = VideoStreamGUI()
    gui.show()
    sys.exit(app.exec())