from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtCore import QUrl, pyqtSignal

class GISWidget(QWidget):
    error_occurred = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()
        self.setLayout(layout)

        self.webview = QWebEngineView()
        self.webview.setMinimumSize(400, 300)  # Set a minimum size
        layout.addWidget(self.webview)

        self.status_label = QLabel("Status: Initializing...")
        layout.addWidget(self.status_label)

        # Load OpenStreetMap
        self.load_map()

        # Connect loadFinished signal
        self.webview.loadFinished.connect(self.handle_load_finished)

    def load_map(self):
        url = "https://www.openstreetmap.org/export/embed.html?bbox=-0.004017,51.470240,0.012703,51.476954&layer=mapnik"
        self.webview.setUrl(QUrl(url))
        self.status_label.setText("Status: Loading map...")

    def update_location(self, latitude, longitude, zoom=15):
        # Update the map view to center on the given coordinates
        url = f"https://www.openstreetmap.org/export/embed.html?bbox={longitude-0.01},{latitude-0.01},{longitude+0.01},{latitude+0.01}&layer=mapnik&marker={latitude},{longitude}"
        self.webview.setUrl(QUrl(url))
        self.status_label.setText(f"Status: Updating location to {latitude:.6f}, {longitude:.6f}")

    def handle_load_finished(self, ok):
        if ok:
            self.status_label.setText("Status: Map loaded successfully")
        else:
            error_msg = "Failed to load the map"
            self.status_label.setText(f"Status: {error_msg}")
            self.error_occurred.emit(error_msg)

if __name__ == '__main__':
    import sys
    from PyQt6.QtWidgets import QApplication
    
    app = QApplication(sys.argv)
    gis_widget = GISWidget()
    gis_widget.show()
    sys.exit(app.exec())