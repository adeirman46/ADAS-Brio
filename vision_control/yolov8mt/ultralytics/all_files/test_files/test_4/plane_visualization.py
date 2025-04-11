from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QColor, QBrush, QPen, QFont, QPolygonF
from PyQt6.QtCore import Qt, QRectF, QSize, QPointF
import numpy as np
import time

class TrackedVehicle:
    def __init__(self, x, y, depth, track_id):
        self.track_id = track_id
        self.positions = [(x, y, depth)]
        self.smoothed_x = x
        self.smoothed_y = y
        self.smoothed_depth = depth
        self.last_update = time.time()
        
    def update_position(self, x, y, depth):
        current_time = time.time()
        self.positions.append((x, y, depth))
        if len(self.positions) > 5:
            self.positions.pop(0)
        
        alpha = 0.3
        self.smoothed_x = alpha * x + (1 - alpha) * self.smoothed_x
        self.smoothed_y = alpha * y + (1 - alpha) * self.smoothed_y
        self.smoothed_depth = alpha * depth + (1 - alpha) * self.smoothed_depth
        self.last_update = current_time

class PlaneVisualizationWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.tracked_objects = []
        self.max_length = 15  # Maximum depth in meters
        self.max_width = 5    # Maximum width in meters
        self.setMinimumSize(400, 400)
        
        self.vehicles = {}
        self.vehicle_timeout = 1.0
        
        # Colors
        self.background_color = QColor(240, 240, 245)  # Light gray background
        self.grid_color = QColor(200, 200, 200)       # Grid lines
        self.axis_color = QColor(150, 150, 150)       # Main axes
        self.vehicle_colors = {
            'ego': QColor(46, 204, 113),     # Green for ego vehicle
            'tracked': QColor(52, 152, 219)   # Blue for tracked vehicles
        }
        
        # Cached pens
        self.grid_pen = QPen(self.grid_color, 1, Qt.PenStyle.DashLine)
        self.axis_pen = QPen(self.axis_color, 2, Qt.PenStyle.SolidLine)

    def draw_car_symbol(self, painter, x, y, scale=1.0, color=None, is_ego=False):
        """Draw a car-shaped symbol."""
        if color is None:
            color = self.vehicle_colors['ego'] if is_ego else self.vehicle_colors['tracked']
        
        # Base size for the car
        width = 30 * scale
        length = 50 * scale
        
        painter.save()
        painter.translate(x, y)
        
        # Draw shadow
        if not is_ego:
            shadow_offset = 2
            shadow_color = QColor(0, 0, 0, 30)
            painter.translate(shadow_offset, shadow_offset)
            self._draw_car_shape(painter, width, length, shadow_color)
            painter.translate(-shadow_offset, -shadow_offset)
        
        # Draw main car shape
        self._draw_car_shape(painter, width, length, color)
        
        # Add details for ego vehicle
        if is_ego:
            # Add directional arrow
            painter.setPen(QPen(Qt.GlobalColor.white, 2))
            arrow_size = width * 0.4
            painter.drawLine(0, -arrow_size, 0, arrow_size)
            painter.drawLine(-arrow_size, 0, arrow_size, 0)
        
        painter.restore()

    def _draw_car_shape(self, painter, width, length, color):
        """Draw the actual car polygon shape."""
        # Create car outline points
        car_shape = QPolygonF([
            QPointF(-width/2, -length/2),    # Back left
            QPointF(width/2, -length/2),     # Back right
            QPointF(width/2, length/3),      # Side right
            QPointF(width/3, length/2),      # Front right
            QPointF(-width/3, length/2),     # Front left
            QPointF(-width/2, length/3),     # Side left
        ])
        
        # Draw car body
        painter.setBrush(QBrush(color))
        painter.setPen(QPen(color.darker(120), 2))
        painter.drawPolygon(car_shape)
        
        # Add windshield
        windshield_color = color.darker(130)
        windshield = QPolygonF([
            QPointF(-width/3, length/6),
            QPointF(width/3, length/6),
            QPointF(width/4, length/3),
            QPointF(-width/4, length/3),
        ])
        painter.setBrush(QBrush(windshield_color))
        painter.setPen(QPen(windshield_color.darker(110), 1))
        painter.drawPolygon(windshield)

    def draw_grid(self, painter, width, height):
        """Draw coordinate grid with distance markers."""
        # Set up coordinate transformation
        painter.save()
        painter.translate(width/2, height)
        painter.scale(1, -1)  # Flip Y axis to make positive go up
        
        # Calculate grid spacing
        grid_spacing_y = height / self.max_length
        grid_spacing_x = width / (2 * self.max_width)
        
        # Draw horizontal grid lines and distance markers
        painter.setPen(self.grid_pen)
        for i in range(self.max_length + 1):
            y = i * grid_spacing_y
            painter.drawLine(-width/2, y, width/2, y)
            
            # Draw distance markers
            painter.save()
            painter.scale(1, -1)  # Flip back for text
            painter.setPen(self.axis_color)
            painter.drawText(-width/2 + 10, -y + 15, f"{i}m")
            painter.restore()
        
        # Draw vertical grid lines
        for i in range(-self.max_width, self.max_width + 1):
            x = i * grid_spacing_x
            painter.drawLine(x, 0, x, height)
        
        # Draw main axes with thicker lines
        painter.setPen(self.axis_pen)
        painter.drawLine(0, 0, 0, height)  # Y axis
        painter.drawLine(-width/2, 0, width/2, 0)  # X axis
        
        painter.restore()

    def update_tracked_objects(self, tracked_objects):
        if tracked_objects is not None and isinstance(tracked_objects, np.ndarray):
            current_time = time.time()
            tracked_ids = set()
            
            for obj in tracked_objects:
                if len(obj) >= 4:
                    cx, cy, depth, track_id = obj
                    tracked_ids.add(track_id)
                    
                    if track_id in self.vehicles:
                        self.vehicles[track_id].update_position(cx, cy, depth)
                    else:
                        self.vehicles[track_id] = TrackedVehicle(cx, cy, depth, track_id)
            
            # Remove stale vehicles
            to_remove = [track_id for track_id, vehicle in self.vehicles.items()
                        if track_id not in tracked_ids and 
                        current_time - vehicle.last_update > self.vehicle_timeout]
            
            for track_id in to_remove:
                del self.vehicles[track_id]
            
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        width = self.width()
        height = self.height()
        
        # Fill background
        painter.fillRect(0, 0, width, height, self.background_color)
        
        # Draw grid
        self.draw_grid(painter, width, height)
        
        # Set up coordinate system for vehicles
        painter.save()
        painter.translate(width/2, height)
        painter.scale(1, -1)
        
        # Draw tracked vehicles
        scale_y = height / self.max_length
        scale_x = width / (2 * self.max_width)
        
        for vehicle in self.vehicles.values():
            if vehicle.smoothed_depth <= self.max_length:
                # Calculate scaled positions
                x_offset = (vehicle.smoothed_x - 640) / 640 * self.max_width
                x = x_offset * scale_x
                y = vehicle.smoothed_depth * scale_y
                
                # Calculate scale based on depth
                scale = 1 - (vehicle.smoothed_depth / self.max_length) * 0.3
                
                # Draw vehicle
                self.draw_car_symbol(painter, x, y, scale)
                
                # Draw ID
                painter.save()
                painter.scale(1, -1)
                painter.setPen(Qt.GlobalColor.black)
                painter.drawText(int(x - 10), int(-y - 30), f'{int(vehicle.track_id)}')
                painter.restore()
        
        # Draw ego vehicle at bottom center
        self.draw_car_symbol(painter, 0, 0, 1.0, is_ego=True)
        
        painter.restore()

    def sizeHint(self):
        return QSize(400, 400)