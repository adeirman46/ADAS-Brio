from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QColor, QBrush, QPen, QFont, QLinearGradient, QPolygon, QTransform
from PyQt6.QtCore import Qt, QRectF, QSize, QPoint, QPointF
import numpy as np
from collections import defaultdict
import time

class TrackedVehicle:
    def __init__(self, x, y, depth, track_id):
        self.track_id = track_id
        self.positions = [(x, y, depth)]  # List of recent positions
        self.smoothed_x = x
        self.smoothed_y = y
        self.smoothed_depth = depth
        self.last_update = time.time()
        
    def update_position(self, x, y, depth):
        current_time = time.time()
        self.positions.append((x, y, depth))
        # Keep only last 5 positions for smoothing
        if len(self.positions) > 5:
            self.positions.pop(0)
            
        # Apply exponential moving average for smoothing
        alpha = 0.3  # Smoothing factor
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
        
        # Track vehicle objects with smoothing
        self.vehicles = {}  # Dictionary to store TrackedVehicle objects
        self.vehicle_timeout = 1.0  # Remove vehicles not updated for 1 second
        
        # Cache colors for better performance
        self.road_color = QColor(64, 64, 64)      # Dark gray for road
        self.lane_color = QColor(255, 255, 255)   # White for lanes
        self.sky_color_top = QColor(135, 206, 235)  # Light blue
        self.sky_color_bottom = QColor(224, 255, 255)  # Lighter blue
        self.vehicle_colors = {
            'ego': QColor(0, 255, 0),     # Green for ego vehicle
            'tracked': QColor(70, 118, 250)  # Blue for tracked vehicles
        }
        
        # Cache pens for better performance
        self.lane_pen = QPen(self.lane_color, 2, Qt.PenStyle.SolidLine)
        self.dashed_pen = QPen(self.lane_color, 2, Qt.PenStyle.DashLine)
        self.dashed_pen.setDashPattern([10, 10])

    def update_tracked_objects(self, tracked_objects):
        """Update tracked objects with smoothing."""
        if tracked_objects is not None and isinstance(tracked_objects, np.ndarray):
            current_time = time.time()
            
            # Update existing vehicles and add new ones
            tracked_ids = set()
            for obj in tracked_objects:
                if len(obj) >= 4:  # Ensure we have cx, cy, depth, track_id
                    cx, cy, depth, track_id = obj
                    tracked_ids.add(track_id)
                    
                    if track_id in self.vehicles:
                        # Update existing vehicle
                        self.vehicles[track_id].update_position(cx, cy, depth)
                    else:
                        # Create new tracked vehicle
                        self.vehicles[track_id] = TrackedVehicle(cx, cy, depth, track_id)
            
            # Remove vehicles that haven't been updated recently
            to_remove = []
            for track_id, vehicle in self.vehicles.items():
                if (track_id not in tracked_ids and 
                    current_time - vehicle.last_update > self.vehicle_timeout):
                    to_remove.append(track_id)
            
            for track_id in to_remove:
                del self.vehicles[track_id]
            
            self.update()

    def draw_road(self, painter, width, height):
        """Draw road surface with perspective effect."""
        # Define road polygon points with perspective
        perspective_factor = 0.6
        road_width = width * 0.8
        
        road_points = QPolygon([
            QPoint(int(width/2 - road_width/2), height),  # Bottom left
            QPoint(int(width/2 + road_width/2), height),  # Bottom right
            QPoint(int(width/2 + road_width/4), 0),      # Top right
            QPoint(int(width/2 - road_width/4), 0)       # Top left
        ])
        
        # Draw road surface
        painter.setBrush(QBrush(self.road_color))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawPolygon(road_points)
        
        # Draw lane markers
        painter.setPen(self.lane_pen)
        
        # Left lane
        left_x1 = int(width/2 - road_width/4)
        left_x2 = int(width/2 - road_width/6)
        painter.drawLine(left_x1, height, left_x2, 0)
        
        # Right lane
        right_x1 = int(width/2 + road_width/4)
        right_x2 = int(width/2 + road_width/6)
        painter.drawLine(right_x1, height, right_x2, 0)
        
        # Center lane (dashed)
        painter.setPen(self.dashed_pen)
        painter.drawLine(width/2, height, width/2, 0)

    def draw_vehicle(self, painter, x, y, scale=1.0, color=None, is_ego=False):
        """Draw a vehicle with 3D-like appearance."""
        if color is None:
            color = self.vehicle_colors['ego'] if is_ego else self.vehicle_colors['tracked']
            
        # Scale vehicle size based on perspective
        base_width = 30 * scale
        base_length = 50 * scale
        
        # Create vehicle body
        painter.save()
        painter.translate(x - base_width/2, y - base_length/2)
        
        # Draw shadow
        shadow_color = QColor(0, 0, 0, 50)
        shadow_offset = 3
        shadow_rect = QRectF(shadow_offset, shadow_offset, base_width, base_length)
        painter.setBrush(shadow_color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(shadow_rect, 5, 5)
        
        # Draw main body
        painter.setBrush(QBrush(color))
        painter.setPen(Qt.PenStyle.NoPen)
        body_rect = QRectF(0, 0, base_width, base_length)
        painter.drawRoundedRect(body_rect, 5, 5)
        
        # Draw windshield
        windshield_color = QColor(50, 50, 50, 180)
        painter.setBrush(QBrush(windshield_color))
        windshield_rect = QRectF(base_width*0.1, base_length*0.2, 
                                base_width*0.8, base_length*0.3)
        painter.drawRoundedRect(windshield_rect, 3, 3)
        
        painter.restore()

    def draw_distance_markers(self, painter, width, height):
        """Draw distance markers with improved visibility."""
        painter.setPen(QPen(QColor(70, 118, 250), 1))
        font = QFont()
        font.setPointSize(10)
        painter.setFont(font)
        
        # Calculate ego vehicle position (bottom of screen)
        ego_y = height - 50
        
        # Draw markers for 0m, 5m, 10m, and 15m
        distances = [0, 5, 10, 15]
        for distance in distances:
            # Calculate y position with perspective
            # 0m should be at ego_y, 15m should be at top
            if distance == 0:
                y = ego_y
            else:
                y = ego_y - (distance / self.max_length) * (ego_y)
                
            # Draw distance label
            painter.drawText(10, int(y), f'{distance}m')
            
            # Draw subtle horizontal grid line
            grid_pen = QPen(QColor(200, 200, 200, 50))
            painter.setPen(grid_pen)
            painter.drawLine(0, int(y), width, int(y))
            
            # Reset pen for next text
            painter.setPen(QPen(QColor(70, 118, 250), 1))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        width = self.width()
        height = self.height()
        
        # Draw sky gradient
        gradient = QLinearGradient(0, 0, 0, height)
        gradient.setColorAt(0, self.sky_color_top)
        gradient.setColorAt(1, self.sky_color_bottom)
        painter.fillRect(0, 0, width, height, gradient)
        
        # Draw road
        self.draw_road(painter, width, height)
        
        # Draw tracked vehicles using smoothed positions
        ego_y = height - 50
        for vehicle in self.vehicles.values():
            if vehicle.smoothed_depth <= self.max_length:
                # Scale factor based on depth
                scale = 1 - (vehicle.smoothed_depth / self.max_length) * 0.5
                
                # Calculate x position (centered on road)
                x_offset = (vehicle.smoothed_x - 640) / 640
                plane_x = width/2 + (x_offset * width/4)
                
                # Calculate y position with perspective
                plane_y = ego_y - (vehicle.smoothed_depth / self.max_length) * (ego_y)
                
                # Draw vehicle
                self.draw_vehicle(painter, plane_x, plane_y, scale)
                
                # Draw ID
                painter.setPen(Qt.GlobalColor.white)
                painter.drawText(int(plane_x - 10), int(plane_y), f'{int(vehicle.track_id)}')
        
        # Draw distance markers and ego vehicle
        self.draw_distance_markers(painter, width, height)
        self.draw_vehicle(painter, width/2, ego_y, 1.0, is_ego=True)

    def sizeHint(self):
        return QSize(400, 400)