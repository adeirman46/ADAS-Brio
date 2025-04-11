from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QColor, QBrush, QPen, QFont
from PyQt6.QtCore import Qt, QRectF, QSize

import numpy as np

class PlaneVisualizationWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.tracked_objects = []
        self.depth_map = None
        self.max_length = 15
        self.max_width = 5
        self.setMinimumSize(400, 400)

    def update_data(self, tracked_objects, depth_map):
        self.tracked_objects = tracked_objects
        self.depth_map = depth_map
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Set background color
        painter.fillRect(self.rect(), QColor(192, 207, 250))

        # Calculate pixels per meter
        pixels_per_meter_y = self.height() / self.max_length
        pixels_per_meter_x = self.width() / self.max_width

        # Draw grid lines
        grid_spacing = 1  # meters
        painter.setPen(QPen(QColor(235, 238, 247), 1))
        for i in range(0, self.width(), int(grid_spacing * pixels_per_meter_x)):
            painter.drawLine(i, 0, i, self.height())
        for i in range(0, self.height(), int(grid_spacing * pixels_per_meter_y)):
            painter.drawLine(0, i, self.width(), i)

        # Draw center line
        painter.setPen(QPen(QColor(100, 100, 100), 2))
        painter.drawLine(self.width() // 2, 0, self.width() // 2, self.height())

        # Draw ego vehicle at the bottom center
        ego_y = self.height() - 20
        ego_x = self.width() // 2

        # Plot tracked objects
        if self.depth_map:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(QColor(70, 118, 250)))
            font = QFont()
            font.setPointSize(8)
            painter.setFont(font)

            for tracked_object in self.tracked_objects:
                if len(tracked_object) >= 5:
                    x1, y1, x2, y2, track_id = tracked_object[:5]
                    cx, cy = int((x1 + x2) // 2), int(y2)
                    
                    if 0 <= cx < self.depth_map.get_width() and 0 <= cy < self.depth_map.get_height():
                        try:
                            depth = self.depth_map.get_value(cx, cy)[1]
                            
                            if np.isfinite(depth) and depth <= self.max_length:
                                x_distance = (cx - self.depth_map.get_width() / 2) * depth / self.depth_map.get_width()
                                
                                # Scale to plane size
                                if abs(x_distance) <= self.max_width / 2:
                                    plane_x = int(ego_x + (x_distance / (self.max_width / 2)) * (self.width() / 2))
                                    plane_y = int(ego_y - (depth / self.max_length) * (self.height() - 40))
                                    
                                    # Draw object on plane as a rectangle
                                    rect_width = max(int(0.5 * pixels_per_meter_x), 20)
                                    rect_height = max(int(1 * pixels_per_meter_y), 40)
                                    rect = QRectF(plane_x - rect_width / 2, plane_y - rect_height / 2, rect_width, rect_height)
                                    painter.drawRect(rect)
                                    
                                    # Add text label
                                    painter.setPen(Qt.GlobalColor.white)
                                    painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, f'{int(track_id)}')
                                    painter.setPen(Qt.PenStyle.NoPen)
                        except Exception as e:
                            print(f"Error processing tracked object: {e}")

        # Add distance markers
        painter.setPen(QColor(70, 118, 250))
        for i in range(grid_spacing, self.max_length + 1, grid_spacing):
            y = int(ego_y - i * pixels_per_meter_y)
            painter.drawText(10, y, f'{i}m')

        # Add width markers
        for i in range(-int(self.max_width/2), int(self.max_width/2) + 1):
            if i != 0:
                x = int(ego_x + i * pixels_per_meter_x)
                painter.drawText(x - 15, self.height() - 5, f'{i}m')

    def sizeHint(self):
        return QSize(400, 400)