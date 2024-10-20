import cv2
import numpy as np

class Visualizer:
    @staticmethod
    def draw_trapezoid(frame):
        """Draw a trapezoid on the frame to represent the region of interest."""
        height, width = frame.shape[:2]
        top_width = int(width * 0.05)
        bottom_width = int(width * 0.5)
        top_x = (width - top_width) // 2
        bottom_x = (width - bottom_width) // 2
        points = np.array([
            [top_x, 0],
            [top_x + top_width, 0],
            [bottom_x + bottom_width, height],
            [bottom_x, height]
        ], np.int32)
        points = points.reshape((-1, 1, 2))
        mask = np.zeros_like(frame)
        cv2.fillConvexPoly(mask, points, (255, 255, 255))
        return cv2.bitwise_and(frame, mask)

    @staticmethod
    def draw_detection(frame, x1, y1, x2, y2):
        """Draw a bounding box for a detection."""
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    @staticmethod
    def draw_tracking(frame, tracked_object):
        """Draw tracking information on the frame."""
        x1, y1, x2, y2, track_id = tracked_object
        track_label = f'Track ID: {int(track_id)}'
        cv2.putText(frame, track_label, (int(x1), int(y1) - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)

    @staticmethod
    def draw_segmentation(frame, mask, color_index):
        """Draw segmentation mask on the frame."""
        color_mask = np.zeros_like(frame)
        if color_index == 0:
            color_mask[:, :, 1] = mask * 255  # Green for the first mask
        elif color_index == 1:
            color_mask[:, :, 2] = mask * 255  # Red for the second mask
        
        mask_indices = np.where(mask > 0)
        if len(mask_indices[0]) > 0 and len(mask_indices[1]) > 0:
            cx = int(np.mean(mask_indices[1]))
            cy = int(np.mean(mask_indices[0]))
            return color_mask, cx, cy
        return color_mask, None, None

    @staticmethod
    def overlay_segmentation(frame, color_mask):
        """Overlay segmentation mask on the frame."""
        alpha = 0.5
        frame[np.any(color_mask != 0, axis=-1)] = (1 - alpha) * frame[np.any(color_mask != 0, axis=-1)] + alpha * color_mask[np.any(color_mask != 0, axis=-1)]
        return frame

    @staticmethod
    def draw_distance_and_velocity(frame, distance, velocity, actual_velocity, actual_brake, desired_brake, state_brake):
        """Draw distance and velocity information on the frame."""
        # cv2.putText(frame, f'Distance: {distance:.2f}m', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f'Desired Velocity: {velocity:.2f}km/h', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # cv2.putText(frame, f'Actual Velocity: {actual_velocity:.2f}km/h', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f'Desired Brake: {desired_brake:.2f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f'Actual Brake: {actual_brake:.2f}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # cv2.putText(frame, f'State Brake: {state_brake}', (10, 360), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    @staticmethod
    def create_2d_plane(tracked_objects, depth_map, max_length=15, max_width=5, plane_size=(400, 400)):
        """Create a 2D plane visualization of tracked objects."""
        plane_height, plane_width = plane_size
        plane = np.zeros((plane_height, plane_width, 3), dtype=np.uint8)
        plane[:] = (192, 207, 250)  # Light blue background

        # Calculate pixels per meter
        pixels_per_meter_y = plane_height / max_length
        pixels_per_meter_x = plane_width / max_width

        # Draw grid lines
        grid_spacing = 1  # meters
        for i in range(0, plane_width, int(grid_spacing * pixels_per_meter_x)):
            cv2.line(plane, (i, 0), (i, plane_height), (235, 238, 247), 1)
        for i in range(0, plane_height, int(grid_spacing * pixels_per_meter_y)):
            cv2.line(plane, (0, i), (plane_width, i), (235, 238, 247), 1)

        # Draw center line
        cv2.line(plane, (plane_width // 2, 0), (plane_width // 2, plane_height), (100, 100, 100), 2)

        # Draw ego vehicle at the bottom center
        ego_y = plane_height - 20
        ego_x = plane_width // 2

        # Plot tracked objects
        for tracked_object in tracked_objects:
            if len(tracked_object) >= 5:
                x1, y1, x2, y2, track_id = tracked_object[:5]
                cx, cy = int((x1 + x2) // 2), int(y2)
                
                if 0 <= cx < depth_map.get_width() and 0 <= cy < depth_map.get_height():
                    try:
                        depth = depth_map.get_value(cx, cy)[1]
                        
                        if np.isfinite(depth) and depth <= max_length:
                            x_distance = (cx - depth_map.get_width() / 2) * depth / depth_map.get_width()
                            
                            # Scale to plane size
                            if abs(x_distance) <= max_width / 2:
                                plane_x = int(ego_x + (x_distance / (max_width / 2)) * (plane_width / 2))
                                plane_y = int(ego_y - (depth / max_length) * (plane_height - 40))
                                
                                # Draw object on plane as a rectangle
                                rect_width = int(0.5 * pixels_per_meter_x)
                                rect_height = int(1 * pixels_per_meter_y)
                                rect_width = max(rect_width, 20)  # Minimum width of 20 pixels
                                rect_height = max(rect_height, 40)  # Minimum height of 40 pixels
                                rect_x = plane_x - rect_width // 2
                                rect_y = plane_y - rect_height // 2
                                cv2.rectangle(plane, (rect_x, rect_y), (rect_x + rect_width, rect_y + rect_height), (70, 118, 250), -1)
                                
                                # Add text label
                                cv2.putText(plane, f'{int(track_id)}', (plane_x - 10, plane_y + 5), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 255), 1)
                    except Exception as e:
                        print(f"Error processing tracked object: {e}")

        # Add distance markers
        for i in range(grid_spacing, max_length + 1, grid_spacing):
            y = int(ego_y - i * pixels_per_meter_y)
            cv2.putText(plane, f'{i}m', (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (70, 118, 250), 1)

        # Add width markers
        for i in range(-int(max_width/2), int(max_width/2) + 1):
            if i != 0:
                x = int(ego_x + i * pixels_per_meter_x)
                cv2.putText(plane, f'{i}m', (x - 15, plane_height - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (70, 118, 250), 1)

        return plane