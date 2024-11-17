import cv2
import numpy as np

class Visualizer:
    @staticmethod
    def draw_trapezoid(frame):
        """Draw a rectangular region of interest in front of the car view."""
        height, width = frame.shape[:2]
        
        # Define rectangle dimensions
        # Rectangle will be in the lower center portion of the frame
        rect_width = int(width * 0.3)  # 30% of frame width
        rect_height = int(height * 0.6)  # 60% of frame height
        
        # Calculate rectangle position
        # Centered horizontally, aligned to bottom of frame
        rect_x = (width - rect_width) // 2  # Center horizontally
        rect_y = height - rect_height  # Start from bottom
        
        # Create points for rectangle
        points = np.array([
            [rect_x, rect_y],  # Top left
            [rect_x + rect_width, rect_y],  # Top right
            [rect_x + rect_width, height],  # Bottom right
            [rect_x, height]  # Bottom left
        ], np.int32)
        
        # Reshape points for fillConvexPoly
        points = points.reshape((-1, 1, 2))
        
        # Create mask and fill rectangle
        mask = np.zeros_like(frame)
        cv2.fillConvexPoly(mask, points, (255, 255, 255))
        
        # Apply mask to frame
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
        """
        Draw segmentation mask on the frame using BDD dataset color format with optimized performance.
        color_index: 0 for direct drivable area (red), 1 for alternative drivable area (purple)
        """
        # Downsample mask for faster processing
        scale_factor = 3
        small_height, small_width = mask.shape[0] // scale_factor, mask.shape[1] // scale_factor
        small_mask = cv2.resize(mask, (small_width, small_height), interpolation=cv2.INTER_NEAREST)
        
        # Create small color mask
        small_color_mask = np.zeros((small_height, small_width, 3), dtype=np.uint8)
        
        # BDD color scheme - apply to small mask
        if color_index == 0:
            # Blue
            small_color_mask[small_mask > 0] = [117, 173, 185]  # [R, G, B]
        elif color_index == 1:
            # Red
            small_color_mask[small_mask > 0] = [63, 108, 125]  # [R, G, B]
        
        # Calculate centroid on small mask for efficiency
        mask_indices = np.where(small_mask > 0)
        if len(mask_indices[0]) > 0 and len(mask_indices[1]) > 0:
            cx = int(np.mean(mask_indices[1])) * scale_factor  # Scale back to original size
            cy = int(np.mean(mask_indices[0])) * scale_factor  # Scale back to original size
            
            # Resize color mask to original size
            color_mask = cv2.resize(small_color_mask, (frame.shape[1], frame.shape[0]), 
                                  interpolation=cv2.INTER_LINEAR)
            return color_mask, cx, cy
            
        # If no mask indices found
        color_mask = cv2.resize(small_color_mask, (frame.shape[1], frame.shape[0]), 
                              interpolation=cv2.INTER_LINEAR)
        return color_mask, None, None

    @staticmethod
    def overlay_segmentation(frame, color_mask):
        """Optimized overlay segmentation mask on the frame with BDD-style transparency."""
        try:
            # Downscale for faster processing
            scale_factor = 3
            small_frame = cv2.resize(frame, (frame.shape[1] // scale_factor, frame.shape[0] // scale_factor), 
                                   interpolation=cv2.INTER_LINEAR)
            small_color_mask = cv2.resize(color_mask, (frame.shape[1] // scale_factor, frame.shape[0] // scale_factor), 
                                        interpolation=cv2.INTER_LINEAR)

            # Create mask of non-zero pixels in small version
            small_mask_area = np.any(small_color_mask != 0, axis=-1)
            
            # Only process if there are pixels to overlay
            if np.any(small_mask_area):
                alpha = 1  # Opacity
                
                # Process blending on smaller images
                small_result = small_frame.copy()
                small_result[small_mask_area] = cv2.addWeighted(
                    small_frame[small_mask_area], 
                    1 - alpha,
                    small_color_mask[small_mask_area], 
                    alpha, 
                    0
                )
                
                # Resize result back to original size
                result = cv2.resize(small_result, (frame.shape[1], frame.shape[0]), 
                                  interpolation=cv2.INTER_LINEAR)
                return result
            
            return frame
            
        except Exception as e:
            print(f"Error in overlay_segmentation: {e}")
            return frame

    @staticmethod
    def draw_distance_and_velocity(frame, distance, velocity, actual_velocity, actual_brake, desired_brake, state_brake):
        """Draw distance and velocity information on the frame."""
        cv2.putText(frame, f'Desired Velocity: {velocity:.2f}km/h', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f'Desired Brake: {desired_brake:.2f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f'Actual Brake: {actual_brake:.2f}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


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