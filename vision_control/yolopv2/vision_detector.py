import pyzed.sl as sl
import numpy as np
import cv2
import argparse
import sys
import torch
from collections import deque
import time
from sort import Sort
from ultralytics import YOLO
import socket
import struct
import pickle

class ZEDCamera:
    def __init__(self):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        # Keep HD720 resolution but optimize settings
        self.init_params.camera_resolution = sl.RESOLUTION.HD720  # 1280x720
        self.init_params.camera_fps = 30
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Better depth quality
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        # Set depth range
        self.init_params.depth_minimum_distance = 0.3  # Minimum depth detection (in meters)
        self.init_params.depth_maximum_distance = 40  # Maximum depth detection (in meters)
        
        # Initialize image containers
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.objects = sl.Objects()

    def open(self):
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Failed to open ZED camera: {err}")
            exit(1)
            
        # Configure runtime parameters for optimal performance
        runtime_params = sl.RuntimeParameters()
        runtime_params.confidence_threshold = 50
        
        # Only set supported camera settings
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 4)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, 4)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, 4)

    def grab_frame(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve left image and depth map
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
            
            # Apply depth post-processing
            depth_data = self.depth.get_data()
            depth_data[depth_data == float('inf')] = 40.0  # Cap maximum depth
            
            return self.image.get_data(), self.depth
        return None, None

    def close(self):
        self.zed.close()

class ObjectDetector:
    def __init__(self, model_path, conf_threshold):
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold

    def detect(self, frame, iou=0.45):
        return self.model.predict(frame, conf=self.conf_threshold, iou=iou, device=[0], imgsz=(384,672), show_labels=False, save=False, stream=True)

class Visualizer:
    @staticmethod
    def draw_trapezoid(frame):
        """Draw a rectangular region of interest in front of the car view."""
        height, width = frame.shape[:2]
        # rect_width = int(width * 0.3)
        # rect_height = int(height * 0.6)
        rect_width = int(width * 0.4)
        rect_height = int(height)
        rect_x = (width - rect_width) // 2
        rect_y = height - rect_height
        
        points = np.array([
            [rect_x, rect_y],
            [rect_x + rect_width, rect_y],
            [rect_x + rect_width, height],
            [rect_x, height]
        ], np.int32)
        
        points = points.reshape((-1, 1, 2))
        mask = np.zeros_like(frame)
        cv2.fillConvexPoly(mask, points, (255, 255, 255))
        return cv2.bitwise_and(frame, mask)

    @staticmethod
    def draw_detection(frame, x1, y1, x2, y2):
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    @staticmethod
    def draw_tracking(frame, tracked_object):
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
        scale_factor = 1
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
            scale_factor = 1
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

class MainApplication:
    def __init__(self, args):
        self.camera = ZEDCamera()
        self.detector = ObjectDetector(args.model, args.conf)
        self.sort_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.45)
        self.frame_buffer = deque(maxlen=args.buffer_size)
        self.prev_time = time.time()  # For FPS calculation
        self.fps = 0
        
        # Initialize UDP socket for sending distance data
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.speed_controller_address = ('localhost', 12345)
        # Initialize UDP socket for GUI
        self.gui_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gui_video_address = ('localhost', 12347)  # New port for video stream
        self.gui_distance_address = ('localhost', 12348)  # New port for distance data

    def send_distance(self, distance):
        """Send distance data to MIMO speed controller via UDP."""
        try:
            # Pack distance as float and send
            distance_bytes = struct.pack('f', distance)
            self.socket.sendto(distance_bytes, self.speed_controller_address)
        except Exception as e:
            print(f"Error sending distance: {e}")

    def run(self):
        self.camera.open()
        
        try:
            while True:
                frame, depth = self.camera.grab_frame()
                if frame is None:
                    continue

                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
                self.frame_buffer.append((frame_rgb, depth))

                if len(self.frame_buffer) == self.frame_buffer.maxlen:
                    self.process_frame()

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.cleanup()

    def send_to_gui(self, frame, distance):
        """Send video frame and distance data to GUI."""
        try:
            # Send distance data
            distance_bytes = struct.pack('f', distance)
            self.gui_socket.sendto(distance_bytes, self.gui_distance_address)
            
            # Send video frame
            # Compress frame to reduce network load
            _, encoded_frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_bytes = encoded_frame.tobytes()
            
            # Send frame size first
            size_bytes = struct.pack('I', len(frame_bytes))
            self.gui_socket.sendto(size_bytes, self.gui_video_address)
            
            # Send frame data in chunks
            chunk_size = 8192
            for i in range(0, len(frame_bytes), chunk_size):
                chunk = frame_bytes[i:i + chunk_size]
                self.gui_socket.sendto(chunk, self.gui_video_address)
                
        except Exception as e:
            print(f"Error sending data to GUI: {e}")

    def send_tracked_objects(self, tracked_objects, depth_map):
        """Send tracked objects data to GUI for plane visualization."""
        try:
            # Create new UDP socket for tracked objects if not exists
            if not hasattr(self, 'tracked_objects_socket'):
                self.tracked_objects_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.tracked_objects_address = ('localhost', 12349)  # New port for tracked objects

            # Convert tracked objects to bytes
            tracked_objects_data = []
            for obj in tracked_objects:
                x1, y1, x2, y2, track_id = obj
                # Calculate center point
                cx = int((x1 + x2) // 2)
                cy = int(y2)  # Use bottom center point
                
                if 0 <= cx < depth_map.get_width() and 0 <= cy < depth_map.get_height():
                    depth = depth_map.get_value(cx, cy)[1]
                    if np.isfinite(depth):
                        tracked_objects_data.append([cx, cy, depth, track_id])

            # Convert to numpy array and then to bytes
            data_array = np.array(tracked_objects_data, dtype=np.float32)
            data_bytes = data_array.tobytes()

            # Send size first
            size_bytes = struct.pack('I', len(data_bytes))
            self.tracked_objects_socket.sendto(size_bytes, self.tracked_objects_address)

            # Send data in chunks
            chunk_size = 8192
            for i in range(0, len(data_bytes), chunk_size):
                chunk = data_bytes[i:i + chunk_size]
                self.tracked_objects_socket.sendto(chunk, self.tracked_objects_address)

        except Exception as e:
            print(f"Error sending tracked objects: {e}")

    def process_frame(self):
        # Frame retrieval and preprocessing
        frame_to_process, depth_to_process = self.frame_buffer.popleft()
        bitwise_frame = Visualizer.draw_trapezoid(frame_to_process)
        bitwise_frame = cv2.resize(bitwise_frame, (1280, 720))
        # bitwise_frame = cv2.resize(frame_to_process, (1280, 720))
        
        # Object detection
        results = self.detector.detect(bitwise_frame)
        detections = []
        min_distance = float('inf')
        road_distance = float('nan')  # Initialize as NaN
        plotted_img = []
        
        # Process detections and segmentation
        for result in results:
            if isinstance(result, list):
                result_ = result[0]
                boxes = result_.boxes
                plot_args = dict({'line_width': None, 'boxes': True, 'conf': True, 'labels': False})
                plotted_img.append(result_.plot(**plot_args))
                
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf = box.conf[0].item()
                    
                    if conf >= self.detector.conf_threshold:
                        detections.append([x1, y1, x2, y2, conf])
                        Visualizer.draw_detection(frame_to_process, x1, y1, x2, y2)
            else:
                plotted_img.append(result)

        # Tracking
        detections_np = np.array(detections) if detections else np.empty((0, 5))
        tracked_objects = self.sort_tracker.update(detections_np)
        
        # Draw tracking results
        for tracked_object in tracked_objects:
            Visualizer.draw_tracking(frame_to_process, tracked_object)

        # Process segmentation masks and measure road distance
        combined_img = frame_to_process.copy()
        if len(plotted_img) > 1:  # Check if we have segmentation masks
            road_mask = plotted_img[1][0].to(torch.uint8).cpu().numpy()
            color_mask, cx, cy = Visualizer.draw_segmentation(combined_img, road_mask, 0)
            
            # Get road mask coordinates
            road_coords = np.where(road_mask > 0)
            if len(road_coords[0]) > 0:
                # Sample points in the lower half of the frame
                height = road_mask.shape[0]
                valid_distances = []
                
                # Sample 5 points from 80% to 100% of frame height
                sample_points = np.linspace(int(height * 0.8), int(height * 1), 5)
                
                for y in sample_points:
                    y = int(y)
                    # Get road pixels at this height
                    road_x = road_coords[1][road_coords[0] == y]
                    if len(road_x) > 0:
                        # Use center point of road
                        x = int(np.mean(road_x))
                        if 0 <= x < depth_to_process.get_width() and 0 <= y < depth_to_process.get_height():
                            depth_value = depth_to_process.get_value(x, y)[1]
                            if np.isfinite(depth_value) and depth_value > 0:
                                valid_distances.append(depth_value)
                                
                                # Draw measurement point (for visualization)
                                cv2.circle(combined_img, (x, y), 3, (0, 255, 0), -1)
                                cv2.putText(combined_img, f'{depth_value:.1f}m', 
                                        (x + 5, y), cv2.FONT_HERSHEY_SIMPLEX, 
                                        0.5, (0, 255, 0), 2)
                
                if valid_distances:
                    # Use weighted average of valid distances
                    weights = np.linspace(1.0, 0.2, len(valid_distances))  # More weight to closer points
                    road_distance = np.average(valid_distances, weights=weights)
            
            combined_img = Visualizer.overlay_segmentation(combined_img, color_mask)

        # Use road_distance if valid, otherwise keep as NaN
        effective_distance = road_distance

        # Send distance to speed controller
        self.send_distance(effective_distance)

        # Send data to GUI
        self.send_to_gui(combined_img, effective_distance)
        self.send_tracked_objects(tracked_objects, depth_to_process)

        # Calculate FPS
        current_time = time.time()
        self.fps = 1.0 / (current_time - self.prev_time)
        self.prev_time = current_time

    def cleanup(self):
        self.socket.close()
        self.gui_socket.close()  # Close GUI socket
        self.camera.close()
        if hasattr(self, 'tracked_objects_socket'):
            self.tracked_objects_socket.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ZED YOLOv8 Object Detection")
    parser.add_argument('--model', type=str, default='../../best.pt', help='Path to the YOLOv8 model')
    parser.add_argument('--conf', type=float, default=0.7, help='Confidence threshold for object detection')
    parser.add_argument('--buffer_size', type=int, default=5, help='Size of the frame buffer')
    args = parser.parse_args()

    app = MainApplication(args)
    app.run()