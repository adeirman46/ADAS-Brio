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

class ZEDCamera:
    def __init__(self):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 30
        self.init_params.coordinate_units = sl.UNIT.METER
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.objects = sl.Objects()

    def open(self):
        if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            print("Failed to open ZED camera")
            exit(1)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1)

    def grab_frame(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
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
        rect_width = int(width * 0.3)
        rect_height = int(height * 0.6)
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

    def process_frame(self):
        # Frame retrieval and preprocessing
        frame_to_process, depth_to_process = self.frame_buffer.popleft()
        bitwise_frame = Visualizer.draw_trapezoid(frame_to_process)
        bitwise_frame = cv2.resize(bitwise_frame, (1280, 720))
        
        # Object detection
        results = self.detector.detect(bitwise_frame)
        detections = []
        min_distance = float('inf')
        half_lane_distance = None
        plotted_img = []
        
        # Process detections
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
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        if 0 <= cx < depth_to_process.get_width() and 0 <= cy < depth_to_process.get_height():
                            depth_value = depth_to_process.get_value(cx, cy)[1]
                            if np.isfinite(depth_value):
                                min_distance = min(min_distance, depth_value)
                            Visualizer.draw_detection(frame_to_process, x1, y1, x2, y2)
                            cv2.putText(frame_to_process, f'{depth_value:.2f}m', (x1, y1 - 10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            else:
                plotted_img.append(result)

        # Tracking
        detections_np = np.array(detections) if detections else np.empty((0, 5))
        tracked_objects = self.sort_tracker.update(detections_np)
        
        # Draw tracking results
        for tracked_object in tracked_objects:
            Visualizer.draw_tracking(frame_to_process, tracked_object)

        # Process segmentation masks
        combined_img = frame_to_process.copy()
        for i in range(1, len(plotted_img)):
            mask = plotted_img[i][0].to(torch.uint8).cpu().numpy()
            color_mask, cx, cy = Visualizer.draw_segmentation(combined_img, mask, i-1)
            
            if cx is not None and cy is not None:
                depth_value = depth_to_process.get_value(cx, cy)[1]
                if np.isfinite(depth_value):
                    half_lane_distance = depth_value
            
            combined_img = Visualizer.overlay_segmentation(combined_img, color_mask)

        # Calculate effective distance
        if half_lane_distance is not None:
            effective_distance = min(min_distance, 2 * half_lane_distance)
        else:
            effective_distance = min_distance

        # Send distance to speed controller
        self.send_distance(effective_distance)

        # Calculate FPS
        current_time = time.time()
        self.fps = 1.0 / (current_time - self.prev_time)
        self.prev_time = current_time

        # Display results
        cv2.putText(combined_img, f'Distance: {effective_distance:.2f}m', (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(combined_img, f'FPS: {self.fps:.2f}', (10, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("ZED + YOLOv8 Detection and Segmentation", cv2.cvtColor(combined_img, cv2.COLOR_RGB2BGR))

    def cleanup(self):
        self.socket.close()
        self.camera.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ZED YOLOv8 Object Detection")
    parser.add_argument('--model', type=str, default='best.pt', help='Path to the YOLOv8 model')
    parser.add_argument('--conf', type=float, default=0.7, help='Confidence threshold for object detection')
    parser.add_argument('--buffer_size', type=int, default=5, help='Size of the frame buffer')
    args = parser.parse_args()

    app = MainApplication(args)
    app.run()