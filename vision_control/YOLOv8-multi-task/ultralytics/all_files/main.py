import argparse
import cv2
import numpy as np
from collections import deque
import time
import torch
import signal
import sys
import os

from zed_camera import ZEDCamera
from object_detector import ObjectDetector
from visualizer import Visualizer
from serial_handler import SerialHandler
from logging_handler import LoggingHandler
from video_stream_gui import VideoStreamGUI, QApplication
from sort import Sort

class MainApplication:
    def __init__(self, args):
        self.camera = ZEDCamera()
        self.detector = ObjectDetector(args.model, args.conf)
        self.sort_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.3)
        self.frame_buffer = deque(maxlen=args.buffer_size)
        self.plane_size = (800, 800)
        
        self.serial_handler = None
        self.logging_handler = None
        
        # Try to initialize serial communication
        try:
            self.serial_handler = SerialHandler(args.serial_port, args.baudrate)
            print(f"SerialHandler initialized successfully on port {args.serial_port}")
        except Exception as e:
            print(f"Failed to initialize serial communication: {e}")
            print("Continuing without serial communication.")

        # Initialize logging regardless of serial communication status
        try:
            self.logging_handler = LoggingHandler(self.serial_handler)
            print("LoggingHandler initialized successfully")
        except Exception as e:
            print(f"Failed to initialize logging: {e}")
            print("Continuing without logging.")

        signal.signal(signal.SIGINT, self.signal_handler)

        self.adas_active = False  # Default ADAS state is active
        
        # Initialize GUI
        self.app = QApplication(sys.argv)
        self.gui = VideoStreamGUI()

    def signal_handler(self, sig, frame):
        print("Ctrl+C detected. Cleaning up...")
        self.cleanup()
        sys.exit(0)

    def run(self):
        self.camera.open()
        self.camera.enable_object_detection()
        
        if self.serial_handler:
            try:
                self.serial_handler.start()
                print("Serial communication started successfully")
            except Exception as e:
                print(f"Error starting serial communication: {e}")
        
        if self.logging_handler:
            try:
                self.logging_handler.start()
                print("Logging started successfully")
            except Exception as e:
                print(f"Error starting logging: {e}")

        try:
            while True:
                frame, depth = self.camera.grab_frame()
                if frame is None:
                    continue

                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
                self.frame_buffer.append((frame_rgb, depth))

                if len(self.frame_buffer) == self.frame_buffer.maxlen:
                    self.process_frame()

                self.app.processEvents()  # Allow GUI to update

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Cleaning up...")
        finally:
            self.cleanup()

    def cleanup(self):
        print("Cleaning up resources...")
        self.camera.close()
        if self.serial_handler:
            self.serial_handler.stop()
        if self.logging_handler:
            self.logging_handler.stop()
        self.gui.close()
        cv2.destroyAllWindows()
        print("Cleanup completed.")

    def process_frame(self):
        overall_start_time = time.time()

        # Frame retrieval
        frame_retrieval_start = time.time()
        frame_to_process, depth_to_process = self.frame_buffer.popleft()
        frame_retrieval_time = time.time() - frame_retrieval_start

        # Preprocessing
        preprocess_start = time.time()
        bitwise_frame = Visualizer.draw_trapezoid(frame_to_process)
        bitwise_frame = cv2.resize(bitwise_frame, (1280, 720))
        preprocess_time = time.time() - preprocess_start
        
        # Object detection
        detection_start = time.time()
        results = self.detector.detect(frame_to_process)
        detection_time = time.time() - detection_start

        # Post-processing and visualization
        postprocess_start = time.time()
        plotted_img = []
        detections = []
        min_bbox_distance = float('inf')
        half_lane_distance = None
        
        for result in results:
            if isinstance(result, list):
                result_ = result[0]
                boxes = result_.boxes
                plot_args = dict({'line_width': None, 'boxes': True, 'conf': True, 'labels': False})
                plotted_img.append(result_.plot(**plot_args))

                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())

                    if conf >= self.detector.conf_threshold:
                        detections.append([x1, y1, x2, y2, conf])
            else:
                plotted_img.append(result)
        postprocess_time = time.time() - postprocess_start

        # Tracking
        tracking_start = time.time()
        detections_np = np.array(detections) if detections else np.empty((0, 5))
        tracked_objects = self.sort_tracker.update(detections_np)
        tracking_time = time.time() - tracking_start

        # Final visualization
        visualization_start = time.time()
        combined_img = frame_to_process.copy()
        self.camera.retrieve_objects()

        for tracked_object in tracked_objects:
            x1, y1, x2, y2, track_id = tracked_object
            cx, cy = int((x1 + x2) // 2), int(y2)
            if 0 <= cx < depth_to_process.get_width() and 0 <= cy < depth_to_process.get_height():
                depth_value = depth_to_process.get_value(cx, cy)[1]
                if np.isfinite(depth_value):
                    min_bbox_distance = min(min_bbox_distance, depth_value)
                    Visualizer.draw_tracking(combined_img, tracked_object)
                    cv2.putText(combined_img, f'{depth_value:.2f}m', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        for i in range(1, len(plotted_img)):
            mask = plotted_img[i][0].to(torch.uint8).cpu().numpy()
            color_mask, cx, cy = Visualizer.draw_segmentation(combined_img, mask, i-1)
            
            if cx is not None and cy is not None:
                depth_value = depth_to_process.get_value(cx, cy)[1]
                if np.isfinite(depth_value):
                    half_lane_distance = depth_value
            
            combined_img = Visualizer.overlay_segmentation(combined_img, color_mask)

        if half_lane_distance is not None:
            effective_distance = min(min_bbox_distance, 2 * half_lane_distance)
        else:
            effective_distance = min_bbox_distance

        if self.serial_handler:
            self.serial_handler.update_obstacle_distance(effective_distance)

            status = self.serial_handler.get_status()
            desired_velocity = status['desired_velocity']
            desired_brake = status['desired_brake']
            actual_velocity = status['speed']
            actual_brake = status['actual_brake']
            brake_state = status['brake_state']
        else:
            desired_velocity = 0
            desired_brake = 0
            actual_velocity = 0
            actual_brake = 0
            brake_state = 'N/A'

        Visualizer.draw_distance_and_velocity(combined_img, effective_distance, desired_velocity, 
                                              actual_velocity, actual_brake, desired_brake, brake_state)
        
        visualization_time = time.time() - visualization_start

        # Calculate and display FPS
        overall_end_time = time.time()
        overall_time = (overall_end_time - overall_start_time)
        fps = 1 / overall_time
        cv2.putText(combined_img, f'FPS: {fps:.2f}', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Display timing information
        # cv2.putText(combined_img, f'Frame Retrieval: {frame_retrieval_time*1000:.2f}ms', (10, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.putText(combined_img, f'Preprocessing: {preprocess_time*1000:.2f}ms', (10, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.putText(combined_img, f'Detection: {detection_time*1000:.2f}ms', (10, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.putText(combined_img, f'Post-processing: {postprocess_time*1000:.2f}ms', (10, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.putText(combined_img, f'Tracking: {tracking_time*1000:.2f}ms', (10, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.putText(combined_img, f'Visualization: {visualization_time*1000:.2f}ms', (10, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        # cv2.putText(combined_img, f'Total: {overall_time*1000:.2f}ms', (10, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Update GUI
        self.gui.update_frame(combined_img, effective_distance, actual_velocity, 
                              "Active" if brake_state == 1 else "Inactive", 
                              self.adas_active)
        
        self.gui.update_plane(tracked_objects, depth_to_process)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ZED YOLOv8 Object Detection")
    parser.add_argument('--model', type=str, default='/home/irman/Documents/FSD-Level-1-Jetson/vision_control/YOLOv8-multi-task/ultralytics/models/yolom4_50.pt', help='Path to the YOLOv8 model')
    parser.add_argument('--conf', type=float, default=0.3, help='Confidence threshold for object detection')
    parser.add_argument('--buffer_size', type=int, default=5, help='Size of the frame buffer')
    parser.add_argument('--serial_port', type=str, default='/dev/ttyUSB0', help='Serial port for communication')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baudrate for serial communication')
    args = parser.parse_args()

    app = MainApplication(args)
    app.run()