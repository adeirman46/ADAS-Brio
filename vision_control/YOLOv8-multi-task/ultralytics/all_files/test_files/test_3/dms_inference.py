# import cv2
# import torch
# from ultralytics import YOLO
# import time
# import socket
# import struct
# import numpy as np

# class DMSInference:
#     def __init__(self, model_path):
#         """Initialize DMS inference system."""
#         # Initialize YOLOv8 model
#         self.model = YOLO(model_path)
        
#         # Initialize webcam
#         self.cap = cv2.VideoCapture(0)
#         if not self.cap.isOpened():
#             raise RuntimeError("Error: Could not open camera")
            
#         # Set camera resolution to something reasonable
#         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
#         # Initialize UDP socket for sending frames to GUI
#         self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.gui_address = ('localhost', 12350)  # New port for DMS stream
        
#         # Initialize FPS counter
#         self.prev_time = time.time()
#         self.fps = 0
#         self.alpha = 0.1  # Smoothing factor for FPS
        
#         # State tracking
#         self.running = True
        
#     def calculate_fps(self):
#         """Calculate smoothed FPS."""
#         current_time = time.time()
#         instantaneous_fps = 1 / (current_time - self.prev_time)
#         self.fps = (1 - self.alpha) * self.fps + self.alpha * instantaneous_fps
#         self.prev_time = current_time
#         return self.fps
        
#     def process_frame(self, frame):
#         """Process a single frame with YOLO model."""
#         # Perform inference
#         # resize frame to 640x480
#         frame_resized = cv2.resize(frame, (640, 480))

#         results = self.model.predict(frame_resized, conf=0.7, iou=0.45)

        
#         # Process detections
#         for result in results:
#             boxes = result.boxes
            
#             for box in boxes:
#                 # Get box coordinates and info
#                 x1, y1, x2, y2 = map(int, box.xyxy[0])
#                 conf = float(box.conf[0])
#                 cls_id = int(box.cls[0])
#                 cls_name = result.names[cls_id]
                
#                 # Draw bounding box
#                 cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
#                 # Create and draw label
#                 label = f'{cls_name} {conf:.2f}'
#                 text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
#                 cv2.rectangle(frame, (x1, y1 - 25), (x1 + text_size[0], y1), (0, 255, 0), -1)
#                 cv2.putText(frame, label, (x1, y1 - 5),
#                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
#         return frame
    
#     def send_frame_to_gui(self, frame):
#         """Send processed frame to GUI via UDP."""
#         try:
#             # Compress frame to reduce network load
#             _, encoded_frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
#             frame_bytes = encoded_frame.tobytes()
            
#             # Send frame size first
#             size_bytes = struct.pack('I', len(frame_bytes))
#             self.socket.sendto(size_bytes, self.gui_address)
            
#             # Send frame data in chunks
#             chunk_size = 8192
#             for i in range(0, len(frame_bytes), chunk_size):
#                 chunk = frame_bytes[i:i + chunk_size]
#                 self.socket.sendto(chunk, self.gui_address)
                
#         except Exception as e:
#             print(f"Error sending frame to GUI: {e}")
    
#     def run(self):
#         """Main processing loop."""
#         try:
#             while self.running:
#                 # Read frame
#                 ret, frame = self.cap.read()
#                 if not ret:
#                     print("Error: Could not read frame")
#                     break
                
#                 # Process frame
#                 frame = self.process_frame(frame)
                
#                 # Calculate and display FPS
#                 fps = self.calculate_fps()
#                 cv2.putText(frame, f'DMS FPS: {fps:.1f}', (10, 30),
#                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
#                 # Send frame to GUI frame rgb
#                 self.send_frame_to_gui(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                
#                 # Optional: Local display for debugging
#                 # cv2.imshow('DMS Debug View', frame)
#                 # if cv2.waitKey(1) & 0xFF == ord('q'):
#                 #     break
                
#         except KeyboardInterrupt:
#             print("\nDMS inference interrupted by user")
#         except Exception as e:
#             print(f"Error in DMS inference: {e}")
#         finally:
#             self.cleanup()
    
#     def cleanup(self):
#         """Clean up resources."""
#         self.running = False
#         self.cap.release()
#         self.socket.close()
#         cv2.destroyAllWindows()

# def main():
#     try:
#         dms = DMSInference('/home/irman/Documents/ADAS-Brio/vision_control/YOLOv8-multi-task/ultralytics/all_files/test_files/test_3/dms_model.pt')
#         dms.run()
#     except Exception as e:
#         print(f"Error in DMS main: {e}")

# if __name__ == "__main__":
#     main()

import cv2
import torch
from ultralytics import YOLO
import time
import socket
import struct
import numpy as np
from pygame import mixer  # For playing audio alerts
import os

class DMSInference:
    def __init__(self, model_path):
        """Initialize DMS inference system."""
        # Initialize YOLOv8 model
        self.model = YOLO(model_path)
        
        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Error: Could not open camera")
            
        # Set camera resolution to something reasonable
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Initialize UDP socket for sending frames to GUI
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gui_address = ('localhost', 12350)  # New port for DMS stream
        
        # Initialize FPS counter
        self.prev_time = time.time()
        self.fps = 0
        self.alpha = 0.1  # Smoothing factor for FPS
        
        # Initialize audio
        mixer.init()
        self.alert_sound = mixer.Sound('driver_alert.mp3')  # Make sure this file exists
        self.last_alert_time = 0
        self.alert_cooldown = 3.0  # Minimum seconds between alerts
        
        # State tracking
        self.running = True
        
    def calculate_fps(self):
        """Calculate smoothed FPS."""
        current_time = time.time()
        instantaneous_fps = 1 / (current_time - self.prev_time)
        self.fps = (1 - self.alpha) * self.fps + self.alpha * instantaneous_fps
        self.prev_time = current_time
        return self.fps

    def play_alert(self):
        """Play drowsiness alert with cooldown."""
        current_time = time.time()
        if current_time - self.last_alert_time >= self.alert_cooldown:
            self.alert_sound.play()
            self.last_alert_time = current_time
        
    def process_frame(self, frame):
        """Process a single frame with YOLO model."""
        # Perform inference
        # resize frame to 640x480
        frame_resized = cv2.resize(frame, (640, 480))

        results = self.model.predict(frame_resized, conf=0.7, iou=0.45)

        # Process detections
        for result in results:
            boxes = result.boxes
            
            for box in boxes:
                # Get box coordinates and info
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                cls_name = result.names[cls_id]
                
                # Check for drowsiness detection
                if cls_name.lower() == 'drowsy':
                    self.play_alert()  # Play alert sound
                
                # Draw bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Create and draw label
                label = f'{cls_name} {conf:.2f}'
                text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                cv2.rectangle(frame, (x1, y1 - 25), (x1 + text_size[0], y1), (0, 255, 0), -1)
                cv2.putText(frame, label, (x1, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        return frame
    
    def send_frame_to_gui(self, frame):
        """Send processed frame to GUI via UDP."""
        try:
            # Compress frame to reduce network load
            _, encoded_frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_bytes = encoded_frame.tobytes()
            
            # Send frame size first
            size_bytes = struct.pack('I', len(frame_bytes))
            self.socket.sendto(size_bytes, self.gui_address)
            
            # Send frame data in chunks
            chunk_size = 8192
            for i in range(0, len(frame_bytes), chunk_size):
                chunk = frame_bytes[i:i + chunk_size]
                self.socket.sendto(chunk, self.gui_address)
                
        except Exception as e:
            print(f"Error sending frame to GUI: {e}")
    
    def run(self):
        """Main processing loop."""
        try:
            while self.running:
                # Read frame
                ret, frame = self.cap.read()
                if not ret:
                    print("Error: Could not read frame")
                    break
                
                # Process frame
                frame = self.process_frame(frame)
                
                # Calculate and display FPS
                fps = self.calculate_fps()
                cv2.putText(frame, f'DMS FPS: {fps:.1f}', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Send frame to GUI frame rgb
                self.send_frame_to_gui(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                
        except KeyboardInterrupt:
            print("\nDMS inference interrupted by user")
        except Exception as e:
            print(f"Error in DMS inference: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        self.running = False
        self.cap.release()
        self.socket.close()
        mixer.quit()
        cv2.destroyAllWindows()

def main():
    try:
        dms = DMSInference('/home/irman/Documents/ADAS-Brio/vision_control/YOLOv8-multi-task/ultralytics/all_files/test_files/test_3/dms_model.pt')
        dms.run()
    except Exception as e:
        print(f"Error in DMS main: {e}")

if __name__ == "__main__":
    main()