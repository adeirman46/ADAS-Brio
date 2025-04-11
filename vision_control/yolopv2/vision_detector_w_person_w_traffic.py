import argparse
import time
from pathlib import Path
import cv2
import torch
import pyzed.sl as sl
import numpy as np
import socket
import struct
from sort import Sort
from collections import deque
from ultralytics import YOLO  # Add YOLO import for person detection

from utils.utils import (
    time_synchronized, select_device, increment_path,
    scale_coords, non_max_suppression, split_for_trace_model,
    driving_area_mask, lane_line_mask, plot_one_box, show_seg_result,
    AverageMeter
)

class SignDisplay:
    def __init__(self):
        # Dictionary mapping sign classes to their names and colors
        self.sign_classes = {
            0: {"name": "Bus Stop", "color": (255, 196, 0)},
            1: {"name": "Do Not Enter", "color": (0, 0, 255)},
            2: {"name": "Do Not Stop", "color": (0, 0, 255)},
            3: {"name": "No Left Turn", "color": (0, 0, 255)},
            4: {"name": "No Right Turn", "color": (0, 0, 255)},
            5: {"name": "No U-Turn", "color": (0, 0, 255)},
            6: {"name": "Enter Left Lane", "color": (0, 255, 0)},
            7: {"name": "Green Light", "color": (0, 255, 0)},
            8: {"name": "Left Right Lane", "color": (255, 196, 0)},
            9: {"name": "No Parking", "color": (0, 0, 255)},
            10: {"name": "Parking", "color": (0, 255, 0)},
            11: {"name": "Pedestrian Crossing", "color": (255, 196, 0)},
            12: {"name": "Zebra Crossing", "color": (255, 196, 0)},
            13: {"name": "Railway Crossing", "color": (255, 196, 0)},
            14: {"name": "Red Light", "color": (0, 0, 255)},
            15: {"name": "Stop", "color": (0, 0, 255)},
            16: {"name": "T-Intersection Left", "color": (255, 196, 0)},
            17: {"name": "Traffic Light", "color": (255, 196, 0)},
            18: {"name": "U-Turn", "color": (0, 255, 0)},
            19: {"name": "Warning", "color": (0, 255, 255)},
            20: {"name": "Yellow Light", "color": (0, 255, 255)}
        }
        self.current_sign = None
        self.sign_confidence = 0
        self.sign_image = None
        self.last_update_time = 0
        self.active_sign = False

    def update_sign(self, cls, conf, time_now, image=None):
        """Update current sign if confidence is higher or same class"""
        if cls == self.current_sign:  # Same sign class, keep showing
            if conf > self.sign_confidence:  # Update confidence and image if better
                self.sign_confidence = conf
                self.sign_image = image
            self.last_update_time = time_now
            self.active_sign = True
        elif conf > self.sign_confidence:  # Different sign with higher confidence
            self.current_sign = cls
            self.sign_confidence = conf
            self.sign_image = image
            self.last_update_time = time_now
            self.active_sign = True

    def check_sign_active(self, time_now):
        """Check if sign was updated recently (within 0.5 second)"""
        if time_now - self.last_update_time > 0.5:  # Half second threshold
            self.active_sign = False
        return self.active_sign

    def get_sign_info(self):
        """Get name and color of current sign"""
        if self.current_sign is not None and self.current_sign in self.sign_classes:
            return self.sign_classes[self.current_sign]
        return {"name": "Unknown Sign", "color": (128, 128, 128)}

    def draw_sign_icon(self, img, x, y, size):
        """Draw sign icon with improved design."""
        info = self.get_sign_info()
        sign_name = info["name"]
        sign_color = info["color"]

        # Draw rounded rectangle for background
        overlay = img.copy()
        cv2.rectangle(overlay, (x, y), (x + size, y + size), (255, 255, 255), -1)
        cv2.addWeighted(overlay, 0.4, img, 0.6, 0, img)

        # Add a gradient effect
        for i in range(size):
            alpha = i / size
            cv2.line(img, (x, y + i), (x + size, y + i), (
                int(sign_color[0] * alpha), 
                int(sign_color[1] * alpha), 
                int(sign_color[2] * alpha)), 1)

        # Draw specific sign details
        center_x = x + size // 2
        center_y = y + size // 2
        radius = size // 3

        if "Light" in sign_name:
            # Traffic light
            light_color = (0, 0, 255) if "Red" in sign_name else \
                        (0, 255, 255) if "Yellow" in sign_name else \
                        (0, 255, 0)
            cv2.circle(img, (center_x, center_y), radius // 2, light_color, -1)
        elif "Stop" in sign_name:
            # Stop sign
            cv2.circle(img, (center_x, center_y), radius, (0, 0, 255), -1)
            cv2.putText(img, "STOP", 
                        (x + size // 4, center_y + size // 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        else:
            # Default sign background
            cv2.rectangle(img, (x + size // 4, y + size // 4), 
                        (x + 3 * size // 4, y + 3 * size // 4),
                        sign_color, -1)

def draw_sign_display(img, sign_display, current_time):
    """Draw the sign display overlay with icon and information"""
    if sign_display.check_sign_active(current_time):  # Only show if sign is active
        # Setup display dimensions
        icon_size = 80
        display_width = 300
        display_height = max(100, icon_size + 20)
        margin = 20
        
        # Create semi-transparent background at top left
        overlay = img.copy()
        x1, y1 = margin, margin
        x2, y2 = margin + display_width, margin + display_height
        
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, img, 0.7, 0, img)
        
        # Draw icon at top left
        icon_x = x1 + 10
        icon_y = y1 + 10
        
        # Use actual sign image if available
        if sign_display.sign_image is not None:
            resized_sign = cv2.resize(sign_display.sign_image, (icon_size, icon_size))
            roi = img[icon_y:icon_y+icon_size, icon_x:icon_x+icon_size]
            # Create a mask for non-black pixels
            gray = cv2.cvtColor(resized_sign, cv2.COLOR_BGR2GRAY)
            _, mask = cv2.threshold(gray, 20, 255, cv2.THRESH_BINARY)
            # Copy only non-black pixels
            roi[mask > 0] = resized_sign[mask > 0]
        else:
            # Draw generic icon
            info = sign_display.get_sign_info()
            cv2.rectangle(img, (icon_x, icon_y), 
                         (icon_x + icon_size, icon_y + icon_size),
                         info["color"], -1)
        
        # Add sign information
        info = sign_display.get_sign_info()
        sign_name = info["name"]
        sign_color = info["color"]
        conf_text = f"{sign_display.sign_confidence:.2f}"
        
        # Position text
        text_x = icon_x + icon_size + 20
        text_y = y1 + 35
        
        # Draw text with white color and black outline for better visibility
        cv2.putText(img, sign_name, 
                   (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4)
        cv2.putText(img, sign_name, 
                   (text_x, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, sign_color, 2)
        
        cv2.putText(img, f"Conf: {conf_text}", 
                   (text_x, text_y + 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
class ZEDCamera:
    def __init__(self):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 60
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        self.image = sl.Mat()
        self.depth = sl.Mat()

    def open(self):
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Failed to open ZED camera: {err}")
            exit(1)

    def grab_frame(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
            
            image_ocv = self.image.get_data()
            
            if image_ocv is not None and image_ocv.shape[2] == 4:
                image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_BGRA2BGR)
                
            if image_ocv is not None:
                print(f"Frame shape: {image_ocv.shape}, dtype: {image_ocv.dtype}")
            else:
                print("Error: Retrieved frame is None")
                
            return image_ocv, self.depth
        return None, None

    def close(self):
        self.zed.close()

class NetworkManager:
    def __init__(self):
        self.distance_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.track_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gui_distance_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.distance_address = ('localhost', 12345)
        self.gui_distance_address = ('localhost', 12348)
        self.video_address = ('localhost', 12347)
        self.track_address = ('localhost', 12349)

    def send_frame(self, frame, fps):
        try:
            if frame is None or frame.size == 0:
                print("Error: Invalid frame for sending")
                return
                
            fps_bytes = struct.pack('f', fps)
            self.video_socket.sendto(fps_bytes, self.video_address)
                
            _, encoded_frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_bytes = encoded_frame.tobytes()
            
            size_bytes = struct.pack('I', len(frame_bytes))
            self.video_socket.sendto(size_bytes, self.video_address)
            
            chunk_size = 8192
            for i in range(0, len(frame_bytes), chunk_size):
                chunk = frame_bytes[i:i + chunk_size]
                self.video_socket.sendto(chunk, self.video_address)
                
            print(f"Sent frame: {frame.shape}, FPS: {fps}")
        except Exception as e:
            print(f"Error sending frame: {e}")

    def send_distance(self, distance):
        try:
            distance_bytes = struct.pack('f', distance)
            self.distance_socket.sendto(distance_bytes, self.distance_address)
            self.gui_distance_socket.sendto(distance_bytes, self.gui_distance_address)
            print(f"Sent distance: {distance}")
        except Exception as e:
            print(f"Error sending distance: {e}")

    def send_tracked_objects(self, tracked_objects, depth_map):
        try:
            tracked_objects_data = []
            for obj in tracked_objects:
                x1, y1, x2, y2, track_id = obj
                cx = int((x1 + x2) // 2)
                cy = int(y2)
                
                if 0 <= cx < depth_map.get_width() and 0 <= cy < depth_map.get_height():
                    depth = depth_map.get_value(cx, cy)[1]
                    if np.isfinite(depth):
                        tracked_objects_data.append([cx, cy, depth, track_id])

            if tracked_objects_data:
                data_array = np.array(tracked_objects_data, dtype=np.float32)
                data_bytes = data_array.tobytes()
                
                size_bytes = struct.pack('I', len(data_bytes))
                self.track_socket.sendto(size_bytes, self.track_address)
                
                chunk_size = 8192
                for i in range(0, len(data_bytes), chunk_size):
                    chunk = data_bytes[i:i + chunk_size]
                    self.track_socket.sendto(chunk, self.track_address)
                    
                print(f"Sent {len(tracked_objects_data)} tracked objects")
        except Exception as e:
            print(f"Error sending tracked objects: {e}")

    def cleanup(self):
        self.distance_socket.close()
        self.video_socket.close()
        self.track_socket.close()
        self.gui_distance_socket.close()

def detect():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='data/weights/yolopv2.pt', help='model path')
    parser.add_argument('--img-size', type=int, default=640, help='inference size')
    parser.add_argument('--conf-thres', type=float, default=0.3, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--device', default='0', help='cuda device')
    opt = parser.parse_args()

    print("\nInitializing components...")

    # Initialize components
    device = select_device(opt.device)
    model = torch.jit.load(opt.weights).to(device)
    
    # Initialize YOLO models
    person_model = YOLO('yolov8n.pt')
    person_model.overrides['classes'] = [0, 2, 4]  # Class 0 is person in COCO dataset
    
    # Initialize traffic sign model
    sign_model = YOLO('runs/detect/train/weights/best.pt')
    
    half = device.type != 'cpu'
    if half:
        model.half()
    model.eval()

    print("Models loaded successfully")

    # Initialize camera and network
    zed = ZEDCamera()
    zed.open()
    network = NetworkManager()
    
    # Initialize SORT trackers
    vehicle_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.45)
    person_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.45)
    sign_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.45)
    
    # FPS counter
    fps_avg = AverageMeter()
    
    # Initialize sign display
    sign_display = SignDisplay()
    
    print("\nStarting detection. Press 'q' to quit.\n")
    
    try:
        while True:
            # Get frame from ZED
            im0, depth = zed.grab_frame()
            if im0 is None:
                print("Error: Failed to grab frame")
                continue

            # Initialize result image
            result_img = im0.copy()

            # Process for YOLOPv2 (vehicle detection and segmentation)
            im0_rgb = cv2.cvtColor(im0, cv2.COLOR_BGR2RGB)
            img = letterbox(im0_rgb, new_shape=opt.img_size, stride=32, auto=True)[0]
            img = torch.from_numpy(img).to(device).float()
            img /= 255.0
            img = img.permute(2, 0, 1).unsqueeze(0)
            
            if half:
                img = img.half()

            # YOLOPv2 Inference
            t1 = time_synchronized()
            [pred, anchor_grid], seg, ll = model(img)
            
            # Person detection with YOLOv8
            person_results = person_model.predict(im0, conf=opt.conf_thres, iou=opt.iou_thres)
            
            # Traffic sign detection with YOLOv8
            sign_results = sign_model.predict(im0, conf=0.7, iou=0.45)
            
            t2 = time_synchronized()

            # Update FPS
            fps = 1 / (t2 - t1)
            fps_avg.update(fps)

            # Process detections and tracking
            vehicle_detections = []
            person_detections = []
            sign_detections = []

            # Process vehicle detections
            try:
                pred = split_for_trace_model(pred, anchor_grid)
                pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres)
                
                for det in pred:
                    if len(det):
                        det_cpu = det.cpu()
                        det_cpu[:, :4] = scale_coords(img.shape[2:], det_cpu[:, :4], im0.shape).round()
                        
                        for *xyxy, conf, cls in det_cpu.numpy():
                            vehicle_detections.append([xyxy[0], xyxy[1], xyxy[2], xyxy[3], conf])
            except Exception as e:
                print(f"Error in vehicle detection: {e}")

            # Process person detections
            if len(person_results) > 0:
                for result in person_results:
                    boxes = result.boxes
                    for box in boxes:
                        xyxy = box.xyxy[0].cpu().numpy()
                        conf = box.conf[0].cpu().numpy()
                        person_detections.append([xyxy[0], xyxy[1], xyxy[2], xyxy[3], conf])

            # Process traffic sign detections and update display
            current_time = time.time()
            if len(sign_results) > 0:
                for result in sign_results:
                    boxes = result.boxes
                    for box in boxes:
                        xyxy = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = map(int, xyxy)
                        conf = box.conf[0].cpu().numpy()
                        cls = int(box.cls[0])
                        sign_detections.append([x1, y1, x2, y2, conf])
                        
                        # Update sign display if confidence is sufficient
                        if conf > 0.5:  # Keep threshold relatively low for persistence
                            sign_crop = im0[y1:y2, x1:x2].copy()
                            sign_display.update_sign(cls, conf, current_time, sign_crop)

            # Update trackers
            tracked_vehicles = vehicle_tracker.update(np.array(vehicle_detections) if vehicle_detections else np.empty((0, 5)))
            tracked_persons = person_tracker.update(np.array(person_detections) if person_detections else np.empty((0, 5)))
            tracked_signs = sign_tracker.update(np.array(sign_detections) if sign_detections else np.empty((0, 5)))

            # Process segmentation
            try:
                if isinstance(seg, torch.Tensor):
                    seg = seg.cpu()
                if isinstance(ll, torch.Tensor):
                    ll = ll.cpu()
                
                da_seg_mask = driving_area_mask(seg).astype(np.uint8)
                ll_seg_mask = lane_line_mask(ll).astype(np.uint8)
                
                da_seg_mask = cv2.resize(da_seg_mask, (im0.shape[1], im0.shape[0]), 
                                       interpolation=cv2.INTER_NEAREST)
                ll_seg_mask = cv2.resize(ll_seg_mask, (im0.shape[1], im0.shape[0]), 
                                       interpolation=cv2.INTER_NEAREST)
            except Exception as e:
                print(f"Error processing segmentation: {e}")
                da_seg_mask = np.zeros((im0.shape[0], im0.shape[1]), dtype=np.uint8)
                ll_seg_mask = np.zeros((im0.shape[0], im0.shape[1]), dtype=np.uint8)

            # Draw tracked objects
            # Vehicles in yellow
            for obj in tracked_vehicles:
                x1, y1, x2, y2, track_id = obj
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                cv2.rectangle(result_img, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(result_img, f'Vehicle ID: {int(track_id)}', (x1, y1-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Persons in red
            for obj in tracked_persons:
                x1, y1, x2, y2, track_id = obj
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                cv2.rectangle(result_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(result_img, f'Person ID: {int(track_id)}', (x1, y1-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Traffic signs in blue
            for obj in tracked_signs:
                x1, y1, x2, y2, track_id = obj
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                cv2.rectangle(result_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(result_img, f'Sign ID: {int(track_id)}', (x1, y1-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Add segmentation visualization
            if result_img is not None and result_img.size > 0:
                try:
                    seg_img = result_img.copy()
                    
                    if da_seg_mask is not None:
                        seg_img[da_seg_mask > 0] = seg_img[da_seg_mask > 0] * 0.5 + np.array([230, 140, 0]) * 0.5
                    
                    if ll_seg_mask is not None:
                        seg_img[ll_seg_mask > 0] = seg_img[ll_seg_mask > 0] * 0.5 + np.array([0, 0, 255]) * 0.5
                    
                    result_img = seg_img
                except Exception as e:
                    print(f"Error in segmentation visualization: {e}")

            # Draw sign display overlay with icon
            draw_sign_display(result_img, sign_display, current_time)

            # Calculate distances
            min_distance = float('inf')
            # Combine only vehicles and persons for distance calculation
            distance_objects = np.concatenate([arr for arr in [tracked_vehicles, tracked_persons] if len(arr) > 0]) if len(tracked_vehicles) > 0 or len(tracked_persons) > 0 else np.array([])
            
            
            for obj in distance_objects:
                x1, y1, x2, y2, _ = obj
                cx = int((x1 + x2) // 2)
                cy = int(y2)
                if 0 <= cx < depth.get_width() and 0 <= cy < depth.get_height():
                    dist = depth.get_value(cx, cy)[1]
                    if np.isfinite(dist):
                        min_distance = min(min_distance, dist)

            # Send data through network manager
            if min_distance != float('inf'):
                network.send_distance(min_distance)

            if result_img is not None and result_img.size > 0:
                network.send_frame(cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB), fps_avg.avg)
                network.send_tracked_objects(distance_objects, depth)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        print("\nCleaning up...")
        zed.close()
        network.cleanup()
        cv2.destroyAllWindows()
        print("Cleanup completed")


def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    shape = img.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:
        r = min(r, 1.0)

    ratio = r, r
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    
    if auto:
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)
    elif scaleFill:
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]

    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, ratio, (dw, dh)

if __name__ == '__main__':
    try:
        print("Starting YOLOPv2 + YOLOv8 detection with ZED camera...")
        with torch.no_grad():
            detect()
    except KeyboardInterrupt:
        print("\nDetection stopped by user")
    except Exception as e:
        print(f"\nError in main execution: {e}")
    finally:
        print("Program terminated")