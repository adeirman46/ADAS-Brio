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
from waypoint_extractor import WaypointExtractor, Waypoint

from utils.utils import (
    time_synchronized, select_device, increment_path,
    scale_coords, non_max_suppression, split_for_trace_model,
    driving_area_mask, lane_line_mask, plot_one_box, show_seg_result,
    AverageMeter
)

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

def calculate_lane_distances(da_seg_mask, ll_seg_mask, image_width):
    """
    Calculate distances from center to left and right lane boundaries
    
    Args:
        da_seg_mask: Driving area segmentation mask
        ll_seg_mask: Lane line segmentation mask
        image_width: Width of the input image
    
    Returns:
        left_distance: Distance from center to left lane boundary in pixels
        right_distance: Distance from center to right lane boundary in pixels
    """
    # Get the bottom row of the masks for distance calculation
    bottom_row_idx = -50  # Look slightly above the bottom edge for more stable measurements
    da_bottom = da_seg_mask[bottom_row_idx, :]
    ll_bottom = ll_seg_mask[bottom_row_idx, :]
    
    # Find center point
    center_x = image_width // 2
    
    # Find left and right boundaries
    # First check lane lines
    left_points = np.where(ll_bottom[:center_x] > 0)[0]
    right_points = np.where(ll_bottom[center_x:] > 0)[0]
    
    # If lane lines not found, use driving area boundaries
    if len(left_points) == 0:
        left_points = np.where(da_bottom[:center_x] > 0)[0]
    if len(right_points) == 0:
        right_points = np.where(da_bottom[center_x:] > 0)[0]
    
    # Calculate distances
    left_distance = center_x - left_points[-1] if len(left_points) > 0 else None
    right_distance = right_points[0] if len(right_points) > 0 else None
    
    if right_distance is not None:
        right_distance += center_x
    
    return left_distance, right_distance

def visualize_lane_distances(image, left_distance, right_distance, pixels_to_meters=0.01):
    """
    Visualize the lane distances on the image
    Args:
        image: Input image
        left_distance: Distance to left lane in pixels
        right_distance: Distance to right lane in pixels
        pixels_to_meters: Conversion factor from pixels to meters (approximate)
    """
    height, width = image.shape[:2]
    center_x = width // 2
    y_position = height - 50  # Match the measurement position
    
    # Draw center point
    cv2.circle(image, (center_x, y_position), 5, (0, 255, 0), -1)
    
    # Draw distances if available
    if left_distance is not None:
        left_x = center_x - left_distance
        cv2.line(image, (center_x, y_position), (left_x, y_position), (255, 0, 0), 2)
        cv2.putText(image, f'{left_distance:.2f}px', (left_x, y_position - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    
    if right_distance is not None:
        right_x = right_distance
        cv2.line(image, (center_x, y_position), (right_x, y_position), (0, 0, 255), 2)
        cv2.putText(image, f'{right_distance:.2f}px', (right_x, y_position - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    return image

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
    
    # Initialize YOLO model for person detection
    person_model = YOLO('yolov8n.pt')
    person_model.overrides['classes'] = [0, 2, 4]  # Class 0 is person in COCO dataset

    # Initialize waypoint extractor
    waypoint_extractor = WaypointExtractor(num_points=20)
    
    half = device.type != 'cpu'
    if half:
        model.half()
    model.eval()

    print("Models loaded successfully")

    # Initialize camera and network
    zed = ZEDCamera()
    zed.open()
    network = NetworkManager()
    
    # Initialize SORT trackers - one for vehicles, one for persons
    vehicle_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.45)
    person_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.45)
    
    # FPS counter
    fps_avg = AverageMeter()
    
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
            print(f"\nProcessing frame...")

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
            
            t2 = time_synchronized()

            # Update FPS
            fps = 1 / (t2 - t1)
            fps_avg.update(fps)
            print(f"Inference FPS: {fps:.1f}")

            # Process YOLOPv2 predictions (vehicles)
            try:
                pred = split_for_trace_model(pred, anchor_grid)
                pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres)
                print("Vehicle predictions processed successfully")
            except Exception as e:
                print(f"Error in vehicle prediction processing: {e}")
                pred = []

            # Process detections and tracking
            vehicle_detections = []
            person_detections = []

            # Process vehicle detections
            for det in pred:
                if len(det):
                    det_cpu = det.cpu()
                    det_cpu[:, :4] = scale_coords(img.shape[2:], det_cpu[:, :4], im0.shape).round()
                    
                    for *xyxy, conf, cls in det_cpu.numpy():
                        vehicle_detections.append([xyxy[0], xyxy[1], xyxy[2], xyxy[3], conf])

            # Process person detections
            if len(person_results) > 0:
                for result in person_results:
                    boxes = result.boxes
                    for box in boxes:
                        xyxy = box.xyxy[0].cpu().numpy()
                        conf = box.conf[0].cpu().numpy()
                        person_detections.append([xyxy[0], xyxy[1], xyxy[2], xyxy[3], conf])

            # Update trackers
            tracked_vehicles = vehicle_tracker.update(np.array(vehicle_detections) if vehicle_detections else np.empty((0, 5)))
            tracked_persons = person_tracker.update(np.array(person_detections) if person_detections else np.empty((0, 5)))

            print(f"Tracking {len(tracked_vehicles)} vehicles and {len(tracked_persons)} persons")

            # Process segmentation
            try:
                if isinstance(seg, torch.Tensor):
                    seg = seg.cpu()
                if isinstance(ll, torch.Tensor):
                    ll = ll.cpu()
                
                da_seg_mask = driving_area_mask(seg).astype(np.uint8)

                # Extract and visualize waypoints
                waypoints = waypoint_extractor.extract_waypoints(
                    da_seg_mask, 
                    depth,
                    im0.shape[0],
                    im0.shape[1]
                )

                # Visualize waypoints on the result image
                result_img = waypoint_extractor.visualize_waypoints(result_img, waypoints)

                ll_seg_mask = lane_line_mask(ll).astype(np.uint8)

                # Calculate lane distances
                left_dist, right_dist = calculate_lane_distances(da_seg_mask, ll_seg_mask, im0.shape[1])

                # Visualize the distances
                result_img = visualize_lane_distances(result_img, left_dist, right_dist)
                
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

            # Calculate distances (consider both vehicles and persons)
            min_distance = float('inf')
            all_tracked_objects = np.concatenate((tracked_vehicles, tracked_persons)) if len(tracked_vehicles) > 0 and len(tracked_persons) > 0 else tracked_vehicles if len(tracked_vehicles) > 0 else tracked_persons if len(tracked_persons) > 0 else np.array([])

            for obj in all_tracked_objects:
                x1, y1, x2, y2, _ = obj
                cx = int((x1 + x2) // 2)
                cy = int(y2)
                if 0 <= cx < depth.get_width() and 0 <= cy < depth.get_height():
                    dist = depth.get_value(cx, cy)[1]
                    if np.isfinite(dist):
                        min_distance = min(min_distance, dist)

            # Send data
            if min_distance != float('inf'):
                network.send_distance(min_distance)

            if result_img is not None and result_img.size > 0:
                # Add FPS display
                # cv2.putText(result_img, f'FPS: {fps_avg.avg:.1f}', (20, 40),
                #            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                network.send_frame(cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB), fps_avg.avg)

            # Send all tracked objects (both vehicles and persons)
            network.send_tracked_objects(all_tracked_objects, depth)

            # Display the frame
            # cv2.imshow("YOLOPv2 + YOLOv8 Detection", result_img)
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