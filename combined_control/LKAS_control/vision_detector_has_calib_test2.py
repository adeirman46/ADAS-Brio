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
from ultralytics import YOLO
from utils.utils import (
    time_synchronized, select_device, increment_path,
    scale_coords, non_max_suppression, split_for_trace_model,
    driving_area_mask, plot_one_box,
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

class NetworkManager:
    def __init__(self):
        self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lane_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.track_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.video_address = ('localhost', 12347)
        self.lane_address = ('localhost', 12350)
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

    def send_lane_boundaries(self, left_boundary, right_boundary):
        try:
            if left_boundary is not None and len(left_boundary) > 0:
                left_data = np.array(left_boundary, dtype=np.float32)
                left_bytes = left_data.tobytes()
                
                header = struct.pack('II', 0, len(left_bytes))
                self.lane_socket.sendto(header + left_bytes, self.lane_address)
            
            if right_boundary is not None and len(right_boundary) > 0:
                right_data = np.array(right_boundary, dtype=np.float32)
                right_bytes = right_data.tobytes()
                
                header = struct.pack('II', 1, len(right_bytes))
                self.lane_socket.sendto(header + right_bytes, self.lane_address)
                
            print(f"Sent lane boundaries: Left={len(left_boundary) if left_boundary else 0} points, Right={len(right_boundary) if right_boundary else 0} points")
        except Exception as e:
            print(f"Error sending lane boundaries: {e}")

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
        self.video_socket.close()
        self.lane_socket.close()
        self.track_socket.close()

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

def extract_lane_boundaries(drivable_mask, roi_mask=None):
    """
    Extract leftmost and rightmost boundaries from drivable area segmentation with improved filtering.
    
    Args:
        drivable_mask: Binary mask of drivable area
        roi_mask: Optional ROI mask to limit processing area
    
    Returns:
        left_boundary: List of (x, y) points for left boundary
        right_boundary: List of (x, y) points for right boundary
    """
    if roi_mask is not None:
        drivable_mask = drivable_mask & roi_mask
    
    # Apply morphological operations to reduce noise
    kernel = np.ones((5, 5), np.uint8)
    drivable_mask = cv2.morphologyEx(drivable_mask, cv2.MORPH_CLOSE, kernel)
    drivable_mask = cv2.morphologyEx(drivable_mask, cv2.MORPH_OPEN, kernel)

    height, width = drivable_mask.shape
    left_boundary = []
    right_boundary = []
    
    scan_start = int(height * 0.95)
    scan_end = int(height * 0.3)
    scan_step = 5
    
    for y in range(scan_start, scan_end, -scan_step):
        if y < 0 or y >= height:
            continue
            
        row = drivable_mask[y]
        drivable_indices = np.where(row > 0)[0]
        
        if len(drivable_indices) > 0:
            leftmost_x = drivable_indices[0]
            rightmost_x = drivable_indices[-1]
            
            min_width = 50
            if rightmost_x - leftmost_x > min_width:
                # Additional filtering: Ensure continuity with previous points
                if left_boundary and abs(leftmost_x - left_boundary[-1][0]) > 50:
                    leftmost_x = left_boundary[-1][0]  # Use previous x to maintain continuity
                if right_boundary and abs(rightmost_x - right_boundary[-1][0]) > 50:
                    rightmost_x = right_boundary[-1][0]
                
                left_boundary.append([int(leftmost_x), int(y)])
                right_boundary.append([int(rightmost_x), int(y)])
    
    return left_boundary, right_boundary

def smooth_boundary(boundary_points, window_size=5):
    if len(boundary_points) < window_size:
        return boundary_points
    
    smoothed_points = []
    boundary_array = np.array(boundary_points)
    
    for i in range(len(boundary_points)):
        start_idx = max(0, i - window_size // 2)
        end_idx = min(len(boundary_points), i + window_size // 2 + 1)
        
        window_points = boundary_array[start_idx:end_idx]
        smooth_x = np.mean(window_points[:, 0])
        smooth_y = boundary_array[i][1]
        
        smoothed_points.append([smooth_x, smooth_y])
    
    return smoothed_points

def draw_lane_boundaries(img, left_boundary, right_boundary):
    if left_boundary and len(left_boundary) > 1:
        pts = np.array(left_boundary, dtype=np.int32)
        cv2.polylines(img, [pts], False, (0, 255, 0), 3)
        for point in left_boundary[::3]:
            cv2.circle(img, (int(point[0]), int(point[1])), 3, (0, 255, 0), -1)
    
    if right_boundary and len(right_boundary) > 1:
        pts = np.array(right_boundary, dtype=np.int32)
        cv2.polylines(img, [pts], False, (0, 0, 255), 3)
        for point in right_boundary[::3]:
            cv2.circle(img, (int(point[0]), int(point[1])), 3, (0, 0, 255), -1)
    
    return img

def calculate_real_world_distances(left_boundary, right_boundary, depth_map, image_width):
    """
    Calculate real-world distances from the center to the left and right boundaries in meters.
    
    Args:
        left_boundary: List of [x, y] points for left boundary
        right_boundary: List of [x, y] points for right boundary
        depth_map: ZED depth map
        image_width: Width of the image
    
    Returns:
        left_distance_m: Distance from center to left boundary in meters
        right_distance_m: Distance from center to right boundary in meters
    """
    image_center_x = image_width // 2
    left_distance_m = None
    right_distance_m = None
    
    if left_boundary and right_boundary:
        # Use the bottom-most points (closest to the vehicle)
        bottom_left = left_boundary[0]
        bottom_right = right_boundary[0]
        
        # Get depth at these points
        depth_left = depth_map.get_value(int(bottom_left[0]), int(bottom_left[1]))[1]
        depth_right = depth_map.get_value(int(bottom_right[0]), int(bottom_right[1]))[1]
        
        if np.isfinite(depth_left) and np.isfinite(depth_right):
            # Approximate real-world distance using depth and pixel distance
            # Assume the camera's field of view and depth give us a rough conversion
            # Simplified: Distance in meters = (pixel distance / image width) * depth * tan(FOV/2)
            # ZED HD720 FOV is approximately 90 degrees horizontally
            fov = 90 * (np.pi / 180)  # Convert to radians
            tan_half_fov = np.tan(fov / 2)
            
            # Calculate pixel distances from center
            pixel_dist_left = abs(image_center_x - bottom_left[0])
            pixel_dist_right = abs(bottom_right[0] - image_center_x)
            
            # Convert to real-world distances
            left_distance_m = (pixel_dist_left / image_width) * depth_left * tan_half_fov
            right_distance_m = (pixel_dist_right / image_width) * depth_right * tan_half_fov
    
    return left_distance_m, right_distance_m

def filter_detections(det, roi_pts):
    if len(det) == 0:
        return det

    roi_polygon = np.array([roi_pts[0], roi_pts[2], roi_pts[3], roi_pts[1]], dtype=np.int32)

    filtered = []
    for detection in det:
        if len(detection) != 5:
            print(f"Unexpected detection format: {detection}")
            continue
        x1, y1, x2, y2, conf = map(float, detection)
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        if cv2.pointPolygonTest(roi_polygon, (cx, cy), False) >= 0:
            filtered.append([x1, y1, x2, y2, conf])
    return np.array(filtered) if filtered else np.empty((0, 5))

def make_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='/home/irman/ADAS-Brio/vision_control/yolopv2/data/weights/yolopv2.pt', help='model path')
    parser.add_argument('--img-size', type=int, default=640, help='inference size')
    parser.add_argument('--conf-thres', type=float, default=0.3, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--device', default='0', help='cuda device')
    parser.add_argument('--calibrate', action='store_true', help='enable ROI calibration and save to ROI.txt')
    return parser

def calibrate_camera(zed, calibrate=False):
    if calibrate:
        points = []
        window_name = "Select 4 Points (TL, BL, TR, BR) - Press 'q' to confirm"

        def click_event(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                points.append((x, y))
                cv2.circle(image, (x, y), 5, (0, 0, 255), -1)
                cv2.imshow(window_name, image)
                print(f"Point selected: ({x}, {y})")

        image, _ = zed.grab_frame()
        if image is None:
            raise ValueError("Could not grab frame from ZED camera for calibration.")

        cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, click_event)

        print("Click 4 points in the order: Top-Left (TL), Bottom-Left (BL), Top-Right (TR), Bottom-Right (BR)")
        print("Press 'q' after selecting all 4 points to proceed.")

        while True:
            cv2.imshow(window_name, image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') and len(points) == 4:
                break
            elif key == ord('q'):
                print("Please select exactly 4 points before proceeding.")
                points = []
                cv2.imshow(window_name, image)

        cv2.destroyWindow(window_name)

        tl, bl, tr, br = points
        print(f"Selected coordinates: TL={tl}, BL={bl}, TR={tr}, BR={br}")

        with open('ROI.txt', 'w') as f:
            f.write(f"{tl[0]},{tl[1]}\n")
            f.write(f"{bl[0]},{bl[1]}\n")
            f.write(f"{tr[0]},{tr[1]}\n")
            f.write(f"{br[0]},{br[1]}\n")
    else:
        try:
            with open('ROI.txt', 'r') as f:
                lines = f.readlines()
                if len(lines) != 4:
                    raise ValueError("ROI.txt must contain exactly 4 lines with coordinates.")
                points = []
                for line in lines:
                    x, y = map(int, line.strip().split(','))
                    points.append((x, y))
                tl, bl, tr, br = points
                print(f"Loaded coordinates from ROI.txt: TL={tl}, BL={bl}, TR={tr}, BR={br}")
        except FileNotFoundError:
            print("ROI.txt not found. Please run with --calibrate to create it.")
            exit(1)
        except ValueError as e:
            print(f"Error reading ROI.txt: {e}")
            exit(1)

    pts = np.float32([tl, bl, tr, br])
    return pts

def create_roi_mask(shape, pts):
    mask = np.zeros(shape, dtype=np.uint8)
    roi_polygon = np.array([pts[0], pts[2], pts[3], pts[1]], dtype=np.int32)
    cv2.fillPoly(mask, [roi_polygon], 1)
    return mask

def draw_roi(frame, pts):
    tl, bl, tr, br = pts
    pts_draw = np.array([tl, tr, br, bl], np.int32).reshape((-1, 1, 2))
    cv2.polylines(frame, [pts_draw], isClosed=True, color=(255, 255, 0), thickness=2)
    return frame

def detect():
    device = select_device(opt.device)
    model = torch.jit.load(opt.weights).to(device)
    
    person_model = YOLO('/home/irman/ADAS-Brio/vision_control/yolopv2/yolov8n.pt')
    person_model.overrides['classes'] = [0, 2, 4]

    half = device.type != 'cpu'
    if half:
        model.half()
    model.eval()

    print("Models loaded successfully")

    zed = ZEDCamera()
    zed.open()
    network = NetworkManager()
    
    vehicle_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.45)
    person_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.45)
    
    fps_avg = AverageMeter()

    roi_pts = calibrate_camera(zed, opt.calibrate)
    roi_mask = create_roi_mask((720, 1280), roi_pts)

    print("\nStarting lane keeping assist with object detection. Press 'q' to quit.\n")
    
    try:
        while True:
            im0, depth = zed.grab_frame()
            if im0 is None:
                print("Error: Failed to grab frame")
                continue

            result_img = im0.copy()
            print(f"\nProcessing frame...")

            im0_rgb = cv2.cvtColor(im0, cv2.COLOR_BGR2RGB)
            img = letterbox(im0_rgb, new_shape=opt.img_size, stride=32, auto=True)[0]
            img = torch.from_numpy(img).to(device).float()
            img /= 255.0
            img = img.permute(2, 0, 1).unsqueeze(0)
            
            if half:
                img = img.half()

            t1 = time_synchronized()
            
            [pred, anchor_grid], seg, ll = model(img)
            person_results = person_model.predict(im0, conf=opt.conf_thres, iou=opt.iou_thres)
            
            t2 = time_synchronized()

            fps = 1 / (t2 - t1)
            fps_avg.update(fps)
            print(f"Inference FPS: {fps:.1f}")

            # Object Detection and Tracking
            try:
                pred = split_for_trace_model(pred, anchor_grid)
                pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres)
                print("Vehicle predictions processed successfully")
            except Exception as e:
                print(f"Error in vehicle prediction processing: {e}")
                pred = []

            vehicle_detections = []
            person_detections = []

            for det in pred:
                if len(det):
                    det_cpu = det.cpu()
                    det_cpu[:, :4] = scale_coords(img.shape[2:], det_cpu[:, :4], im0.shape).round()
                    
                    for *xyxy, conf, cls in det_cpu.numpy():
                        vehicle_detections.append([xyxy[0], xyxy[1], xyxy[2], xyxy[3], conf])

            if len(person_results) > 0:
                for result in person_results:
                    boxes = result.boxes
                    for box in boxes:
                        xyxy = box.xyxy[0].cpu().numpy()
                        conf = box.conf[0].cpu().numpy()
                        person_detections.append([xyxy[0], xyxy[1], xyxy[2], xyxy[3], conf])

            vehicle_detections = filter_detections(np.array(vehicle_detections) if vehicle_detections else np.empty((0, 5)), roi_pts)
            person_detections = filter_detections(np.array(person_detections) if person_detections else np.empty((0, 5)), roi_pts)

            tracked_vehicles = vehicle_tracker.update(vehicle_detections)
            tracked_persons = person_tracker.update(person_detections)

            print(f"Tracking {len(tracked_vehicles)} vehicles and {len(tracked_persons)} persons within ROI")

            # Process Drivable Area Segmentation
            try:
                if isinstance(seg, torch.Tensor):
                    seg = seg.cpu()
                
                da_seg_mask = driving_area_mask(seg).astype(np.uint8)
                da_seg_mask = cv2.resize(da_seg_mask, (im0.shape[1], im0.shape[0]), 
                                       interpolation=cv2.INTER_NEAREST)
                
                print("Drivable area segmentation processed successfully")
                
            except Exception as e:
                print(f"Error processing segmentation: {e}")
                da_seg_mask = np.zeros((im0.shape[0], im0.shape[1]), dtype=np.uint8)

            # Extract and Smooth Lane Boundaries
            left_boundary, right_boundary = extract_lane_boundaries(da_seg_mask, roi_mask)
            
            if left_boundary:
                left_boundary = smooth_boundary(left_boundary, window_size=5)
            if right_boundary:
                right_boundary = smooth_boundary(right_boundary, window_size=5)

            print(f"Extracted boundaries: Left={len(left_boundary)} points, Right={len(right_boundary)} points")

            # Calculate Real-World Distances
            left_distance_m, right_distance_m = calculate_real_world_distances(left_boundary, right_boundary, depth, im0.shape[1])

            # Draw ROI and Visualizations
            result_img = draw_roi(result_img, roi_pts)

            if da_seg_mask is not None:
                try:
                    seg_img = result_img.astype(np.float32)
                    da_indices = (da_seg_mask > 0) & (roi_mask > 0)
                    seg_img[da_indices] = seg_img[da_indices] * 0.7 + np.array([100, 200, 100], dtype=np.float32) * 0.3
                    result_img = np.clip(seg_img, 0, 255).astype(np.uint8)
                except Exception as e:
                    print(f"Error in segmentation visualization: {e}")

            # Draw Lane Boundaries
            result_img = draw_lane_boundaries(result_img, left_boundary, right_boundary)

            # Draw Tracked Objects
            for obj in tracked_vehicles:
                x1, y1, x2, y2, track_id = obj
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                cv2.rectangle(result_img, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.putText(result_img, f'Vehicle ID: {int(track_id)}', (x1, y1-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            for obj in tracked_persons:
                x1, y1, x2, y2, track_id = obj
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                cv2.rectangle(result_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(result_img, f'Person ID: {int(track_id)}', (x1, y1-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Display Lane Information
            if left_boundary and right_boundary:
                bottom_left = left_boundary[0]
                bottom_right = right_boundary[0]
                
                lane_center_x = (bottom_left[0] + bottom_right[0]) / 2
                image_center_x = im0.shape[1] / 2
                offset = lane_center_x - image_center_x
                
                cv2.line(result_img, (int(lane_center_x), im0.shape[0] - 50), 
                        (int(lane_center_x), im0.shape[0] - 10), (255, 0, 255), 3)
                cv2.line(result_img, (int(image_center_x), im0.shape[0] - 50), 
                        (int(image_center_x), im0.shape[0] - 10), (255, 255, 255), 3)
                
                cv2.putText(result_img, f'Lane Offset: {offset:.1f}px', (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                lane_width = bottom_right[0] - bottom_left[0]
                cv2.putText(result_img, f'Lane Width: {lane_width:.1f}px', (10, 60), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                # Display Real-World Distances
                if left_distance_m is not None and right_distance_m is not None:
                    cv2.putText(result_img, f'Left Distance: {left_distance_m:.2f}m', (10, 90), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(result_img, f'Right Distance: {right_distance_m:.2f}m', (10, 120), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Send Data Over Network
            if result_img is not None and result_img.size > 0:
                network.send_frame(cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB), fps_avg.avg)

            network.send_lane_boundaries(left_boundary, right_boundary)

            all_tracked_objects = np.concatenate((tracked_vehicles, tracked_persons)) if len(tracked_vehicles) > 0 and len(tracked_persons) > 0 else tracked_vehicles if len(tracked_vehicles) > 0 else tracked_persons if len(tracked_persons) > 0 else np.array([])
            network.send_tracked_objects(all_tracked_objects, depth)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        print("\nCleaning up...")
        zed.close()
        network.cleanup()
        cv2.destroyAllWindows()
        print("Cleanup completed")

if __name__ == '__main__':
    opt = make_parser().parse_args()
    print(opt)
    try:
        print("Starting Lane Keeping Assist with YOLOPv2 drivable area segmentation and object detection...")
        with torch.no_grad():
            detect()
    except KeyboardInterrupt:
        print("\nDetection stopped by user")
    except Exception as e:
        print(f"\nError in main execution: {e}")
    finally:
        print("Program terminated")