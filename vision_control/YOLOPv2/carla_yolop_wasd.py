import argparse
import time
from pathlib import Path
import cv2
import torch
import numpy as np
import socket
import struct
from sort import Sort
from collections import deque
from ultralytics import YOLO
from waypoint_extractor import WaypointExtractor, Waypoint
import pygame  # Add pygame import
import carla
import queue
import random

from utils.utils import (
    time_synchronized, select_device, increment_path,
    scale_coords, non_max_suppression, split_for_trace_model,
    driving_area_mask, lane_line_mask, plot_one_box, show_seg_result,
    AverageMeter
)

class CARLACamera:
    def __init__(self):
        self.client = None
        self.world = None
        self.vehicle = None
        self.camera_rgb = None
        self.camera_depth = None
        self._image_queue = queue.Queue()
        self._depth_queue = queue.Queue()
        self.rgb_image = None
        self.depth_array = None
        self.control = None
        self.pygame_initialized = False

    def setup_pygame(self):
        """Initialize Pygame for manual control"""
        if not self.pygame_initialized:
            pygame.init()
            pygame.display.set_caption("CARLA Manual Control")
            self.screen = pygame.display.set_mode((320, 240))  # Small control window
            self.clock = pygame.time.Clock()
            self.control = carla.VehicleControl()
            self.pygame_initialized = True

    def process_input(self):
        """Process keyboard input for vehicle control"""
        if not self.pygame_initialized or not self.vehicle:
            return

        # Process Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

        # Get pressed keys
        keys = pygame.key.get_pressed()

        # Update control based on key states
        # Throttle (W)
        if keys[pygame.K_w]:
            self.control.throttle = min(self.control.throttle + 0.05, 1.0)
        else:
            self.control.throttle = max(self.control.throttle - 0.05, 0.0)

        # Brake (S)
        if keys[pygame.K_s]:
            self.control.brake = min(self.control.brake + 0.2, 1.0)
        else:
            self.control.brake = 0.0

        # Steering (A/D)
        if keys[pygame.K_a]:
            self.control.steer = max(self.control.steer - 0.05, -1.0)
        elif keys[pygame.K_d]:
            self.control.steer = min(self.control.steer + 0.05, 1.0)
        else:
            self.control.steer = 0.0

        # Handbrake (SPACE)
        self.control.hand_brake = keys[pygame.K_SPACE]

        # Reverse gear (R)
        if keys[pygame.K_r]:
            self.control.reverse = not self.control.reverse

        # Apply control to vehicle
        if self.vehicle:
            self.vehicle.apply_control(self.control)

        # Maintain consistent timing
        self.clock.tick(60)
        return True
        
    def image_callback(self, image):
        """Callback for RGB camera data"""
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        array = array[:, :, ::-1]  # Convert RGBA to BGR
        self.rgb_image = array
        self._image_queue.put(array)

    def depth_callback(self, image):
        """Callback for depth camera data"""
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array.astype(np.float32)
        array = array[:, :, 0] * 1000.0 / 255.0  # Convert to meters
        self.depth_array = array
        self._depth_queue.put(array)

    def _find_intersection_spawn(self, spawn_points):
        """Find a clear spawn point near an intersection"""
        try:
            # Shuffle spawn points for randomization
            random.shuffle(spawn_points)
            
            for spawn_point in spawn_points:
                # Get waypoint at spawn point
                waypoint = self.world.get_map().get_waypoint(
                    spawn_point.location,
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving
                )
                
                # Look for intersection
                intersection_waypoint = waypoint
                distance = 0
                while not intersection_waypoint.is_intersection and distance < 100:
                    # Get next waypoints
                    next_waypoints = intersection_waypoint.next(5.0)
                    if not next_waypoints:
                        break
                    intersection_waypoint = next_waypoints[0]
                    distance += 5.0
                
                if intersection_waypoint.is_intersection:
                    # Get spawn point before intersection
                    spawn_transform = carla.Transform()
                    prev_waypoint = intersection_waypoint.previous(20.0)[0]
                    spawn_transform.location = prev_waypoint.transform.location
                    spawn_transform.rotation = prev_waypoint.transform.rotation
                    
                    # Check if spawn point is clear
                    if not self.world.cast_ray(
                        spawn_transform.location,
                        spawn_transform.location + carla.Location(z=2.0)
                    ):
                        return spawn_transform
                        
            # If no clear intersection spawn found, try regular spawn points
            for spawn_point in spawn_points:
                if not self.world.cast_ray(
                    spawn_point.location,
                    spawn_point.location + carla.Location(z=2.0)
                ):
                    return spawn_point
                    
            # If still no clear spawn found, return first spawn point
            return spawn_points[0]
            
        except Exception as e:
            print(f"Error finding spawn point: {e}")
            return spawn_points[0]

    def open(self, host='localhost', port=2000):
        """Initialize CARLA connection and setup vehicle with sensors."""
        try:
            # Connect to CARLA
            self.client = carla.Client(host, port)
            self.client.set_timeout(20.0)
            self.client.load_world('Town02', reset_settings=True)

            # Load world
            self.world = self.client.get_world()
            
            # Configure weather
            weather = carla.WeatherParameters(
                cloudiness=0.0,
                precipitation=0.0,
                sun_altitude_angle=70.0
            )
            self.world.set_weather(weather)

            # Configure world settings
            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            self.world.apply_settings(settings)

            # Clear existing actors
            self._clear_existing_actors()

            # Setup vehicle
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
            
            # Find spawn point
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                raise RuntimeError("No spawn points found!")
                
            self.vehicle = self.world.spawn_actor(vehicle_bp, random.choice(spawn_points))
            
            # Setup cameras
            camera_bp = blueprint_library.find('sensor.camera.rgb')
            depth_bp = blueprint_library.find('sensor.camera.depth')

            for bp in [camera_bp, depth_bp]:
                bp.set_attribute('image_size_x', '800')
                bp.set_attribute('image_size_y', '600')
                bp.set_attribute('fov', '90')

            camera_transform = carla.Transform(
                carla.Location(x=0.8, y=0.0, z=1.7),
                carla.Rotation(pitch=-5.0)
            )

            self.camera_rgb = self.world.spawn_actor(
                camera_bp,
                camera_transform,
                attach_to=self.vehicle
            )
            self.camera_depth = self.world.spawn_actor(
                depth_bp,
                camera_transform,
                attach_to=self.vehicle
            )

            self.camera_rgb.listen(self.image_callback)
            self.camera_depth.listen(self.depth_callback)
            # Move spectator to follow vehicle
            self.spectator = self.world.get_spectator()
            self._update_spectator()  # Initial update
            transform = self.vehicle.get_transform()
            self.spectator.set_transform(carla.Transform(
                transform.location + carla.Location(z=50),
                carla.Rotation(pitch=-90)
            ))
            # Initialize Pygame for manual control
            self.setup_pygame()

            # Wait for sensors to initialize
            time.sleep(1)
            
            return True

        except Exception as error:
            print(f"Failed to connect to CARLA: {error}")
            self._cleanup_actors()
            return False

    def grab_frame(self):
        """Get frame from CARLA and process manual control input"""
        try:
            # Update spectator position before ticking the world
            self._update_spectator()
            
            # Process manual control input
            if not self.process_input():
                return None, None

            self.world.tick()
            
            try:
                image = self._image_queue.get(timeout=2.0)
                depth = self._depth_queue.get(timeout=2.0)
                return image, depth
            except queue.Empty:
                print("Timeout waiting for sensor data")
                return None, None
                
        except Exception as e:
            print(f"Error grabbing frame: {e}")
            return None, None

    def close(self):
        """Cleanup CARLA and Pygame resources"""
        try:
            if self.camera_rgb:
                self.camera_rgb.stop()
                self.camera_rgb.destroy()
            if self.camera_depth:
                self.camera_depth.stop()
                self.camera_depth.destroy()
            if self.vehicle:
                self.vehicle.destroy()
            
            if self.world:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)
            
            if self.pygame_initialized:
                pygame.quit()
            
            print("Closed CARLA connection")
        except Exception as e:
            print(f"Error during cleanup: {e}")
        
    def _update_spectator(self):
        """Update spectator to follow vehicle from behind"""
        if self.vehicle and self.spectator:
            # Get vehicle's transform
            vehicle_transform = self.vehicle.get_transform()
            
            # Calculate camera position behind and above vehicle
            camera_distance = 10  # Distance behind the vehicle
            camera_height = 5    # Height above the vehicle
            
            # Get vehicle's forward vector
            forward_vector = vehicle_transform.get_forward_vector()
            
            # Calculate camera position
            camera_location = vehicle_transform.location - forward_vector * camera_distance
            camera_location.z = vehicle_transform.location.z + camera_height
            
            # Calculate camera rotation to look at vehicle
            camera_rotation = vehicle_transform.rotation
            camera_rotation.pitch = -15  # Look down slightly
            
            # Set spectator transform
            self.spectator.set_transform(
                carla.Transform(camera_location, camera_rotation)
            )

    def _clear_existing_actors(self):
        """Clear all existing actors from the world."""
        try:
            actor_list = self.world.get_actors()
            for actor in actor_list:
                if actor.type_id.startswith(('vehicle', 'sensor', 'walker')):
                    actor.destroy()
        except Exception as e:
            print(f"Error clearing actors: {e}")

    def _cleanup_actors(self):
        """Clean up actors on error."""
        try:
            if hasattr(self, 'camera_rgb') and self.camera_rgb:
                self.camera_rgb.destroy()
            if hasattr(self, 'camera_depth') and self.camera_depth:
                self.camera_depth.destroy()
            if hasattr(self, 'vehicle') and self.vehicle:
                self.vehicle.destroy()
        except Exception as e:
            print(f"Error during actor cleanup: {e}")
            
    def _spawn_npcs(self, num_vehicles=10, num_pedestrians=5):
        """Spawn NPC vehicles and pedestrians"""
        try:
            # Spawn NPC vehicles
            blueprint_library = self.world.get_blueprint_library()
            car_blueprints = [bp for bp in blueprint_library.filter('vehicle.*')
                            if int(bp.get_attribute('number_of_wheels')) == 4]
            
            spawn_points = self.world.get_map().get_spawn_points()
            for _ in range(min(num_vehicles, len(spawn_points))):
                spawn_point = random.choice(spawn_points)
                bp = random.choice(car_blueprints)
                npc = self.world.spawn_actor(bp, spawn_point)
                if npc:
                    npc.set_autopilot(True)
                    spawn_points.remove(spawn_point)
            
            # Spawn pedestrians
            pedestrian_bps = blueprint_library.filter('walker.pedestrian.*')
            for _ in range(num_pedestrians):
                bp = random.choice(pedestrian_bps)
                spawn_point = carla.Transform(
                    self.world.get_random_location_from_navigation()
                )
                pedestrian = self.world.spawn_actor(bp, spawn_point)
                if pedestrian:
                    # Add pedestrian controller
                    controller_bp = blueprint_library.find('controller.ai.walker')
                    controller = self.world.spawn_actor(controller_bp, carla.Transform(), attach_to=pedestrian)
                    controller.start()
                    controller.go_to_location(self.world.get_random_location_from_navigation())
                    
        except Exception as e:
            print(f"Error spawning NPCs: {e}")

        
    # In CARLACamera class, add after vehicle spawn:
    def _setup_vehicle_behavior(self):
        """Setup automatic driving behavior"""
        # Enable autopilot
        self.vehicle.set_autopilot(True)
        
        # Set high-level behavior parameters
        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.global_percentage_speed_difference(10.0)  # Drive 10% slower than speed limit
        traffic_manager.set_synchronous_mode(True)
        
        # Set vehicle-specific behavior
        tm_port = traffic_manager.get_port()
        self.vehicle.set_autopilot(True, tm_port)
        
        # Configure vehicle behavior
        traffic_manager.vehicle_percentage_speed_difference(self.vehicle, 0)
        traffic_manager.set_desired_speed(self.vehicle, 30)  # Set desired speed to 30 km/h
        traffic_manager.distance_to_leading_vehicle(self.vehicle, 5.0)  # Set safe distance
        traffic_manager.ignore_lights_percentage(self.vehicle, 0)  # Always obey traffic lights
        traffic_manager.ignore_signs_percentage(self.vehicle, 0)  # Always obey signs


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
                
        except Exception as e:
            print(f"Error sending frame: {e}")

    def send_distance(self, distance):
        try:
            distance_bytes = struct.pack('f', distance)
            self.distance_socket.sendto(distance_bytes, self.distance_address)
            self.gui_distance_socket.sendto(distance_bytes, self.gui_distance_address)
        except Exception as e:
            print(f"Error sending distance: {e}")

    def send_tracked_objects(self, tracked_objects, depth_array):
        try:
            tracked_objects_data = []
            for obj in tracked_objects:
                x1, y1, x2, y2, track_id = obj
                cx = int((x1 + x2) // 2)
                cy = int(y2)
                
                if 0 <= cx < depth_array.shape[1] and 0 <= cy < depth_array.shape[0]:
                    depth = depth_array[cy, cx]
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
                    
        except Exception as e:
            print(f"Error sending tracked objects: {e}")

    def cleanup(self):
        self.distance_socket.close()
        self.video_socket.close()
        self.track_socket.close()
        self.gui_distance_socket.close()

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


def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Ensure img is in the correct format
    if len(img.shape) == 2:  # If grayscale, add channel dimension
        img = np.stack((img,) * 3, axis=-1)
    elif len(img.shape) == 4:  # If RGBA, convert to RGB
        img = img[:, :, :3]
        
    shape = img.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:
        r = min(r, 1.0)

    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    
    if auto:
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)
    elif scaleFill:
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])

    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    
    return img

def driving_area_mask_numpy(seg_array):
    """
    Create driving area mask from segmentation output (numpy version).
    
    Args:
        seg_array: numpy array of segmentation output
        
    Returns:
        Binary mask of driving area
    """
    # Convert 3D array to 2D by taking argmax along last axis if needed
    if len(seg_array.shape) == 3:
        seg_array = np.argmax(seg_array, axis=0)
    elif len(seg_array.shape) == 4:
        seg_array = np.argmax(seg_array[0], axis=0)
        
    # Create binary mask where class 1 is driving area
    mask = (seg_array == 1).astype(np.uint8) * 255
    return mask

def lane_line_mask_numpy(ll_array):
    """
    Create lane line mask from line detection output (numpy version).
    
    Args:
        ll_array: numpy array of lane line detection output
        
    Returns:
        Binary mask of lane lines
    """
    # Handle different input shapes
    if len(ll_array.shape) == 4:
        ll_array = ll_array[0]
    elif len(ll_array.shape) == 3:
        pass  # Already in correct format
    else:
        raise ValueError(f"Unexpected shape for lane line array: {ll_array.shape}")
        
    # If array has multiple channels, take the most confident one
    if ll_array.shape[0] > 1:
        ll_array = np.argmax(ll_array, axis=0)
    else:
        ll_array = ll_array[0] > 0.5  # Threshold if single channel
        
    # Create binary mask
    mask = ll_array.astype(np.uint8) * 255
    return mask

def process_segmentation(seg, ll, im0, img):
    """
    Process segmentation masks with proper alignment and scaling.
    
    Args:
        seg: Segmentation output (tensor or numpy array)
        ll: Lane line output (tensor or numpy array)
        im0: Original image
        img: Preprocessed image tensor
    
    Returns:
        da_seg_mask, ll_seg_mask: Properly aligned segmentation masks
    """
    # Convert tensors to numpy if needed
    if torch.is_tensor(seg):
        seg = seg.cpu().numpy()
    if torch.is_tensor(ll):
        ll = ll.cpu().numpy()
    if torch.is_tensor(img):
        img = img.cpu().numpy()
    
    # Get original and preprocessed dimensions
    orig_h, orig_w = im0.shape[:2]
    
    # Get preprocessed dimensions
    if len(img.shape) == 4:
        lb_h, lb_w = img.shape[2:]
    else:
        lb_h, lb_w = img.shape[:2]
    
    # Process masks using numpy versions of mask functions
    da_seg_mask = driving_area_mask_numpy(seg)
    ll_seg_mask = lane_line_mask_numpy(ll)

    # Resize masks to match original image dimensions
    if da_seg_mask.shape[:2] != (orig_h, orig_w):
        da_seg_mask = cv2.resize(da_seg_mask, (orig_w, orig_h), 
                                interpolation=cv2.INTER_NEAREST)
    
    if ll_seg_mask.shape[:2] != (orig_h, orig_w):
        ll_seg_mask = cv2.resize(ll_seg_mask, (orig_w, orig_h), 
                                interpolation=cv2.INTER_NEAREST)
    
    return da_seg_mask, ll_seg_mask

def apply_segmentation_overlay(frame, da_seg_mask, ll_seg_mask):
    """
    Apply segmentation overlays to frame with proper blending and colors.
    """
    result_img = frame.copy()
    
    # Apply driving area overlay (light blue)
    if da_seg_mask.any():
        da_overlay = np.zeros_like(frame)
        da_overlay[da_seg_mask > 0] = [0, 0, 255]  # BGR light blue
        # Use lower alpha for more subtle blend
        result_img = cv2.addWeighted(result_img, 0.7, da_overlay, 1.0, 0)
    
    # Apply lane line overlay (red)
    if ll_seg_mask.any():
        ll_overlay = np.zeros_like(frame)
        ll_overlay[ll_seg_mask > 0] = [255, 0, 0]  # BGR red
        # Use higher alpha for more visible lane lines
        result_img = cv2.addWeighted(result_img, 0.7, ll_overlay, 1.0, 0)
    
    return result_img

def detect(
    weights='data/weights/yolopv2.pt',
    img_size=640,
    conf_thres=0.3,
    iou_thres=0.45,
    device='0',
    host='localhost',
    port=2000,
    show_vid=False
):
    print("\nInitializing components...")

    # Initialize models
    try:
        device = select_device(device)
        model = torch.jit.load(weights).to(device)
        person_model = YOLO('yolov8n.pt')
        person_model.overrides['classes'] = [0, 4]  # person, car, motorcycle
        # Initialize waypoint extractor
        waypoint_extractor = WaypointExtractor(num_points=6)

        half = device.type != 'cpu'
        if half:
            model.half()
        model.eval()
        print("Models loaded successfully")
    except Exception as e:
        print(f"Error initializing models: {e}")
        return

    # Initialize CARLA camera and network
    try:
        camera = CARLACamera()
        if not camera.open(host, port):
            print("Failed to initialize CARLA camera")
            return

        network = NetworkManager()
        print("Camera and network initialized")
    except Exception as e:
        print(f"Error initializing camera/network: {e}")
        return
    
    # Initialize trackers
    vehicle_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.45)
    person_tracker = Sort(max_age=10, min_hits=3, iou_threshold=0.45)
    
    # Performance monitoring
    fps_avg = AverageMeter()
    frame_count = 0

    # Create visualization window
    if show_vid:
        cv2.namedWindow('CARLA Detection', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('CARLA Detection', 1280, 720)
    
    try:
        print("\nStarting detection loop...")
        while True:
            try:
                # Get frame from CARLA
                im0, depth = camera.grab_frame()
                if im0 is None or depth is None:
                    print("Error: Failed to grab frame")
                    time.sleep(0.1)
                    continue

                frame_count += 1
                if frame_count % 30 == 0:  # Report FPS every 30 frames
                    print(f"Processed {frame_count} frames, FPS: {fps_avg.avg:.1f}")

                result_img = im0.copy()

                # Image preprocessing
                im0_rgb = cv2.cvtColor(im0, cv2.COLOR_BGR2RGB)
                img = letterbox(im0_rgb, new_shape=img_size)
                
                # Convert to tensor
                img = torch.from_numpy(img).to(device).float()
                img /= 255.0
                img = img.permute(2, 0, 1).unsqueeze(0)
                
                if half:
                    img = img.half()

                # Run inference
                t1 = time_synchronized()
                with torch.no_grad():
                    # YOLOPv2 inference
                    [pred, anchor_grid], seg, ll = model(img)
                    
                    # YOLOv8 inference for person detection
                    person_results = person_model.predict(im0, conf=conf_thres, iou=iou_thres)
                
                t2 = time_synchronized()
                fps = 1 / (t2 - t1)
                fps_avg.update(fps)

                # Process YOLOPv2 predictions
                pred = split_for_trace_model(pred, anchor_grid)
                pred = non_max_suppression(pred, conf_thres, iou_thres)

                # Rest of the code remains the same...
                # (Processing detections, updating trackers, drawing results, etc.)
                
                # Process detections
                vehicle_detections = []
                person_detections = []

                # Process vehicle detections
                for det in pred:
                    if len(det):
                        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                        for *xyxy, conf, cls in det.cpu().numpy():
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
                tracked_vehicles = vehicle_tracker.update(
                    np.array(vehicle_detections) if vehicle_detections else np.empty((0, 5))
                )
                tracked_persons = person_tracker.update(
                    np.array(person_detections) if person_detections else np.empty((0, 5))
                )

                try:
                    # Get segmentation masks
                    da_seg_mask, ll_seg_mask = process_segmentation(seg, ll, im0, img)

                    # Extract and visualize waypoints
                    waypoints = waypoint_extractor.extract_waypoints(
                        da_seg_mask, 
                        depth,
                        im0.shape[0],
                        im0.shape[1]
                    )

                    # Visualize waypoints on the result image
                    result_img = waypoint_extractor.visualize_waypoints(result_img, waypoints)
                    # Calculate lane distances
                    left_dist, right_dist = calculate_lane_distances(da_seg_mask, ll_seg_mask, im0.shape[1])

                    # Visualize the distances
                    result_img = visualize_lane_distances(result_img, left_dist, right_dist)
    
                    
                    # Apply segmentation overlay
                    result_img = apply_segmentation_overlay(result_img, da_seg_mask, ll_seg_mask)
                    result_img = cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB)

                except Exception as e:
                    print(f"Error in segmentation processing: {e}")
                    print(f"Segmentation shape: {seg.shape if hasattr(seg, 'shape') else 'None'}")
                    print(f"Lane line shape: {ll.shape if hasattr(ll, 'shape') else 'None'}")
                    print(f"Image shape: {im0.shape}")
                
                # Draw detections
                try:
                    # Draw vehicle boxes
                    for obj in tracked_vehicles:
                        x1, y1, x2, y2, track_id = map(int, obj)
                        cv2.rectangle(result_img, (x1, y1), (x2, y2), (0, 255, 255), 2)
                        cv2.putText(result_img, f'Vehicle {int(track_id)}', (x1, y1-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                    # Draw person boxes
                    for obj in tracked_persons:
                        x1, y1, x2, y2, track_id = map(int, obj)
                        cv2.rectangle(result_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(result_img, f'Person {int(track_id)}', (x1, y1-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    # Add FPS display
                    cv2.putText(result_img, f'FPS: {fps_avg.avg:.1f}', (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                except Exception as e:
                    print(f"Error in drawing detections: {e}")

                # Calculate distances and send data
                try:
                    # Combine tracked objects
                    all_tracked_objects = np.concatenate((tracked_vehicles, tracked_persons)) \
                        if len(tracked_vehicles) > 0 and len(tracked_persons) > 0 \
                        else tracked_vehicles if len(tracked_vehicles) > 0 \
                        else tracked_persons if len(tracked_persons) > 0 \
                        else np.array([])

                    # Find minimum distance
                    min_distance = float('inf')
                    for obj in all_tracked_objects:
                        x1, y1, x2, y2, _ = obj
                        cx = int((x1 + x2) // 2)
                        cy = int(y2)
                        if 0 <= cx < depth.shape[1] and 0 <= cy < depth.shape[0]:
                            dist = depth[cy, cx]
                            if np.isfinite(dist):
                                min_distance = min(min_distance, dist)

                    if min_distance != float('inf'):
                        network.send_distance(min_distance)

                    # Send visualization and tracking data
                    if result_img is not None and result_img.size > 0:
                        cv2.putText(result_img, f'Min Distance: {min_distance:.2f}m', (20, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        network.send_frame(result_img, fps_avg.avg)
                    network.send_tracked_objects(all_tracked_objects, depth)

                except Exception as e:
                    print(f"Error in distance calculation/data transmission: {e}")

                # Show results
                if show_vid:
                    cv2.imshow('CARLA Detection', result_img)

                # Check for exit condition
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("\nDetection stopped by user")
                    break

            except Exception as e:
                print(f"Error in main detection loop: {e}")
                continue

    except KeyboardInterrupt:
        print("\nDetection stopped by user")
    except Exception as e:
        print(f"Critical error in detection: {e}")
    finally:
        print("\nCleaning up...")
        try:
            camera.close()
            network.cleanup()
            cv2.destroyAllWindows()
            print("Cleanup completed successfully")
        except Exception as e:
            print(f"Error during cleanup: {e}")

if __name__ == "__main__":
    try:
        print("\nStarting CARLA Vision Detection System...")
        print("Initializing CARLA client and loading models...")
        with torch.no_grad():
            detect(show_vid=True)
    except KeyboardInterrupt:
        print("\nDetection stopped by user")
    except Exception as e:
        print(f"\nError in main execution: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nProgram terminated")
