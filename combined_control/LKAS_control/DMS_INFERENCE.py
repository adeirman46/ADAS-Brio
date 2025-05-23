import cv2
import dlib
import numpy as np
from scipy.spatial import distance
from imutils import face_utils
import time
import socket
import struct
from pygame import mixer
from collections import deque

class DMSInference:
    def __init__(self):
        """Initialize DMS inference system with facial landmarks."""
        # Initialize face detector and facial landmarks predictor
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor("/home/irman/ADAS-Brio/vision_control/yolopv2/shape_predictor_68_face_landmarks.dat")
        
        # Get indexes for eyes and mouth
        (self.lStart, self.lEnd) = face_utils.FACIAL_LANDMARKS_IDXS["left_eye"]
        (self.rStart, self.rEnd) = face_utils.FACIAL_LANDMARKS_IDXS["right_eye"]
        (self.mStart, self.mEnd) = face_utils.FACIAL_LANDMARKS_IDXS["mouth"]
        
        # Constants for drowsiness detection - adjusted thresholds
        self.EAR_THRESHOLD = 0.23
        self.MAR_THRESHOLD = 0.75  # Adjusted for smaller mouth area
        self.EYE_AR_CONSEC_FRAMES = 20
        self.MOUTH_AR_CONSEC_FRAMES = 15
        self.HEAD_POSE_THRESHOLD = 35
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Error: Could not open camera")
        
        # Set camera resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Initialize UDP socket for GUI
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gui_address = ('localhost', 12350)
        
        # Initialize audio with multiple sounds
        mixer.init()
        self.alert_sounds = {
            'drowsy': mixer.Sound('/home/irman/ADAS-Brio/vision_control/yolopv2/driver_drowsy.mp3'),
            'head': mixer.Sound('/home/irman/ADAS-Brio/vision_control/yolopv2/driver_not_focus_lane.mp3')
        }
        self.last_alert_times = {
            'drowsy': 0,
            'head': 0
        }
        self.alert_cooldowns = {
            'drowsy': 3.0,
            'head': 2.0
        }
        
        # State tracking with separate counters
        self.running = True
        self.eye_drowsy_counter = 0
        self.mouth_drowsy_counter = 0
        self.not_forward_counter = 0
        
        # Add rolling windows for smoothing
        self.ear_window = deque(maxlen=5)
        self.mar_window = deque(maxlen=5)
        self.yaw_window = deque(maxlen=5)
        
        # FPS calculation
        self.prev_time = time.time()
        self.fps = 0
        self.alpha = 0.1
        
        # Drowsiness state
        self.is_drowsy = False
        self.drowsy_start_time = None

    def calculate_EAR(self, eye):
        """Calculate eye aspect ratio with improved noise handling."""
        try:
            A = distance.euclidean(eye[1], eye[5])
            B = distance.euclidean(eye[2], eye[4])
            C = distance.euclidean(eye[0], eye[3])
            ear = (A + B) / (2.0 * C) if C > 0 else 0
            return min(ear, 0.5)  # Cap maximum EAR to filter outliers
        except:
            return 0.3  # Default value if calculation fails

    def calculate_MAR(self, mouth):
        """Calculate mouth aspect ratio with improved focus on inner mouth area."""
        try:
            # Vertical distances - using inner mouth points
            A = distance.euclidean(mouth[2], mouth[10])  # Inner vertical line 1
            B = distance.euclidean(mouth[3], mouth[9])   # Inner vertical line 2
            C = distance.euclidean(mouth[4], mouth[8])   # Inner vertical line 3
            
            # Horizontal distance - using inner mouth points
            D = distance.euclidean(mouth[0], mouth[6])   # Inner horizontal line
            
            # Weighted MAR calculation
            mar = (A + B + C) / (3.0 * D) if D > 0 else 0
            
            # Normalize and limit value
            mar = min(mar * 1.5, 1.0)  # Scale and cap
            
            return mar
        except:
            return 0.2  # Default value if calculation fails

    def calculate_head_pose(self, shape, frame):
        """Calculate head pose with improved stability."""
        try:
            image_points = np.array([
                shape[33],    # Nose tip
                shape[8],     # Chin
                shape[36],    # Left eye left corner
                shape[45],    # Right eye right corner
                shape[48],    # Left mouth corner
                shape[54]     # Right mouth corner
            ], dtype="double")

            model_points = np.array([
                (0.0, 0.0, 0.0),
                (0.0, -330.0, -65.0),
                (-225.0, 170.0, -135.0),
                (225.0, 170.0, -135.0),
                (-150.0, -150.0, -125.0),
                (150.0, -150.0, -125.0)
            ])

            size = frame.shape
            focal_length = size[1]
            center = (size[1]/2, size[0]/2)
            camera_matrix = np.array([
                [focal_length, 0, center[0]],
                [0, focal_length, center[1]],
                [0, 0, 1]
            ], dtype="double")

            dist_coeffs = np.zeros((4,1))
            _, rotation_vector, translation_vector = cv2.solvePnP(
                model_points, image_points, camera_matrix, dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE)

            rotation_mat, _ = cv2.Rodrigues(rotation_vector)
            pose_mat = cv2.hconcat((rotation_mat, translation_vector))
            _, _, _, _, _, _, euler_angles = cv2.decomposeProjectionMatrix(pose_mat)
            
            return euler_angles
        except:
            return np.array([[0.0], [0.0], [0.0]])

    def play_alert(self, alert_type="drowsy"):
        """Play alert with type-specific cooldown."""
        current_time = time.time()
        last_alert = self.last_alert_times[alert_type]
        cooldown = self.alert_cooldowns[alert_type]
        
        if current_time - last_alert >= cooldown:
            self.alert_sounds[alert_type].play()
            self.last_alert_times[alert_type] = current_time

    def check_drowsiness(self, ear, mar, yaw):
        """Check drowsiness with improved logic and separate conditions."""
        # Update rolling windows
        self.ear_window.append(ear)
        self.mar_window.append(mar)
        self.yaw_window.append(abs(yaw))
        
        # Get smooth values
        avg_ear = np.mean(self.ear_window) if self.ear_window else ear
        avg_mar = np.mean(self.mar_window) if self.mar_window else mar
        avg_yaw = np.mean(self.yaw_window) if self.yaw_window else abs(yaw)
        
        # Check if head is in valid position for drowsiness detection
        head_position_valid = avg_yaw <= self.HEAD_POSE_THRESHOLD
        
        # Only increment drowsy counters if head position is valid
        if head_position_valid:
            # Check eyes
            if avg_ear < self.EAR_THRESHOLD:
                self.eye_drowsy_counter += 1
            else:
                self.eye_drowsy_counter = max(0, self.eye_drowsy_counter - 1)
            
            # Check mouth
            if avg_mar > self.MAR_THRESHOLD:
                self.mouth_drowsy_counter += 1
            else:
                self.mouth_drowsy_counter = max(0, self.mouth_drowsy_counter - 1)
        else:
            # Reset counters if head position is invalid
            self.eye_drowsy_counter = 0
            self.mouth_drowsy_counter = 0
        
        # Determine drowsiness state
        eyes_closed = self.eye_drowsy_counter >= self.EYE_AR_CONSEC_FRAMES
        yawning = self.mouth_drowsy_counter >= self.MOUTH_AR_CONSEC_FRAMES
        
        return eyes_closed, yawning, head_position_valid

    def process_frame(self, frame):
        """Process frame with improved drowsiness detection."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.detector(gray, 0)
        
        if len(faces) == 0:
            # Reset counters when no face is detected
            self.eye_drowsy_counter = 0
            self.mouth_drowsy_counter = 0
            self.not_forward_counter = 0
            return frame
        
        for face in faces:
            shape = self.predictor(gray, face)
            shape = face_utils.shape_to_np(shape)
            
            # Extract facial features
            leftEye = shape[self.lStart:self.lEnd]
            rightEye = shape[self.rStart:self.rEnd]
            mouth = shape[self.mStart:self.mEnd]
            
            # Calculate metrics
            ear = (self.calculate_EAR(leftEye) + self.calculate_EAR(rightEye)) / 2.0
            mar = self.calculate_MAR(mouth)
            euler_angles = self.calculate_head_pose(shape, frame)
            yaw = euler_angles[1]
            
            # Draw facial features
            leftEyeHull = cv2.convexHull(leftEye)
            rightEyeHull = cv2.convexHull(rightEye)
            mouthHull = cv2.convexHull(mouth)
            
            cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
            cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
            cv2.drawContours(frame, [mouthHull], -1, (0, 255, 0), 1)
            
            # Draw inner mouth points for visualization
            inner_mouth_points = [mouth[2], mouth[3], mouth[4], mouth[8], mouth[9], mouth[10]]
            for point in inner_mouth_points:
                cv2.circle(frame, tuple(point), 2, (0, 0, 255), -1)
            
            # Check drowsiness conditions
            eyes_closed, yawning, head_valid = self.check_drowsiness(ear, mar, yaw[0])
            
            # Update alerts and display
            if head_valid:
                if eyes_closed:
                    cv2.putText(frame, "EYES CLOSED!", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self.play_alert("drowsy")
                
                if yawning:
                    cv2.putText(frame, "YAWNING!", (10, 60),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    self.play_alert("drowsy")
            else:
                if abs(yaw) > self.HEAD_POSE_THRESHOLD:
                    self.not_forward_counter += 1
                    if self.not_forward_counter >= 10:
                        cv2.putText(frame, "HEAD NOT FORWARD!", (10, 90),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        self.play_alert("head")  # Play head alert sound
                else:
                    self.not_forward_counter = 0
            
            # Display metrics
            cv2.putText(frame, f"EAR: {ear:.2f}", (300, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"MAR: {mar:.2f}", (300, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Yaw: {yaw[0]:.2f}", (300, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return frame

    def send_frame_to_gui(self, frame):
        """Send processed frame to GUI via UDP."""
        try:
            _, encoded_frame = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            frame_bytes = encoded_frame.tobytes()
            
            size_bytes = struct.pack('I', len(frame_bytes))
            self.socket.sendto(size_bytes, self.gui_address)
            
            chunk_size = 8192
            for i in range(0, len(frame_bytes), chunk_size):
                chunk = frame_bytes[i:i + chunk_size]
                self.socket.sendto(chunk, self.gui_address)
        except Exception as e:
            print(f"Error sending frame to GUI: {e}")

    def calculate_fps(self):
        """Calculate smoothed FPS."""
        current_time = time.time()
        instantaneous_fps = 1 / (current_time - self.prev_time)
        self.fps = (1 - self.alpha) * self.fps + self.alpha * instantaneous_fps
        self.prev_time = current_time
        return self.fps

    def run(self):
        """Main processing loop."""
        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    print("Error: Could not read frame")
                    break
                
                frame = self.process_frame(frame)
                
                fps = self.calculate_fps()
                cv2.putText(frame, f'DMS FPS: {fps:.1f}', (10, 120),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
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
        dms = DMSInference()
        dms.run()
    except Exception as e:
        print(f"Error in DMS main: {e}")

if __name__ == "__main__":
    main()