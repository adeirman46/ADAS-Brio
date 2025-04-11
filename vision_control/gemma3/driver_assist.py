import cv2
import time
import ollama
import threading
import pyzed.sl as sl
import json

class DriverAssistant:
    def __init__(self):
        # Initialize ZED camera
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 60
        self.init_params.coordinate_units = sl.UNIT.METER
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        self.image = sl.Mat()
        self.depth = sl.Mat()
        
        # Open ZED camera
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Failed to open ZED camera: {err}")
            exit(1)
            
        self.current_recommendation = "Starting..."
        self.inference_lock = threading.Lock()
        self.inference_running = False
        self.min_interval = 1.0  # Minimum delay between inferences to prevent overloading
        self.last_inference_time = 0
        
    def capture_frame(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
            
            image_ocv = self.image.get_data()
            
            if image_ocv is not None and image_ocv.shape[2] == 4:
                image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_BGRA2BGR)
                
            return image_ocv
        return None
    
    def run_inference(self, frame):
        # Set flag to indicate inference is running
        self.inference_running = True
        
        # Save frame temporarily
        temp_path = "temp_frame.jpg"
        cv2.imwrite(temp_path, frame)

        prompt = """You are an expert driver assistance system. Given the current speed: 20 kph, steer: 0, provide values for the next speed (0 to 30 kph) and next steer (-1 to 1, representing steering angle).
        I want to drive moving forward with Lane Keeping Assist System (LKAS) to follow lines or lanes, Adaptive Cruise Control (ACC) to maintain a safe speed, and Automatic Emergency Braking (AEB) to stop if needed. If the car is moving forward, provide the next speed and steer values. If the car is not moving forward, set next_speed to 0.
        If the environment is blurry, black, not defined, or if there is an obstacle nearby in front, set next_speed to 0 for safety. Provide the reason in one sentence. 
        Parse the values in {"next_speed": value, "next_steer": value}. 
        """
        try:
            # Call Gemma model
            with self.inference_lock:
                response = ollama.chat(
                    model='gemma3:4b',
                    messages=[{
                        'role': 'user',
                        'content': prompt,
                        'images': [temp_path]
                    }]
                )
                
                # Parse response
                content = response['message']["content"]
                self.current_recommendation = content
                print(f"Raw inference: {content}")
                
                try:
                    # Look for JSON pattern in the response
                    import re
                    json_pattern = r'(\{[^}]*\})'
                    json_match = re.search(json_pattern, content)
                    
                    if json_match:
                        json_str = json_match.group(1)
                        # Parse the JSON
                        json_data = json.loads(json_str)
                        
                        # Handle specific variations of "next_steer"
                        steer_key = None
                        valid_steer_variations = {"next_steer", "next_steeer", "next_stear"}
                        for key in json_data.keys():
                            if key.lower() in valid_steer_variations or key.lower().startswith("next_") and key.lower()[5:] in valid_steer_variations:
                                steer_key = key
                                break
                        
                        # Extract values with fallback
                        speed = json_data.get('next_speed', 0)
                        steer = json_data.get(steer_key, 0) if steer_key else json_data.get('next_steer', 0)
                        
                        # Ensure steer is within valid range (-1 to 1)
                        steer = max(min(steer, 1), -1)
                        
                        # Extract only the reason part
                        reason_match = re.search(r'\*\*Reason:\*\*(.*?)(?:$|\.)', content, re.DOTALL)
                        if reason_match:
                            reason = reason_match.group(1).strip()
                        else:
                            # Fallback: try to find text after the JSON that looks like a reason
                            parts = content.split(json_str)
                            if len(parts) > 1:
                                after_json = parts[1].strip()
                                # Remove common prefixes
                                for prefix in ["**Reason:**", "Reason:", "Reason is:", "Because:", "**Reasoning:**"]:
                                    if prefix in after_json:
                                        reason = after_json.split(prefix, 1)[1].strip()
                                        break
                                else:
                                    # If no prefix found, use first sentence after JSON
                                    sentences = re.split(r'(?<=[.!?])\s+', after_json)
                                    reason = sentences[0] if sentences else "Unknown reason"
                            else:
                                reason = "Unknown reason"
                        
                        self.current_recommendation = f"Speed: {speed:.1f} kph S: {steer:.2f} - {reason}"
                except Exception as e:
                    print(f"JSON parsing error: {e}")
                    self.current_recommendation = f"Error: JSON parsing failed - {str(e)}"
                
        except Exception as e:
            self.current_recommendation = f"Error: {str(e)}"
            print(f"Inference error: {e}")
        
        # Record the time when inference completed
        self.last_inference_time = time.time()
        
        # Set flag to indicate inference is complete
        self.inference_running = False
    
    def run(self):
        # Start the first inference
        self.schedule_inference()
        
        while True:
            frame = self.capture_frame()
            if frame is None:
                break
            
            # Show recommendations at the bottom of the frame
            h, w = frame.shape[:2]
            
            # Show recommendation on frame (at the bottom)
            cv2.putText(
                frame,
                self.current_recommendation,
                (20, h - 30),  # Position at the bottom
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,  # Smaller font size
                (0, 255, 0),
                1  # Thinner line
            )
            
            # Schedule a new inference if the previous one is complete and minimum interval passed
            current_time = time.time()
            time_since_last = current_time - self.last_inference_time
            
            if not self.inference_running and time_since_last >= self.min_interval:
                self.schedule_inference()
            
            # Show inference status (at the bottom)
            status = "Analyzing..." if self.inference_running else f"Ready (last: {time_since_last:.1f}s ago)"
            cv2.putText(
                frame,
                status,
                (20, h - 10),  # Position at the bottom
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,  # Smaller font size
                (0, 0, 255),
                1  # Thinner line
            )
            
            # Show the frame
            cv2.imshow('Driver Expert Assistant', frame)
            
            # Check for exit key (q)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Clean up
        self.zed.close()
        cv2.destroyAllWindows()
    
    def schedule_inference(self):
        """Start a new inference in a background thread"""
        frame = self.capture_frame()
        if frame is not None:
            threading.Thread(
                target=self.run_inference,
                args=(frame.copy(),),
                daemon=True
            ).start()

if __name__ == "__main__":
    print("Starting Driver Expert Assistant...")
    print("Press 'q' to quit")
    assistant = DriverAssistant()
    assistant.run()