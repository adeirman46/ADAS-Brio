import cv2
import time
import ollama
import threading

class FacialExpressionDetector:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.current_expression = "Starting..."
        self.inference_lock = threading.Lock()
        self.inference_running = False
        self.min_interval = 1.0  # Minimum delay between inferences to prevent overloading
        self.last_inference_time = 0
        
    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame
    
    def run_inference(self, frame):
        # Set flag to indicate inference is running
        self.inference_running = True
        
        # Save frame temporarily
        temp_path = "temp_frame.jpg"
        cv2.imwrite(temp_path, frame)
        
        # Create prompt for Gemma
        # prompt = """You are a facial expression expert. Analyze this image and determine if the person is smiling, angry, sad, or neutral.
        # Give the reason in 1 sentence.
        # """

        prompt = """
        Anda adalah ahli ekspresi wajah. Analisis gambar ini dan tentukan apakah orang tersebut tersenyum, marah, sedih, atau netral. Berikan alasan dalam 1 kalimat.
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
                self.current_expression = content
                print(f"Raw inference: {content}")
                
        except Exception as e:
            self.current_expression = f"Error: {str(e)}"
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
            
            # Show expression on frame
            cv2.putText(
                frame,
                self.current_expression,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )
            
            # Schedule a new inference if the previous one is complete and minimum interval passed
            current_time = time.time()
            time_since_last = current_time - self.last_inference_time
            
            if not self.inference_running and time_since_last >= self.min_interval:
                self.schedule_inference()
            
            # Show inference status
            status = "Analyzing..." if self.inference_running else f"Ready (last: {time_since_last:.1f}s ago)"
            cv2.putText(
                frame,
                status,
                (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )
            
            # Show the frame
            cv2.imshow('Facial Expression Detection', frame)
            
            # Check for exit key (q)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Clean up
        self.cap.release()
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
    print("Starting Facial Expression Detector...")
    print("Press 'q' to quit")
    detector = FacialExpressionDetector()
    detector.run()