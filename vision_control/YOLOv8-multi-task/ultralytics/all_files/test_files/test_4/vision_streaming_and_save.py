import pyzed.sl as sl
import cv2
import time
import os
import signal
import sys
from datetime import datetime

class ZEDCamera:
    def __init__(self):
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 30
        self.init_params.depth_mode = sl.DEPTH_MODE.NONE
        
        self.image = sl.Mat()
        self.video_writer = None
        self.recording = False
        self.frame_count = 0  # Add frame counter
        
        # Register signal handlers for proper cleanup
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print("\nReceived shutdown signal. Cleaning up...")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        """Ensure proper cleanup of resources"""
        try:
            if self.recording and self.video_writer is not None:
                print(f"Saving video file... ({self.frame_count} frames written)")
                self.video_writer.release()
                self.recording = False
                print("Video saved successfully")
            
            if self.zed is not None:
                self.zed.close()
                print("Camera closed successfully")
                
        except Exception as e:
            print(f"Error during cleanup: {e}")
        finally:
            cv2.destroyAllWindows()

    def open(self):
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Failed to open ZED camera: {err}")
            sys.exit(1)
            
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 4)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, 4)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, 4)
        
        self.start_recording()

    def start_recording(self):
        if not self.recording:
            try:
                if not os.path.exists('videos'):
                    os.makedirs('videos')
                
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"videos/zed_recording_{timestamp}.mp4"
                
                # Use MJPG codec which is more widely supported
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                    
                self.video_writer = cv2.VideoWriter(
                    filename,
                    fourcc,
                    30.0,
                    (1280, 720),
                    isColor=True
                )
                
                if self.video_writer.isOpened():
                    self.recording = True
                    print(f"Started recording to {filename}")
                    print(f"Using MJPG codec")
                else:
                    print("Failed to initialize video writer. Make sure proper codecs are installed.")
                    self.video_writer = None
            
            except Exception as e:
                print(f"Error starting recording: {e}")

    def grab_frame(self):
        try:
            if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
                frame = self.image.get_data()
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
                
                if self.recording and self.video_writer is not None:
                    try:
                        self.video_writer.write(frame_rgb)
                        self.frame_count += 1
                        if self.frame_count % 30 == 0:  # Print every second (assuming 30 fps)
                            print(f"Frames recorded: {self.frame_count}", end='\r')
                    except Exception as e:
                        print(f"Error writing frame: {e}")
                
                return frame_rgb
            return None
        except Exception as e:
            print(f"Error grabbing frame: {e}")
            return None

def main():
    camera = None
    try:
        camera = ZEDCamera()
        camera.open()
        
        print("ZED Camera Stream")
        print("Recording automatically started")
        print("Press 'q' to quit or use Ctrl+C")
        
        while True:
            frame = camera.grab_frame()
            if frame is None:
                continue
            
            cv2.imshow("ZED Stream", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nQuitting program...")
                break
                
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        if camera is not None:
            camera.cleanup()

if __name__ == "__main__":
    main()