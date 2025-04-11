
import cv2
import time
from ultralytics import YOLO
import pyzed.sl as sl

# Load the YOLOv8 model
model = YOLO('/home/irman/Documents/YOLOPv2/runs/detect/train/weights/best.pt')  # Load the YOLOv8 model

# Initialize the ZED camera
zed = sl.Camera()

# Set up ZED camera parameters
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 resolution
init_params.camera_fps = 60  # Set the camera FPS


# Open the ZED camera
if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("Error: Unable to open ZED camera.")
    exit()

runtime_params = sl.RuntimeParameters()

# Create ZED Mat to retrieve images
image = sl.Mat()

print("Press 'q' to exit.")

# Initialize variables for FPS calculation
fps = 0
frame_count = 0
start_time = time.time()

while True:
    # Start timer for FPS calculation
    frame_start_time = time.time()

    # Grab a new frame from the ZED camera
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # Retrieve the left image from the ZED camera
        zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = image.get_data()

        # Convert the ZED frame to OpenCV format
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        # Perform inference on the frame
        results = model.predict(frame_bgr, conf=0.7, iou=0.45)  # Adjust thresholds as needed

        # Render the results on the frame (bounding boxes, labels, etc.)
        frame_with_boxes = results[0].plot()  # This plots the results directly on the frame

        # Calculate FPS dynamically
        frame_end_time = time.time()
        total_frame_time = frame_end_time - frame_start_time
        fps = 1 / total_frame_time  # FPS = 1 / frame processing time

        # Display FPS on the frame
        cv2.putText(frame_with_boxes, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Display the frame with bounding boxes
        cv2.imshow("YOLOv8 ZED Inference", frame_with_boxes)

        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the ZED camera and close the OpenCV window
zed.close()
cv2.destroyAllWindows()
