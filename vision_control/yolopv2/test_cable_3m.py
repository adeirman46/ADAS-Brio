# # taking video cv2 imshow

# import cv2

# cap = cv2.VideoCapture(1)

# while True:
#     ret, frame = cap.read()
#     cv2.imshow('frame', frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# cap.release()
# cv2.destroyAllWindows()

import cv2

# Open the camera (0 for default camera, 1 for another camera)
cap = cv2.VideoCapture(1)  # Try using 0 or 1 depending on the available camera

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    ret, frame = cap.read()  # Capture frame-by-frame
    
    # Check if the frame was successfully captured
    if not ret:
        print("Error: Failed to capture frame.")
        break
    
    # Display the frame
    cv2.imshow('frame', frame)
    
    # Wait for the user to press the 'q' key to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
