import cv2
import numpy as np

# Global variables to store the coordinates
points = []
window_name = "Select 4 Points (TL, BL, TR, BR) - Press 'q' to confirm"

# Mouse callback function to capture clicks
def click_event(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        # Draw a small circle at the clicked point
        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow(window_name, image)
        print(f"Point selected: ({x}, {y})")

# Load the image
image = cv2.imread("bdd1.jpg")
if image is None:
    raise ValueError("Could not load the image. Check the file path.")

# Create a copy of the image to display points
image_copy = image.copy()

# Set up the window and mouse callback
cv2.namedWindow(window_name)
cv2.setMouseCallback(window_name, click_event)

# Display the image and wait for 4 points to be selected
print("Click 4 points in the order: Top-Left (TL), Bottom-Left (BL), Top-Right (TR), Bottom-Right (BR)")
print("Press 'q' after selecting all 4 points to proceed.")

while True:
    cv2.imshow(window_name, image_copy)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') and len(points) == 4:
        break
    elif key == ord('q'):
        print("Please select exactly 4 points before proceeding.")
        points = []  # Reset points if not exactly 4
        image_copy = image.copy()  # Reset the image
        cv2.imshow(window_name, image_copy)

cv2.destroyWindow(window_name)

# Extract the selected coordinates
tl, bl, tr, br = points
print(f"Selected coordinates: TL={tl}, BL={bl}, TR={tr}, BR={br}")

# Define the source points
pts1 = np.float32([tl, bl, tr, br])

# Define the destination points (output size: 640x480)
pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])

# Get the perspective transform matrix
matrix = cv2.getPerspectiveTransform(pts1, pts2)

# Apply the warp perspective transformation
transformed_image = cv2.warpPerspective(image, matrix, (640, 480))

# Display the original and transformed images
cv2.imshow("Original Image", image)
cv2.imshow("Transformed Image - Bird's Eye View", transformed_image)
cv2.waitKey(0)
cv2.destroyAllWindows()