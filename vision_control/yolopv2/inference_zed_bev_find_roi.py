import argparse
import time
from pathlib import Path
import cv2
import torch
import pyzed.sl as sl
import numpy as np

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
        self.init_params.camera_resolution = sl.RESOLUTION.HD720  # 1280x720
        self.init_params.camera_fps = 60
        self.image = sl.Mat()

    def open(self):
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"Failed to open ZED camera: {err}")
            exit(1)

    def grab_frame(self):
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            return self.image.get_data()
        return None

    def close(self):
        self.zed.close()

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    """Resize and pad image while meeting stride-multiple constraints"""
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, ratio, (dw, dh)

def make_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='data/weights/yolopv2.pt', help='model.pt path')
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.3, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or cpu')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    return parser

def calibrate_camera(zed):
    points = []
    window_name = "Select 4 Points (TL, BL, TR, BR) - Press 'q' to confirm"

    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
            cv2.circle(image, (x, y), 5, (0, 0, 255), -1)
            cv2.imshow(window_name, image)
            print(f"Point selected: ({x}, {y})")

    image = zed.grab_frame()
    if image is None:
        raise ValueError("Could not grab frame from ZED camera for calibration.")

    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    image_copy = image.copy()

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, click_event)

    print("Click 4 points in the order: Top-Left (TL), Bottom-Left (BL), Top-Right (TR), Bottom-Right (BR)")
    print("Press 'q' after selecting all 4 points to proceed.")

    while True:
        cv2.imshow(window_name, image_copy)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') and len(points) == 4:
            break
        elif key == ord('q'):
            print("Please select exactly 4 points before proceeding.")
            points = []
            image_copy = image.copy()
            cv2.imshow(window_name, image_copy)

    cv2.destroyWindow(window_name)

    tl, bl, tr, br = points
    print(f"Selected coordinates: TL={tl}, BL={bl}, TR={tr}, BR={br}")

    pts1 = np.float32([tl, bl, tr, br])
    pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])

    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    return matrix, pts1, pts2

def transform_boxes(boxes, matrix, output_shape, device):
    """Transform bounding box coordinates using the perspective transform matrix."""
    if len(boxes) == 0:
        return boxes

    # Extract the box coordinates (x1, y1, x2, y2)
    boxes_np = boxes.cpu().numpy() if boxes.is_cuda else boxes.numpy()
    num_boxes = len(boxes_np)

    # Prepare points for transformation: (x1, y1) and (x2, y2) for each box
    points = np.zeros((num_boxes * 2, 1, 2), dtype=np.float32)
    for i in range(num_boxes):
        points[i * 2, 0, :] = [boxes_np[i, 0], boxes_np[i, 1]]  # (x1, y1)
        points[i * 2 + 1, 0, :] = [boxes_np[i, 2], boxes_np[i, 3]]  # (x2, y2)

    # Apply perspective transform
    transformed_points = cv2.perspectiveTransform(points, matrix)

    # Reshape back to bounding box format
    transformed_boxes = np.zeros_like(boxes_np)
    for i in range(num_boxes):
        transformed_boxes[i, 0] = transformed_points[i * 2, 0, 0]  # x1
        transformed_boxes[i, 1] = transformed_points[i * 2, 0, 1]  # y1
        transformed_boxes[i, 2] = transformed_points[i * 2 + 1, 0, 0]  # x2
        transformed_boxes[i, 3] = transformed_points[i * 2 + 1, 0, 1]  # y2

    # Clip coordinates to the output shape
    transformed_boxes[:, [0, 2]] = np.clip(transformed_boxes[:, [0, 2]], 0, output_shape[1])
    transformed_boxes[:, [1, 3]] = np.clip(transformed_boxes[:, [1, 3]], 0, output_shape[0])

    # Copy back to the original tensor, preserving confidence and class
    result = boxes_np.copy()
    result[:, :4] = transformed_boxes
    return torch.from_numpy(result).to(device)

def detect():
    weights, imgsz = opt.weights, opt.img_size

    save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))
    save_dir.mkdir(parents=True, exist_ok=True)

    zed = ZEDCamera()
    zed.open()

    matrix, pts1, pts2 = calibrate_camera(zed)

    device = select_device(opt.device)
    model = torch.jit.load(weights).to(device)
    half = device.type != 'cpu'
    if half:
        model.half()
    model.eval()

    fps_avg = AverageMeter()

    print("Starting ZED camera stream with YOLOPv2 inference on original frame, results transformed to bird's-eye view. Press 'q' to quit.")
    try:
        while True:
            im0 = zed.grab_frame()
            if im0 is None:
                print("Failed to grab frame. Exiting.")
                break

            # Convert BGRA to RGB for inference
            im0_rgb = cv2.cvtColor(im0, cv2.COLOR_BGRA2RGB)

            # Prepare the original frame for inference
            img, ratio, (dw, dh) = letterbox(im0_rgb, new_shape=(imgsz, imgsz))
            img = img.transpose(2, 0, 1)  # HWC to CHW
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(device).float()
            img /= 255.0  # Normalize
            img = img.unsqueeze(0)  # Add batch dimension

            if half:
                img = img.half()

            # Inference on the original frame
            t1 = time_synchronized()
            [pred, anchor_grid], seg, ll = model(img)
            t2 = time_synchronized()

            # Calculate and update FPS
            fps = 1 / (t2 - t1)
            fps_avg.update(fps)

            # Apply perspective transform to the original frame for display
            transformed_frame = cv2.warpPerspective(im0_rgb, matrix, (640, 480))

            # Create a blank canvas for the transformed results
            result_frame = transformed_frame.copy()

            # Apply NMS to detection results
            pred = split_for_trace_model(pred, anchor_grid)
            pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, 
                                       classes=opt.classes, agnostic=opt.agnostic_nms)

            # Process detection and segmentation results
            da_seg_mask = driving_area_mask(seg)
            ll_seg_mask = lane_line_mask(ll)

            # Resize segmentation masks to match the original frame's dimensions
            da_seg_mask = cv2.resize(da_seg_mask, (im0_rgb.shape[1], im0_rgb.shape[0]), 
                                     interpolation=cv2.INTER_NEAREST)
            ll_seg_mask = cv2.resize(ll_seg_mask, (im0_rgb.shape[1], im0_rgb.shape[0]), 
                                     interpolation=cv2.INTER_NEAREST)

            # Transform segmentation masks to bird's-eye view
            da_seg_mask_transformed = cv2.warpPerspective(da_seg_mask, matrix, (640, 480), flags=cv2.INTER_NEAREST)
            ll_seg_mask_transformed = cv2.warpPerspective(ll_seg_mask, matrix, (640, 480), flags=cv2.INTER_NEAREST)

            # Transform bounding boxes
            for i, det in enumerate(pred):
                if len(det):
                    # Rescale boxes to the original frame's dimensions
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0_rgb.shape).round()
                    # Transform boxes to bird's-eye view
                    pred[i][:, :4] = transform_boxes(det[:, :4], matrix, (480, 640), device)

            # Overlay transformed detection results on the result frame
            for det in pred:
                if len(det):
                    for *xyxy, conf, cls in reversed(det):
                        plot_one_box(xyxy, result_frame, line_thickness=3)

            # Overlay transformed segmentation results
            show_seg_result(result_frame, (da_seg_mask_transformed, ll_seg_mask_transformed), is_demo=True)

            # Display FPS on the result frame
            cv2.putText(result_frame, f'FPS: {fps_avg.avg:.2f}', (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Convert frames for display
            im0_bgr = cv2.cvtColor(im0, cv2.COLOR_BGRA2BGR)
            result_display = cv2.cvtColor(result_frame, cv2.COLOR_RGB2BGR)

            # Display the original frame and the transformed results
            cv2.imshow("Original ZED Stream", im0_bgr)
            cv2.imshow("Transformed YOLOPv2 Stream - Bird's Eye View", result_display)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        zed.close()
        cv2.destroyAllWindows()
        print("Stream ended. Exiting.")

if __name__ == '__main__':
    opt = make_parser().parse_args()
    print(opt)
    with torch.no_grad():
        detect()