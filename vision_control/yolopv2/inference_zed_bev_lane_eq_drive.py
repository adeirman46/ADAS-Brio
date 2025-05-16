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
        self.init_params.coordinate_units = sl.UNIT.METER
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
            return self.image.get_data(), self.depth.get_data()
        return None, None

    def close(self):
        self.zed.close()

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    """Resize and pad image while meeting stride-multiple constraints"""
    shape = img.shape[:2]  # current shape [height, width]
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

def make_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='data/weights/yolopv2.pt', help='model.pt path')
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.6, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or cpu')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
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

    pts1 = np.float32([tl, bl, tr, br])
    pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])

    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    return matrix, pts1, pts2

def draw_roi(frame, pts1):
    """Draw the ROI quadrilateral on the frame using the given points in sequential order."""
    tl, bl, tr, br = pts1
    pts_draw = np.array([tl, tr, br, bl], np.int32).reshape((-1, 1, 2))
    cv2.polylines(frame, [pts_draw], isClosed=True, color=(0, 0, 255), thickness=2)
    return frame

def transform_boxes(boxes, matrix, output_shape, device):
    """Transform bounding box coordinates using the perspective transform matrix."""
    if len(boxes) == 0:
        return boxes

    boxes_np = boxes.cpu().numpy() if boxes.is_cuda else boxes.numpy()
    num_boxes = len(boxes_np)

    points = np.zeros((num_boxes * 2, 1, 2), dtype=np.float32)
    for i in range(num_boxes):
        points[i * 2, 0, :] = [boxes_np[i, 0], boxes_np[i, 1]]
        points[i * 2 + 1, 0, :] = [boxes_np[i, 2], boxes_np[i, 3]]

    transformed_points = cv2.perspectiveTransform(points, matrix)

    transformed_boxes = np.zeros_like(boxes_np)
    for i in range(num_boxes):
        transformed_boxes[i, 0] = transformed_points[i * 2, 0, 0]
        transformed_boxes[i, 1] = transformed_points[i * 2, 0, 1]
        transformed_boxes[i, 2] = transformed_points[i * 2 + 1, 0, 0]
        transformed_boxes[i, 3] = transformed_points[i * 2 + 1, 0, 1]

    transformed_boxes[:, [0, 2]] = np.clip(transformed_boxes[:, [0, 2]], 0, output_shape[1])
    transformed_boxes[:, [1, 3]] = np.clip(transformed_boxes[:, [1, 3]], 0, output_shape[0])

    result = boxes_np.copy()
    result[:, :4] = transformed_boxes
    return torch.from_numpy(result).to(device)

def fit_polynomial(lane_mask):
    """Fit polynomial curves to lane lines"""
    nonzero = lane_mask.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    midpoint = lane_mask.shape[1] // 2
    left_lane_inds = nonzerox < midpoint
    right_lane_inds = nonzerox >= midpoint
    
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    
    left_fit = None
    right_fit = None
    
    if len(leftx) > 10 and len(lefty) > 10:
        try:
            left_fit = np.polyfit(lefty, leftx, 2)
        except:
            left_fit = None
    
    if len(rightx) > 10 and len(righty) > 10:
        try:
            right_fit = np.polyfit(righty, rightx, 2)
        except:
            right_fit = None
    
    return left_fit, right_fit

def draw_polynomial_lines(image, left_fit, right_fit, color=(255, 0, 0), thickness=3):
    """Draw polynomial lines on the image"""
    ploty = np.linspace(0, image.shape[0]-1, image.shape[0])
    
    if left_fit is not None:
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        left_points = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        cv2.polylines(image, np.int32([left_points]), False, color, thickness)
    
    if right_fit is not None:
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        right_points = np.array([np.transpose(np.vstack([right_fitx, ploty]))])
        cv2.polylines(image, np.int32([right_points]), False, color, thickness)
    
    return image

def detect():
    weights, imgsz = opt.weights, opt.img_size

    save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))
    save_dir.mkdir(parents=True, exist_ok=True)

    zed = ZEDCamera()
    zed.open()

    matrix, pts1, pts2 = calibrate_camera(zed, opt.calibrate)

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
            im0, depth = zed.grab_frame()
            if im0 is None:
                print("Failed to grab frame. Exiting.")
                break

            im0_rgb = cv2.cvtColor(im0, cv2.COLOR_BGRA2RGB)

            img, ratio, (dw, dh) = letterbox(im0_rgb, new_shape=(imgsz, imgsz))
            img = img.transpose(2, 0, 1)
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(device).float()
            img /= 255.0
            img = img.unsqueeze(0)

            if half:
                img = img.half()

            t1 = time_synchronized()
            [pred, anchor_grid], seg, ll = model(img)
            t2 = time_synchronized()

            fps = 1 / (t2 - t1)
            fps_avg.update(fps)

            transformed_frame = cv2.warpPerspective(im0_rgb, matrix, (640, 480))
            result_frame = transformed_frame.copy()

            pred = split_for_trace_model(pred, anchor_grid)
            pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, 
                                       classes=opt.classes, agnostic=opt.agnostic_nms)

            ll_seg_mask = lane_line_mask(ll)
            ll_seg_mask = cv2.resize(ll_seg_mask, (im0_rgb.shape[1], im0_rgb.shape[0]), 
                                     interpolation=cv2.INTER_NEAREST)
            da_seg_mask = driving_area_mask(seg)
            da_seg_mask = cv2.resize(da_seg_mask, (im0_rgb.shape[1], im0_rgb.shape[0]), 
                                            interpolation=cv2.INTER_NEAREST)
            
            ll_seg_mask_transformed = cv2.warpPerspective(ll_seg_mask, matrix, (640, 480), flags=cv2.INTER_NEAREST)
            da_seg_mask_transformed = cv2.warpPerspective(da_seg_mask, matrix, (640, 480), 
                                                                flags=cv2.INTER_NEAREST)

            im0_rgb_with_roi = im0_rgb.copy()
            im0_rgb_with_roi = draw_roi(im0_rgb_with_roi, pts1)

            for i, det in enumerate(pred):
                if len(det):
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0_rgb.shape).round()
                    for *xyxy, conf, cls in reversed(det):
                        x_center = int((xyxy[0] + xyxy[2]) / 2)
                        y_center = int((xyxy[1] + xyxy[3]) / 2)
                        depth_value = depth[y_center, x_center]
                        if not np.isnan(depth_value):
                            depth_text = f"{depth_value:.2f} m"
                            cv2.putText(im0_rgb_with_roi, depth_text, (x_center, y_center - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                        plot_one_box(xyxy, im0_rgb_with_roi, line_thickness=3)

            show_seg_result(result_frame, (da_seg_mask_transformed, ll_seg_mask_transformed), is_demo=True)

            # Fit and draw polynomial lanes
            left_fit, right_fit = fit_polynomial(ll_seg_mask_transformed)
            result_frame = draw_polynomial_lines(result_frame, left_fit, right_fit, color=(255, 0, 0), thickness=3)

            # Create a copy for displaying polynomial equations
            poly_vis = result_frame.copy()
            if left_fit is not None:
                cv2.putText(poly_vis, f'f_left(y) = {left_fit[0]:.2e}y^2 + {left_fit[1]:.2e}y + {left_fit[2]:.2e}', 
                            (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            if right_fit is not None:
                cv2.putText(poly_vis, f'f_right(y) = {right_fit[0]:.2e}y^2 + {right_fit[1]:.2e}y + {right_fit[2]:.2e}', 
                            (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # Add FPS to the result frame
            cv2.putText(poly_vis, f'FPS: {fps_avg.avg:.2f}', (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Convert frames to BGR for display
            im0_bgr = cv2.cvtColor(im0_rgb_with_roi, cv2.COLOR_RGB2BGR)
            result_display = cv2.cvtColor(result_frame, cv2.COLOR_RGB2BGR)
            poly_vis_display = cv2.cvtColor(poly_vis, cv2.COLOR_RGB2BGR)

            # Display all windows
            cv2.imshow("Original ZED Stream with ROI", im0_bgr)
            # cv2.imshow("Transformed YOLOPv2 Stream - Bird's Eye View", result_display)
            # cv2.imshow("Polynomial Fit Equations", poly_vis_display)
            cv2.imshow("Transformed YOLOPv2 Stream - Bird's Eye View", poly_vis_display)

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