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

def detect():
    weights, imgsz = opt.weights, opt.img_size

    # Set up directories
    save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))
    save_dir.mkdir(parents=True, exist_ok=True)

    # Initialize ZED camera
    zed = ZEDCamera()
    zed.open()

    # Load model
    device = select_device(opt.device)
    model = torch.jit.load(weights).to(device)
    half = device.type != 'cpu'
    if half:
        model.half()
    model.eval()

    # Initialize FPS counter
    fps_avg = AverageMeter()

    print("Starting ZED camera stream. Press 'q' to quit.")
    try:
        while True:
            # Get frame from ZED
            im0 = zed.grab_frame()
            if im0 is None:
                print("Failed to grab frame. Exiting.")
                break

            # Preprocess frame
            # Convert BGRA to RGB
            im0 = cv2.cvtColor(im0, cv2.COLOR_BGRA2RGB)

            # Apply letterbox resizing
            img, ratio, (dw, dh) = letterbox(im0, new_shape=(imgsz, imgsz))
            
            # Prepare image for model
            img = img.transpose(2, 0, 1)  # HWC to CHW
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(device).float()
            img /= 255.0  # Normalize
            img = img.unsqueeze(0)  # Add batch dimension

            if half:
                img = img.half()

            # Inference
            t1 = time_synchronized()
            [pred, anchor_grid], seg, ll = model(img)
            t2 = time_synchronized()

            # Calculate and update FPS
            fps = 1 / (t2 - t1)
            fps_avg.update(fps)

            # Display FPS
            cv2.putText(im0, f'FPS: {fps_avg.avg:.2f}', (20, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Apply NMS
            pred = split_for_trace_model(pred, anchor_grid)
            pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, 
                                     classes=opt.classes, agnostic=opt.agnostic_nms)

            # Process detection results
            da_seg_mask = driving_area_mask(seg)
            ll_seg_mask = lane_line_mask(ll)

            for det in pred:
                if len(det):
                    # Rescale boxes
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    for *xyxy, conf, cls in reversed(det):
                        plot_one_box(xyxy, im0, line_thickness=3)

            # Resize segmentation masks to match im0's dimensions
            da_seg_mask = cv2.resize(da_seg_mask, (im0.shape[1], im0.shape[0]), 
                                   interpolation=cv2.INTER_NEAREST)
            ll_seg_mask = cv2.resize(ll_seg_mask, (im0.shape[1], im0.shape[0]), 
                                   interpolation=cv2.INTER_NEAREST)

            # Show segmentation results
            show_seg_result(im0, (da_seg_mask, ll_seg_mask), is_demo=True)

            # Display results
            cv2.imshow("YOLOPv2 ZED Stream", cv2.cvtColor(im0, cv2.COLOR_BGR2RGB))

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
