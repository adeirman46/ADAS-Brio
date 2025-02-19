import argparse
import time
from pathlib import Path
import cv2
import torch

# Conclude setting / general reprocessing / plots / metrics / datasets
from utils.utils import (
    time_synchronized, select_device, increment_path,
    scale_coords, xyxy2xywh, non_max_suppression, split_for_trace_model,
    driving_area_mask, lane_line_mask, plot_one_box, show_seg_result,
    AverageMeter
)

def make_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='data/weights/yolopv2.pt', help='model.pt path')
    parser.add_argument('--source', type=int, default=0, help='camera source (default=0)')
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
    source, weights, imgsz = opt.source, opt.weights, opt.img_size

    # Set up directories
    save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))
    save_dir.mkdir(parents=True, exist_ok=True)

    # Load model
    stride = 32
    device = select_device(opt.device)
    model = torch.jit.load(weights).to(device)
    half = device.type != 'cpu'
    if half:
        model.half()
    model.eval()

    # Camera stream
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        print(f"Error: Unable to open camera source {source}")
        return

    print("Starting camera stream. Press 'q' to quit.")
    while cap.isOpened():
        ret, im0 = cap.read()
        if not ret:
            print("Failed to read frame. Exiting.")
            break

        # Preprocess frame
        img = cv2.resize(im0, (imgsz, imgsz))
        img = torch.from_numpy(img).to(device).float()
        img /= 255.0  # Normalize
        img = img.permute(2, 0, 1).unsqueeze(0)  # HWC to CHW

        if half:
            img = img.half()

        # Inference
        t1 = time_synchronized()
        [pred, anchor_grid], seg, ll = model(img)
        t2 = time_synchronized()

        # Calculate FPS
        fps = 1 / (t2 - t1)

        # Display FPS
        cv2.putText(im0, f'FPS: {fps:.2f}', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)


        # Apply NMS
        pred = split_for_trace_model(pred, anchor_grid)
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)

        # Process results
        da_seg_mask = driving_area_mask(seg)
        ll_seg_mask = lane_line_mask(ll)

        for det in pred:
            if len(det):
                # Rescale boxes
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                for *xyxy, conf, cls in reversed(det):
                    plot_one_box(xyxy, im0, line_thickness=3)

        # Resize segmentation masks to match im0's dimensions
        da_seg_mask = cv2.resize(da_seg_mask, (im0.shape[1], im0.shape[0]), interpolation=cv2.INTER_NEAREST)
        ll_seg_mask = cv2.resize(ll_seg_mask, (im0.shape[1], im0.shape[0]), interpolation=cv2.INTER_NEAREST)

        # Pass the resized masks
        show_seg_result(im0, (da_seg_mask, ll_seg_mask), is_demo=True)


        # Display results
        cv2.imshow("YOLOPv2 Stream", im0)

        # Break loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Stream ended. Exiting.")

if __name__ == '__main__':
    opt = make_parser().parse_args()
    print(opt)
    with torch.no_grad():
        detect()
