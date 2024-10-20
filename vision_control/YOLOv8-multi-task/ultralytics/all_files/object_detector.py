from ultralytics import YOLO
import torch

class ObjectDetector:
    def __init__(self, model_path, conf_threshold):
        """
        Initialize the YOLO object detector.
        
        :param model_path: Path to the YOLO model file
        :param conf_threshold: Confidence threshold for detections
        """
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.device = torch.device(0 if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")

    def detect(self, frame, iou=0.45):
        """
        Perform object detection on a frame.
        
        :param frame: Input frame for detection
        :param iou: IOU threshold for non-max suppression
        :return: Detection results
        """
        return self.model.predict(
            frame, 
            conf=self.conf_threshold, 
            iou=iou, 
            device=self.device, 
            imgsz=(384,672), 
            show_labels=False, 
            save=False, 
            stream=True
        )

    def get_model_info(self):
        """Get information about the loaded model."""
        return {
            "model_name": self.model.name,
            "model_type": self.model.type,
            "task": self.model.task,
            "num_classes": len(self.model.names)
        }

    def get_class_names(self):
        """Get the names of classes the model can detect."""
        return self.model.names

    def set_conf_threshold(self, new_threshold):
        """Set a new confidence threshold."""
        if 0 <= new_threshold <= 1:
            self.conf_threshold = new_threshold
        else:
            print("Invalid confidence threshold. It should be between 0 and 1.")

    def warmup(self):
        """Perform a warmup inference to initialize the model."""
        dummy_input = torch.zeros((1, 3, 384, 672)).to(self.device)
        self.model(dummy_input)
        print("Model warmup completed.")