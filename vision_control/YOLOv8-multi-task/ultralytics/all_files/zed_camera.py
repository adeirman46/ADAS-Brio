import pyzed.sl as sl

class ZEDCamera:
    def __init__(self):
        """Initialize the ZED camera with default parameters."""
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.camera_fps = 30
        self.init_params.coordinate_units = sl.UNIT.METER
        self.image = sl.Mat()
        self.depth = sl.Mat()
        self.objects = sl.Objects()

    def open(self):
        """Open the ZED camera."""
        if self.zed.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            print("Failed to open ZED camera")
            exit(1)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1)

    def enable_object_detection(self):
        """Enable object detection on the ZED camera."""
        obj_param = sl.ObjectDetectionParameters()
        obj_param.enable_tracking = True
        if not self.zed.enable_object_detection(obj_param):
            print("Failed to enable object detection")
            exit(1)

    def grab_frame(self):
        """Grab a frame from the ZED camera."""
        if self.zed.grab() == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
            self.zed.retrieve_measure(self.depth, sl.MEASURE.DEPTH)
            return self.image.get_data(), self.depth
        return None, None

    def retrieve_objects(self):
        """Retrieve detected objects from the ZED camera."""
        self.zed.retrieve_objects(self.objects)

    def get_object_list(self):
        """Get the list of detected objects."""
        return self.objects.object_list

    def get_camera_info(self):
        """Get camera information."""
        camera_info = self.zed.get_camera_information()
        return {
            "serial_number": camera_info.serial_number,
            "firmware_version": camera_info.firmware_version,
            "resolution": camera_info.camera_resolution,
            "fps": camera_info.camera_fps
        }

    def set_detection_parameters(self, confidence_threshold=50, instance_segmentation_confidence_threshold=50):
        """Set object detection parameters."""
        detection_params = sl.ObjectDetectionParameters()
        detection_params.detection_confidence_threshold = confidence_threshold
        detection_params.instance_segmentation_confidence_threshold = instance_segmentation_confidence_threshold
        self.zed.update_object_detection_parameters(detection_params)

    def close(self):
        """Close the ZED camera."""
        self.zed.close()