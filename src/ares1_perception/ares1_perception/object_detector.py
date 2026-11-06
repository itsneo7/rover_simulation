"""object_detector node.

Publishes vision_msgs/Detection2DArray on /perception/detections from the
D456 RGB stream.

IMPORTANT — read before using in any non-simulation context:

This file defines TWO detector backends behind a common interface,
DetectorBackend. They are NOT interchangeable in capability and must not be
confused:

  1. SimulatedColorBlobDetector (used by default in this simulation)
     A lightweight, honest OpenCV color-thresholding blob detector tuned to
     find the orange traffic-cone-style obstacles placed in igvc_arena.sdf.
     This is a REAL, RUNNING detector — it is not a fake stand-in that
     invents detections — but it is NOT YOLO and NOT a learned object
     detector. It exists so the perception pipeline (detection -> depth
     association -> 3D obstacle -> costmap) can be exercised end-to-end in
     simulation without requiring GPU inference or a trained model.

  2. RealYoloDetectorBackend
     A documented, NOT-implemented integration point for a real YOLOv11
     model. It intentionally raises NotImplementedError until a person
     wires in actual model loading + inference (e.g. via ultralytics'
     YOLO class or a TensorRT/ONNXRuntime engine). Selecting this backend
     via the `detector_backend:=yolo` parameter without implementing it
     will fail loudly rather than silently behaving like the simulated
     detector — this is intentional, to prevent ever mistaking one for
     the other.
"""
import abc

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


class DetectorBackend(abc.ABC):
    """Common interface every detector backend must implement."""

    @abc.abstractmethod
    def detect(self, bgr_image: np.ndarray):
        """Return a list of (class_id: str, score: float, cx, cy, w, h) in pixels."""
        raise NotImplementedError


class SimulatedColorBlobDetector(DetectorBackend):
    """Real, working detector for simulation: HSV color thresholding for the
    orange cone/barricade obstacles in igvc_arena.sdf. Not a learned model."""

    def __init__(self, min_area_px: int = 80):
        self.min_area_px = min_area_px
        # Orange in HSV (OpenCV range: H 0-179, S/V 0-255)
        self.lower_orange = np.array([5, 120, 120])
        self.upper_orange = np.array([25, 255, 255])

    def detect(self, bgr_image: np.ndarray):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area_px:
                continue
            x, y, w, h = cv2.boundingRect(c)
            cx = x + w / 2.0
            cy = y + h / 2.0
            score = float(min(1.0, area / 2000.0))
            detections.append(('cone_or_barricade', score, cx, cy, float(w), float(h)))
        return detections


class RealYoloDetectorBackend(DetectorBackend):
    """Integration point for a real YOLOv11 model.

    NOT implemented here. To wire in a real model:
      1. `pip install ultralytics` (or bring your own ONNX/TensorRT runtime).
      2. Load your trained weights in __init__.
      3. Implement detect() to run inference and return the same
         (class_id, score, cx, cy, w, h) tuple format as
         SimulatedColorBlobDetector, so downstream nodes need no changes.
    """

    def __init__(self, weights_path: str):
        self.weights_path = weights_path
        raise NotImplementedError(
            'RealYoloDetectorBackend is an integration stub. Implement model '
            'loading and inference before selecting detector_backend:=yolo.'
        )

    def detect(self, bgr_image: np.ndarray):
        raise NotImplementedError


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('image_topic', '/realsense_D456/color/image_raw')
        self.declare_parameter('detector_backend', 'simulated')
        self.declare_parameter('yolo_weights_path', '')

        backend_name = self.get_parameter('detector_backend').value
        if backend_name == 'simulated':
            self.backend: DetectorBackend = SimulatedColorBlobDetector()
        elif backend_name == 'yolo':
            self.backend = RealYoloDetectorBackend(
                self.get_parameter('yolo_weights_path').value)
        else:
            raise ValueError(f'Unknown detector_backend "{backend_name}"')

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)

        self.image_sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._image_callback, qos)
        self.detections_pub = self.create_publisher(
            Detection2DArray, '/perception/detections', qos)

        self.get_logger().info(f'object_detector using backend="{backend_name}"')

    def _image_callback(self, msg: Image) -> None:
        bgr = self._image_msg_to_bgr(msg)
        if bgr is None:
            return

        raw_detections = self.backend.detect(bgr)

        out = Detection2DArray()
        out.header = msg.header
        for class_id, score, cx, cy, w, h in raw_detections:
            det = Detection2D()
            det.header = msg.header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_id
            hyp.hypothesis.score = score
            det.results.append(hyp)
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h
            out.detections.append(det)

        self.detections_pub.publish(out)

    @staticmethod
    def _image_msg_to_bgr(msg: Image):
        if msg.encoding not in ('rgb8', 'bgr8'):
            return None
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'rgb8':
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
