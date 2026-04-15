from __future__ import annotations

from pathlib import Path
from typing import List

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image

from rf_msgs.msg import RfDetection, RfDetectionArray, StftFrame


class YoloDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector_node")

        self.declare_parameter("model_path", str(_default_model_path()))
        self.declare_parameter("conf_thresh", 0.2)
        self.declare_parameter("input_topic", "/stft_node/stft")
        self.declare_parameter("output_topic", "~/detections")
        self.declare_parameter("image_topic", "~/image")

        self._model_path = str(self.get_parameter("model_path").value)
        self._conf_thresh = float(self.get_parameter("conf_thresh").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)

        self._labels = ["Chirp", "Wideband", "Narrowband", "FHSS"]
        self._model = self._load_model(self._model_path)

        self._pub = self.create_publisher(RfDetectionArray, output_topic, 10)
        self._image_pub = self.create_publisher(Image, image_topic, 10)
        self._sub = self.create_subscription(StftFrame, input_topic, self._on_stft, 10)

        self.get_logger().info(
            f"YoloDetectorNode ready: model={self._model_path} conf={self._conf_thresh:.2f} "
            f"input={input_topic} output={output_topic} image={image_topic}"
        )

    def _load_model(self, model_path: str):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is required for YoloDetectorNode. "
                "Install it in your ROS environment before running this node."
            ) from exc

        return YOLO(model_path)

    def _on_stft(self, msg: StftFrame) -> None:
        detections_msg = RfDetectionArray()
        detections_msg.stamp = msg.stamp

        if msg.num_frames == 0 or msg.fft_size == 0:
            self._pub.publish(detections_msg)
            return

        try:
            sxx_db = np.asarray(msg.waterfall_db, dtype=np.float64).reshape(
                int(msg.num_frames), int(msg.fft_size)
            ).T
            f_ghz = np.asarray(msg.freq_ghz, dtype=np.float64)
            t_s = np.asarray(msg.time_s, dtype=np.float64)
        except ValueError:
            self.get_logger().warning("Received malformed STFT frame; skipping")
            self._pub.publish(detections_msg)
            return

        if sxx_db.size == 0 or f_ghz.size < 2 or t_s.size < 2:
            self._pub.publish(detections_msg)
            return

        img_rgb = self._stft_to_image(sxx_db)
        self._image_pub.publish(self._image_msg_from_array(msg, img_rgb))
        results = self._model.predict(img_rgb, conf=self._conf_thresh, verbose=False)
        if not results:
            self._pub.publish(detections_msg)
            return

        result = results[0]
        if result.boxes is None or len(result.boxes) == 0:
            self._pub.publish(detections_msg)
            return

        detections_msg.detections = self._boxes_to_detections(result, f_ghz, t_s)
        self._pub.publish(detections_msg)

    def _boxes_to_detections(self, result, f_ghz: np.ndarray, t_s: np.ndarray) -> List[RfDetection]:
        f_min = float(f_ghz[0])
        f_max = float(f_ghz[-1])
        t_min = float(t_s[0])
        t_max = float(t_s[-1])
        f_span = f_max - f_min if f_max != f_min else 1.0
        t_span = t_max - t_min if t_max != t_min else 1.0

        boxes = result.boxes
        xyxy_all = boxes.xyxy.cpu().numpy()
        conf_all = boxes.conf.cpu().numpy()
        cls_all = boxes.cls.cpu().numpy().astype(int)

        detections: List[RfDetection] = []
        for (x1, y1, x2, y2), conf, cls_idx in zip(xyxy_all, conf_all, cls_all):
            f_lo = f_min + (float(x1) / 640.0) * f_span
            f_hi = f_min + (float(x2) / 640.0) * f_span
            t_lo = t_min + (float(y1) / 640.0) * t_span
            t_hi = t_min + (float(y2) / 640.0) * t_span

            det = RfDetection()
            det.kind = self._labels[int(cls_idx)] if int(cls_idx) < len(self._labels) else str(int(cls_idx))
            det.confidence = float(conf)
            det.f_lo_ghz = f_lo
            det.t_lo_s = t_lo
            det.f_hi_ghz = f_hi
            det.t_hi_s = t_hi
            det.f_center_ghz = (f_lo + f_hi) * 0.5
            det.bw_ghz = abs(f_hi - f_lo)
            detections.append(det)

        return detections

    def _stft_to_image(self, sxx_db: np.ndarray) -> np.ndarray:
        try:
            from scipy import ndimage
        except ImportError as exc:
            raise RuntimeError(
                "scipy is required for YoloDetectorNode image preprocessing."
            ) from exc

        lo = float(np.percentile(sxx_db, 1.0))
        hi = float(np.percentile(sxx_db, 99.5))
        span = hi - lo if hi > lo else 1.0
        s_norm = np.clip((sxx_db - lo) / span, 0.0, 1.0)
        s_tf = s_norm.T

        zy = 640.0 / s_tf.shape[0]
        zx = 640.0 / s_tf.shape[1]
        img = ndimage.zoom(s_tf, zoom=(zy, zx), order=1)
        img_u8 = (255.0 * np.clip(img, 0.0, 1.0)).astype(np.uint8)
        return np.stack([img_u8, img_u8, img_u8], axis=-1)

    def _image_msg_from_array(self, stft_msg: StftFrame, img_rgb: np.ndarray) -> Image:
        image_msg = Image()
        image_msg.header.stamp = stft_msg.stamp
        image_msg.header.frame_id = "stft"
        image_msg.height = int(img_rgb.shape[0])
        image_msg.width = int(img_rgb.shape[1])
        image_msg.encoding = "rgb8"
        image_msg.is_bigendian = False
        image_msg.step = int(img_rgb.shape[1] * img_rgb.shape[2])
        image_msg.data = img_rgb.tobytes()
        return image_msg


def _default_model_path() -> Path:
    return Path(get_package_share_directory("rf_dectectors")) / "resource" / "best.pt"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
