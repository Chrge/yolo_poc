import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from vision_msgs.msg import (
    Detection2D, Detection2DArray,
    ObjectHypothesisWithPose, BoundingBox2D
)
from cv_bridge import CvBridge
from ultralytics import YOLO


def sensor_qos():
    """SENSOR_DATA: BestEffort + kleine Tiefe"""
    try:
        from rclpy.qos import QoSPresetProfiles
        return QoSPresetProfiles.SENSOR_DATA.value
    except Exception:
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        return qos


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # ---- Parameter ----
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf', 0.25)
        self.declare_parameter('device', 'cpu')          # 'cpu' oder 'cuda:0'
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('log_every_n', 30)

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        model_path       = self.get_parameter('model_path').get_parameter_value().string_value
        self.conf        = float(self.get_parameter('conf').get_parameter_value().double_value)
        self.device      = self.get_parameter('device').get_parameter_value().string_value
        self.publish_annotated = bool(self.get_parameter('publish_annotated').get_parameter_value().bool_value)
        self.log_every_n = int(self.get_parameter('log_every_n').get_parameter_value().integer_value)

        self.get_logger().info(f'NumPy {np.__version__}, OpenCV {cv2.__version__}')
        if int(np.__version__.split('.')[0]) >= 2:
            self.get_logger().warn(
                "NumPy >= 2 erkannt. Falls cv_bridge-Fehler wie '_ARRAY_API not found' auftreten, "
                "downgrade mit: pip install 'numpy<2' --user"
            )

        # ---- YOLO ----
        self.get_logger().info(f'Loading YOLO model {model_path} on {self.device}')
        self.model = YOLO(model_path)

        # ---- Bridge ----
        self.bridge = CvBridge()

        # ---- I/O ----
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, sensor_qos())
        self.pub_det   = self.create_publisher(Detection2DArray, 'detections', 10)
        self.pub_annot = self.create_publisher(Image, 'yolo/annotated', 10)
        self.get_logger().info(f"Subscribed to {self.image_topic}")

        # ---- Ermitteln, wie BoundingBox2D.center gesetzt werden darf ----
        self._center_mode = self._detect_center_set_mode()

        # ---- Status ----
        self.frame_count = 0
        self.first_stamp = None

    # ----------------------------------------------
    # Detect best way to set BoundingBox2D.center
    # ----------------------------------------------
    def _detect_center_set_mode(self):
        tmp = BoundingBox2D()

        # 1) Felder direkt mutieren
        try:
            tmp.center.x = 0.0
            tmp.center.y = 0.0
            tmp.center.theta = 0.0
            self.get_logger().info("BBox center mode: mutate_fields (center.x/y/theta)")
            return "mutate_fields"
        except Exception as e:
            self.get_logger().warn(f"Mutate fields failed: {e}")

        # 2) Pose2D-Objekt zuweisen
        try:
            tmp2 = BoundingBox2D()
            tmp2.center = Pose2D(x=0.0, y=0.0, theta=0.0)
            self.get_logger().info("BBox center mode: assign_pose (center = Pose2D(...))")
            return "assign_pose"
        except Exception as e:
            self.get_logger().warn(f"Assign Pose2D failed: {e}")

        # 3) Nur size setzen, center auf 0/0 lassen
        self.get_logger().error(
            "Konnte BoundingBox2D.center nicht setzen (weder Felder noch Pose2D-Zuweisung). "
            "Ich publiziere Detections ohne Center (size_x/size_y gesetzt)."
        )
        return "none"

    # ----------------------------------------------
    # FunctionHandle: BoundingBox robust erstellen
    # ----------------------------------------------
    def _make_bbox(self, cx: float, cy: float, w: float, h: float):
        bbox = BoundingBox2D()
        if self._center_mode == "mutate_fields":
            try:
                bbox.center.x = float(cx)
                bbox.center.y = float(cy)
                bbox.center.theta = 0.0
            except Exception as e:
                self.get_logger().error(f"mutate_fields failed at runtime: {e}")
                return None
        elif self._center_mode == "assign_pose":
            try:
                bbox.center = Pose2D(x=float(cx), y=float(cy), theta=0.0)
            except Exception as e:
                self.get_logger().error(f"assign_pose failed at runtime: {e}")
                return None
        else:
            # center nicht setzbar
            pass

        bbox.size_x = float(max(0.0, w))
        bbox.size_y = float(max(0.0, h))
        return bbox

    # ----------------------------------------------
    # Image-Callback
    # ----------------------------------------------
    def on_image(self, msg: Image):
        self.frame_count += 1
        if self.first_stamp is None:
            self.first_stamp = msg.header.stamp
            self.get_logger().info(
                f"First image: {msg.width}x{msg.height} enc='{msg.encoding}', "
                f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
            )

        # ROS -> OpenCV
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge imgmsg_to_cv2 error: {e}')
            return

        # YOLO Inferenz
        try:
            results = self.model.predict(cv_img, verbose=False, conf=self.conf, device=self.device)
        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')
            return

        det_array = Detection2DArray()
        det_array.header = msg.header
        annotated = cv_img

        num_boxes = 0
        if results:
            r = results[0]
            boxes = getattr(r, "boxes", None)
            if boxes is not None and getattr(boxes, "xyxy", None) is not None:
                n = len(boxes)
                for i in range(n):
                    xyxy = boxes.xyxy[i].tolist()
                    x1, y1, x2, y2 = map(float, xyxy)
                    w = x2 - x1
                    h = y2 - y1
                    cx = x1 + w * 0.5
                    cy = y1 + h * 0.5

                    conf = float(boxes.conf[i].item()) if getattr(boxes, "conf", None) is not None else 0.0
                    cls_id = int(boxes.cls[i].item()) if getattr(boxes, "cls", None) is not None else -1

                    names = getattr(self.model, "names", {})
                    if isinstance(names, dict):
                        label = names.get(cls_id, str(cls_id))
                    elif isinstance(names, (list, tuple)) and 0 <= cls_id < len(names):
                        label = names[cls_id]
                    else:
                        label = str(cls_id)

                    bbox = self._make_bbox(cx, cy, w, h)
                    if bbox is None:
                        # nicht publizieren, wenn center partout nicht setzbar
                        continue

                    det = Detection2D()
                    det.bbox = bbox

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = label
                    hyp.hypothesis.score = conf
                    det.results.append(hyp)
                    det_array.detections.append(det)

                    # Zeichnen
                    p1 = (int(round(x1)), int(round(y1)))
                    p2 = (int(round(x2)), int(round(y2)))
                    try:
                        cv2.rectangle(annotated, p1, p2, (0, 255, 0), 2)
                        cv2.putText(annotated, f'{label} {conf:.2f}', (p1[0], max(0, p1[1] - 5)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                    except Exception as draw_err:
                        self.get_logger().warn(f"Drawing failed: {draw_err}")
                    num_boxes += 1

        # Publish
        self.pub_det.publish(det_array)
        if self.publish_annotated:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                img_msg.header = msg.header
                self.pub_annot.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'cv_bridge cv2_to_imgmsg error: {e}')

        # Throttled log
        if self.frame_count % max(1, self.log_every_n) == 0:
            self.get_logger().info(
                f"Frame {self.frame_count}: published {num_boxes} detections "
                f"({len(det_array.detections)} in array)."
            )


def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
