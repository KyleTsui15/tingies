
import cv2
import numpy as np

from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class YOLOSegAngleNode(Node):
    def __init__(self):
        super().__init__("yolo_seg_angle_node")

        self.bridge = CvBridge()

        self.model_path = "/home/hiwonder/ros_ws/src/yolo_seg_live/models/e.onnx"
        self.model = YOLO(self.model_path)
        
        self.min_conf = 0.5

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image,
            "/depth_cam/rgb/image_raw",
            self.image_callback,
            sensor_qos,
        )

        #Master node object type

        self.angle_pub = self.create_publisher(
            Float32,
            "/yolo_seg/object_angle",
            10,
        )

        self.get_logger().info("YOLO-seg angle node started.")

    def image_callback(self, msg):
        image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        results = self.model.predict(
            source=image_bgr,
            conf=self.min_conf,
            verbose=False,
            retina_masks=True,
        )

        result = results[0]

        if result.boxes is None or len(result.boxes) == 0:
            return

        if result.masks is None or result.masks.xy is None:
            return

        scores = result.boxes.conf.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)

        best_index = int(np.argmax(scores))

        polygon = result.masks.xy[best_index]

        if polygon is None or len(polygon) < 4:
            return

        contour = polygon.astype(np.float32)

        rect = cv2.minAreaRect(contour)
        (cx, cy), (w, h), angle = rect

        if w < h:
            angle += 90.0

        if angle > 90:
            angle -= 180
        elif angle < -90:
            angle += 180

        angle_msg = Float32()
        angle_msg.data = float(angle)
        self.angle_pub.publish(angle_msg)

        cls = int(classes[best_index])
        class_name = self.model.names.get(cls, str(cls))

        self.get_logger().info(
            f"{class_name}: angle={angle:.2f} deg, center=({cx:.1f}, {cy:.1f})"
        )


def main():
    rclpy.init()
    node = YOLOSegAngleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()