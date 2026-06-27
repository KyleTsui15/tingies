
import cv2
import numpy as np

from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
# === BEGIN MASTER-INTEGRATION CHANGES (imports) ===
from std_msgs.msg import Float32, String, Float32MultiArray
# === END MASTER-INTEGRATION CHANGES (imports) ===
from cv_bridge import CvBridge


class YOLOSegAngleNode(Node):
    def __init__(self):
        super().__init__("yolo_seg_angle_node")

        self.bridge = CvBridge()

        self.model_path = "/home/hiwonder/ros_ws/src/yolo_seg_live/models/e.pt"
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

        # === BEGIN MASTER-INTEGRATION CHANGES (per-arm topics + class filter) ===
        # One node instance per arm. arm_num namespaces all topics so the master
        # and vision_move_bridge can address this arm specifically. Run a second
        # instance with arm_num:=2 for a second arm.
        self.declare_parameter("arm_num", 1)
        self.arm_num = self.get_parameter("arm_num").value
        ns = f"/arm_{self.arm_num}"

        # Only report objects matching the class the master currently wants.
        self.target_class = None
        self.block_class_sub = self.create_subscription(
            String, f"{ns}/block_class", self.block_class_callback, 10,
        )

        # Detected object position (x, y, z, angle) in robot/world frame.
        self.pos_pub = self.create_publisher(Float32MultiArray, f"{ns}/yolo_seg/object_info", 10)

        #====LEGACY====#
        #self.angle_pub = self.create_publisher(Float32, f"{ns}/yolo_seg/object_angle", 10)
        #====LEGACY====#

        self.get_logger().info(f"YOLO-seg node started for arm_{self.arm_num}.")
        # === END MASTER-INTEGRATION CHANGES (per-arm topics + class filter) ===

    # === BEGIN MASTER-INTEGRATION CHANGES (block_class + pixel->world) ===
    def block_class_callback(self, msg):
        """Master tells us which block type to look for."""
        self.target_class = msg.data if msg.data else None
        self.get_logger().info(f"arm_{self.arm_num}: now looking for '{self.target_class}'")

    def pixel_to_world(self, cx, cy):
        """Convert an image-space center (px) to a robot-frame (x, y, z).

        STUB: replace with a real calibration / depth lookup. The values below
        are a placeholder linear mapping so the pipeline produces a usable pose.
        """
        # TODO: use depth image + camera intrinsics/extrinsics for real coords.
        x = 0.10 + (cy / 480.0) * 0.10
        y = (cx / 640.0 - 0.5) * 0.20
        z = 0.02
        return [float(x), float(y), float(z)]
    # === END MASTER-INTEGRATION CHANGES (block_class + pixel->world) ===

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

        # === BEGIN MASTER-INTEGRATION CHANGES (filter to requested class) ===
        # Pick the most confident detection of the class the master asked for.
        # If no target class is set yet, fall back to the overall best.
        candidate_indices = list(range(len(scores)))
        if self.target_class is not None:
            candidate_indices = [
                i for i in candidate_indices
                if self.model.names.get(int(classes[i]), str(int(classes[i]))) == self.target_class
            ]
            if not candidate_indices:
                return  # requested block type not visible this frame

        best_index = max(candidate_indices, key=lambda i: scores[i])
        # === END MASTER-INTEGRATION CHANGES (filter to requested class) ===

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

        #====LEGACY====#
        # angle_msg = Float32()
        # angle_msg.data = float(angle)
        # self.angle_pub.publish(angle_msg)
        #====LEGACY====#

        # === BEGIN MASTER-INTEGRATION CHANGES (publish object position + angle) ===
        pos_msg = Float32MultiArray()

        pos = self.pixel_to_world(cx, cy)   # expected: [x, y, z] or similar
        pos_msg.data = [float(v) for v in pos] + [float(angle)]

        self.pos_pub.publish(pos_msg)
        # === END MASTER-INTEGRATION CHANGES (publish object position) ===

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