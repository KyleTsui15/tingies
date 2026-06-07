#!/usr/bin/env python3

import cv2
import numpy as np

from tensorflow.lite.python.interpreter import Interpreter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge


class TFLiteDetectorNode(Node):
    def __init__(self):
        super().__init__("tflite_detector_node")

        self.bridge = CvBridge()

        self.model_path = "/home/hiwonder/ros_ws/src/ssd_mobilenet_live/models/model.tflite"
        self.label_path = "/home/hiwonder/ros_ws/src/ssd_mobilenet_live/models/labelmap.txt"
        self.min_conf = 0.5

        with open(self.label_path, "r") as f:
            self.labels = [line.strip() for line in f.readlines()]

        self.interpreter = Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.model_height = self.input_details[0]["shape"][1]
        self.model_width = self.input_details[0]["shape"][2]
        self.float_input = self.input_details[0]["dtype"] == np.float32

        self.input_mean = 127.5
        self.input_std = 127.5

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

        self.box_pub = self.create_publisher(
            Int32MultiArray,
            "/ssd_mobilenet/box_corners",
            10,
        )

        #NOTE Add subcription to master node for class retrieval 

        self.get_logger().info("TFLite detector node started.")

    def image_callback(self, msg):
        # ROS Image -> OpenCV RGB image
        image_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        imH, imW, _ = image_rgb.shape

        # Resize to model input size
        image_resized = cv2.resize(image_rgb, (self.model_width, self.model_height))
        input_data = np.expand_dims(image_resized, axis=0)

        # Normalize only if the model expects float32
        if self.float_input:
            input_data = (np.float32(input_data) - self.input_mean) / self.input_std

        # Run TFLite inference
        self.interpreter.set_tensor(self.input_details[0]["index"], input_data)
        self.interpreter.invoke()

        # These indices match your original program
        scores = self.interpreter.get_tensor(self.output_details[0]["index"])[0]
        boxes = self.interpreter.get_tensor(self.output_details[1]["index"])[0]
        classes = self.interpreter.get_tensor(self.output_details[3]["index"])[0]

        best_box = None
        best_score = 0.0
        best_class = None

        for i in range(len(scores)):
            score = scores[i]

            if score > self.min_conf and score <= 1.0:
                if score > best_score:
                    best_score = score
                    best_box = boxes[i]
                    best_class = int(classes[i])

        if best_box is None:
            return

        ymin = int(max(1, best_box[0] * imH))
        xmin = int(max(1, best_box[1] * imW))
        ymax = int(min(imH, best_box[2] * imH))
        xmax = int(min(imW, best_box[3] * imW))

        box_corners = [
            xmin, ymin,   # top-left
            xmax, ymin,   # top-right
            xmax, ymax,   # bottom-right
            xmin, ymax,   # bottom-left
        ]

        msg_out = Int32MultiArray()
        msg_out.data = box_corners
        self.box_pub.publish(msg_out)

        label = self.labels[best_class] if best_class < len(self.labels) else str(best_class)

        self.get_logger().info(
            f"Published {label} {best_score:.2f}: {box_corners}"
        )


def main():
    rclpy.init()
    node = TFLiteDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()