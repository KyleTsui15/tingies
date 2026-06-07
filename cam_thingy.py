#!/usr/bin/env python3

import cv2
import numpy as np
import tensorflow as tf

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray


class SSDMobileNetLiveNode(Node):
    def __init__(self):
        super().__init__("ssd_mobilenet_live_node")

        self.bridge = CvBridge()

        # Change these to your actual exported model + labels.
        self.model_path = "/home/ubuntu/ros2_ws/src/your_package/models/saved_model"
        self.detect_fn = tf.saved_model.load(self.model_path)

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

        #NOTE: Add subscriber to master node to get the class of block to find

        self.get_logger().info("Subscribed to /depth_cam/rgb/image_raw")

    def image_callback(self, msg):
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        input_tensor = tf.convert_to_tensor(
            np.expand_dims(rgb_image, axis=0),
            dtype=tf.uint8
        )

        detections = self.detect_fn(input_tensor)

        num = int(detections.pop("num_detections"))
        detections = {
            key: value[0, :num].numpy()
            for key, value in detections.items()
        }

        boxes = detections["detection_boxes"]
        scores = detections["detection_scores"]
        classes = detections["detection_classes"].astype(np.int32)

        box_corners = self.get_best_box_corners(rgb_image, boxes, scores, classes, threshold=0.5)

        if box_corners is not None:
            msg_out = Int32MultiArray()
            msg_out.data = box_corners
            self.box_pub.publish(msg_out)

            self.get_logger().info(f"Published box corners: {box_corners}")

    #NOTE:CHANGE LATER, ONLY ACCEPTS ONE OBJECT, ADD ARGUMENT FOR CLASS TO FIND BLOCKS OF SPECIFIC TYPE
    
    def get_best_box_corners(self, image, boxes, scores, classes, threshold=0.5):
        h, w = image.shape[:2]

        best_score = 0.0
        best_box = None

        for box, score, _cls in zip(boxes, scores, classes): #NOTE:_cls is currently throwaway, but will be used later to filter for specific block types
            if score < threshold:
                continue

            if score > best_score:
                best_score = score
                best_box = box

        if best_box is None:
            return None

        ymin, xmin, ymax, xmax = best_box

        x1 = int(xmin * w)
        y1 = int(ymin * h)
        x2 = int(xmax * w)
        y2 = int(ymax * h)

        top_left = (x1, y1)
        top_right = (x2, y1)
        bottom_right = (x2, y2)
        bottom_left = (x1, y2)

        return [
            top_left[0], top_left[1],
            top_right[0], top_right[1],
            bottom_right[0], bottom_right[1],
            bottom_left[0], bottom_left[1],
        ]

def main():
    rclpy.init()
    node = SSDMobileNetLiveNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()