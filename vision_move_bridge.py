#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from kinematics_msgs.srv import SetRobotPose
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position


class VisionArmMover(Node):
    def __init__(self):
        super().__init__('vision_arm_mover')

        self.busy = False

        self.ik_client = self.create_client(
            SetRobotPose,
            '/kinematics/set_pose_target'
        )

        self.joints_pub = self.create_publisher(
            ServosPosition,
            '/servo_controller',
            1
        )

        self.target_sub = self.create_subscription(
            Pose,
            '/target_pose',
            self.target_pose_callback,
            10
        )

        self.get_logger().info('Waiting for /kinematics/set_pose_target...')
        self.ik_client.wait_for_service()
        self.get_logger().info('vision_arm_mover ready')

    def target_pose_callback(self, pose_msg):
        if self.busy:
            return

        self.busy = True

        req = SetRobotPose.Request()
        req.position = [
            float(pose_msg.position.x),
            float(pose_msg.position.y),
            float(pose_msg.position.z)
        ]

        # Gripper pitch in degrees.
        # Adjust this for your actual picking pose.
        req.pitch = -90.0

        # Allow IK to search around the target pitch.
        req.pitch_range = [-120.0, -60.0]

        # Smaller = more IK search detail, but slower.
        req.resolution = 1.0

        # Not used by set_pose_target, but harmless.
        req.duration = 1.0

        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_done_callback)

    def ik_done_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'IK service failed: {e}')
            self.busy = False
            return

        if not res.success or len(res.pulse) < 5:
            self.get_logger().warn('No valid IK solution')
            self.busy = False
            return

        servo_data = res.pulse

        self.get_logger().info(f'Moving to pulses: {servo_data}')

        set_servo_position(
            self.joints_pub,
            0.5,
            (
                (10, 500),                 # gripper or auxiliary servo
                (5, 500),                  # wrist rotate / neutral
                (4, int(servo_data[3])),
                (3, int(servo_data[2])),
                (2, int(servo_data[1])),
                (1, int(servo_data[0])),
            )
        )

        time.sleep(0.5)
        self.busy = False


def main():
    rclpy.init()
    node = VisionArmMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()