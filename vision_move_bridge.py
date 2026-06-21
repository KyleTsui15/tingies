#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
import kinematics.transform as transform
from kinematics.inverse_kinematics import get_ik
from servo_controller_msgs.msg import ServoPosition, ServosPosition


class IkDemo(Node):
    def __init__(self):
        super().__init__('ik_demo_node')

        #-----------#>
        self.coordinate = None
        self.offset = [0.07, 0.13, 0.0]
        #-----------#>

        self.servo_list = []
        self.duration = 1.0

        #-----------#>
        self.coords_sub = self.create_subscription(Float32MultiArray, '/yolo_seg/object_pos', self.coordinate_callback, 10)
        self.move_srv = self.create_service(Trigger, '/move_to_object', self.move_callback)
        #-----------#>


        self.servos_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)

        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()

        self.get_logger().info('Waiting for object position and move command')

    def coordinate_callback(self, msg):
        if len(msg.data) >= 3:
            self.coordinate = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]

    def move_callback(self, request, response):
        if self.coordinate is None:
            response.success = False
            response.message = 'No object position received'
            return response

        target = [self.coordinate[i] + self.offset[i] for i in range(3)]
        self.get_logger().info(f'Object: {self.coordinate}, target: {target}')


        res = get_ik(target, 0, [-180, 180])
        if res != []:
            pulse = transform.angle2pulse(res[0][0])
            self.servo_list = pulse[0]
            self.set_servo_position(self.servos_pub, self.duration, self.servo_list)
            response.success = True
            response.message = f'Moving to {target}'
        else:
            response.success = False
            response.message = f'No IK solution for {target}'

        return response

    def set_servo_position(self, pub, duration, positions):
        msg = ServosPosition()
        msg.duration = float(duration)
        position_list = []
        for i in range(1, 6):
            position = ServoPosition()
            position.id = i
            position.position = float(positions[i - 1])
            position_list.append(position)
        msg.position = position_list
        msg.position_unit = 'pulse'
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ik_demo_node = IkDemo()
    rclpy.spin(ik_demo_node)
    ik_demo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()