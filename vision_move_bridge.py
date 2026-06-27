#!/usr/bin/env python3
# encoding: utf-8
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger
import kinematics.transform as transform
from kinematics.inverse_kinematics import get_ik
from servo_controller_msgs.msg import ServoPosition, ServosPosition

# === BEGIN MASTER-INTEGRATION CHANGES (imports) ===
import time
from std_msgs.msg import Float32, String
# === END MASTER-INTEGRATION CHANGES (imports) ===


# Arm states published on /arm_{num}/arm_state
ST_IDLE = "idle"
ST_PICKING = "picking"
ST_STANDBY = "standby"
ST_PLACING = "placing"


class IkDemo(Node):
    def __init__(self):
        super().__init__('ik_demo_node')

        #-----------#
        self.coordinate = None
        self.offset = [0.07, 0.13, 0.0]
        #-----------#

        self.servo_list = []
        self.duration = 1.0

        # One bridge instance per arm; arm_num namespaces all topics.
        self.declare_parameter('arm_num', 1)
        self.arm_num = self.get_parameter('arm_num').value
        ns = f'/arm_{self.arm_num}'

        # Sequence parameters (from the MISC pseudocode).
        self.vertical_offset = 0.05
        # global standby + (optional) per-arm reservoir standby pose.
        self.global_standby = [0.0, -0.20, 0.20]
        self.standby_pos = list(self.global_standby)

        self.angle = 0.0          # latest detected object angle (deg)
        self.busy = False         # guard so we don't restart a pick mid-motion
        self.state = ST_IDLE

        # Vision inputs (from yolo_seg_node).
        self.coords_sub = self.create_subscription(Float32MultiArray, f'{ns}/yolo_seg/object_info', self.coordinate_callback, 10)
        
        # Placement target handed down by the master (x, y, z, angle).
        self.place_sub = self.create_subscription(Float32MultiArray, f'{ns}/place_target', self.place_callback, 10)

        # Arm state status output (master subscribes to this).
        self.state_pub = self.create_publisher(String, f'{ns}/arm_state', 10)

        #====LEGACY====#
        #self.move_srv = self.create_service(Trigger, f'{ns}/move_to_object', self.move_callback)
        #self.angle_sub = self.create_subscription(Float32, f'{ns}/yolo_seg/object_angle', self.angle_callback, 10)
        #====LEGACY====#

        self.servos_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)

        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()

        self.publish_state(ST_IDLE)
        self.get_logger().info(f'arm_{self.arm_num} bridge ready (auto-pick on detection).')

    #====LEGACY====#
    # def angle_callback(self, msg):
    #     self.angle = float(msg.data)
    #====LEGACY====#

    
    def publish_state(self, state):
        self.state = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def coordinate_callback(self, msg):
        if len(msg.data) >= 3:
            self.coordinate = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]
            self.angle = float(msg.data[3])
            # Immediately move to the object without waiting for a trigger,
            # but only when this arm is free and holding nothing.
            if not self.busy and self.state == ST_IDLE:
                self.pick_object()

    def move_to(self, coord, duration=None):
        """Solve IK for `coord` and command the servos. Returns True on success."""
        duration = self.duration if duration is None else duration
        res = get_ik(coord, 0, [-180, 180]) #CHANGE
        if res == []:
            self.get_logger().warn(f'No IK solution for {coord}')
            return False
        pulse = transform.angle2pulse(res[0][0])
        self.servo_list = pulse[0]
        self.set_servo_position(self.servos_pub, duration, self.servo_list)
        return True

    def wait_motion(self, duration=None):
        """Block until the commanded motion is (approximately) complete.

        NOTE: time.sleep here blocks the executor. For a foundation this is fine;
        a hardened version should use servo feedback / an action server instead.
        """
        time.sleep(self.duration if duration is None else duration)

    def spin_servo(self, servo_id, direction, turns):
        """Spin a continuous servo (e.g. the lead-screw gripper, id 10).

        STUB: the real lead-screw release needs continuous-rotation control or
        repeated incremental pulses. Logged here so the sequence flow is intact.
        """
        self.get_logger().info(
            f'arm_{self.arm_num}: spin servo {servo_id} {direction} x{turns} (stub)')
        # TODO: publish actual ServosPosition increments for servo `servo_id`.
        self.wait_motion(0.3 * turns)

    def pickup_sequence(self, coord):
        """MISC PICKUP SEQUENCE: down (offset+0.05), wait, up (offset)."""
        down = [coord[0], coord[1], coord[2] - (self.vertical_offset + 0.05)] #---CHANGE---#
        self.move_to(down)
        self.wait_motion()
        up = [coord[0], coord[1], down[2] + self.vertical_offset]
        self.move_to(up)
        self.wait_motion()

    def drop_sequence(self, coord):
        """MISC DROP SEQUENCE: down (offset), release via servo 10, reset."""
        down = [coord[0], coord[1], coord[2] - self.vertical_offset]
        self.move_to(down)
        self.wait_motion()
        self.spin_servo(10, 'CW', 3)   #---CHANGE---#
        time.sleep(0.5)
        self.spin_servo(10, 'CCW', 3)  #---CHANGE---#

    def pick_object(self):
        """Move straight to the detected object, pick it, park at standby."""
        if self.coordinate is None:
            return
        self.busy = True
        self.publish_state(ST_PICKING)

        target = [self.coordinate[i] + self.offset[i] for i in range(3)]
        self.get_logger().info(f'arm_{self.arm_num}: picking object at {target}')

        if not self.move_to(target):
            # Could not reach the object; release the assignment back to idle.
            self.publish_state(ST_IDLE)
            self.busy = False
            return
        self.wait_motion()

        #TODO: Add wrist rotation based on self.angle

        self.pickup_sequence(target)

        # Park at global standby and wait for the master's placement trigger.
        self.move_to(self.standby_pos)
        self.wait_motion()
        self.publish_state(ST_STANDBY)
        self.get_logger().info(f'arm_{self.arm_num}: holding block at standby.')

    def place_callback(self, msg):
        """Master-triggered placement. Master has already set state -> placing."""
        if len(msg.data) < 3:
            return
        place_coord = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]
        # msg.data[3] (placement angle) is available for wrist orientation.
        self.publish_state(ST_PLACING)
        self.get_logger().info(f'arm_{self.arm_num}: placing at {place_coord}')

        self.move_to(place_coord)
        self.wait_motion()
        self.drop_sequence(place_coord)

        # Return home and report idle so the master marks the objective complete.
        self.move_to(self.global_standby)
        self.wait_motion()
        self.coordinate = None
        self.busy = False
        self.publish_state(ST_IDLE)

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