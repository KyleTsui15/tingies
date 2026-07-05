#!/usr/bin/env python3
# encoding: utf-8
#
# Master / coordinator node.
#
# This is a concrete (but intentionally skeletal) implementation of the
# pseudocode that previously lived in this file. It is meant as a *foundation*
# for the intended flow, not a production controller. The original pseudocode is
# kept verbatim at the bottom of this file for reference.
#
# Flow implemented here (one master, N arms):
#
#   1. Master owns the build plan: a per-layer dictionary of objectives
#      (layer -> block_type -> list of {location, angle, state}).
#   2. For every arm that is free, the master reserves an unassigned objective
#      and publishes the required block type to  /arm_{num}/block_class.
#         -> yolo_seg_node looks for that class and publishes pos/angle.
#         -> vision_move_bridge picks it up and parks at global standby
#            (publishing  /arm_{num}/arm_state = "picking" then "standby").
#   3. Master "Step 2": only ONE arm may be in the shared build zone at a time.
#      While no arm is "placing", the master takes the next arm sitting at
#      "standby", marks it "placing", and hands the placement target to that
#      arm's bridge. When the arm reports "idle" again, the objective is marked
#      complete and the layer advances once every objective is done.
#
# --------------------------------------------------------------------------
# NOTE ON A LOGIC GAP IN THE PSEUDOCODE:
# The pseudocode lists "DROP SEQUENCE" and "Move back to global standby" under
# the *master* in Step 2. The master node has no kinematics or servo control --
# only vision_move_bridge does. Physically the master cannot move an arm.
# So the actual DROP SEQUENCE / motion is delegated to vision_move_bridge (see
# the labelled section in that file); the master only orchestrates: it sets the
# state to "placing", sends the target via /arm_{num}/place_target, and waits to
# observe the arm return to "idle" before marking the objective complete.
# Everything else follows the pseudocode as written.
# --------------------------------------------------------------------------

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray


# Objective lifecycle states
UNASSIGNED = "unassigned"
RESERVED = "reserved"
COMPLETE = "complete"

# Arm states seen on /arm_{num}/arm_state
ST_IDLE = "idle"        # free, ready for a new block_class assignment
ST_PICKING = "picking"  # moving to / grabbing the detected object
ST_STANDBY = "standby"  # holding a block at global standby, ready to place
ST_PLACING = "placing"  # currently in the shared build zone placing a block


# === Modified Master Integration: shared build frame helpers ===
def normalize_angle_deg(angle):
    """Normalize a planar angle to [-180, 180) degrees."""
    return (float(angle) + 180.0) % 360.0 - 180.0
# === End Modified Master Integration: shared build frame helpers ===


class MasterNode(Node):
    def __init__(self):
        super().__init__("master_node")

        # ---- Configuration ------------------------------------------------
        self.declare_parameter("num_arms", 2)
        self.num_arms = self.get_parameter("num_arms").value
        self.arm_nums = list(range(1, self.num_arms + 1))

        self.global_standby = [0.0, -0.20, 0.20] ##----CHANGE----##

        # === Modified Master Integration: shared build frame configuration ===
        # Build locations are now written in ONE shared build frame:
        #   x/y origin = center of the build square, z = height above build plate.
        # Each arm then gets a transform from that build frame into its own local
        # IK frame before /arm_{n}/place_target is published.
        #
        # build_half_width is the measured distance from each arm base-frame
        # origin to the build center. The default 0.15 m preserves the old
        # behavior where [0.15, 0.0, z] was roughly the center target for one arm.
        self.declare_parameter("build_half_width", 0.15)
        self.build_half_width = float(self.get_parameter("build_half_width").value)

        h = self.build_half_width
        self.arm_frames = {
            # origin: arm base position in the shared build frame.
            # yaw_deg: direction of the arm's local +X/forward axis in build-frame degrees.
            # Default square-midpoint layout, looking inward toward the build center:
            # arm_1 = south side, arm_2 = east side, arm_3 = north side, arm_4 = west side.
            1: {"origin": [0.0, -h, 0.0], "yaw_deg": 90.0},
            2: {"origin": [h, 0.0, 0.0], "yaw_deg": 180.0},
            3: {"origin": [0.0, h, 0.0], "yaw_deg": -90.0},
            4: {"origin": [-h, 0.0, 0.0], "yaw_deg": 0.0},
        }

        for n in self.arm_nums:
            if n not in self.arm_frames:
                self.arm_frames[n] = {"origin": [0.0, 0.0, 0.0], "yaw_deg": 0.0}
                self.get_logger().warn(
                    f"arm_{n} has no configured build-frame transform; using identity transform."
                )
        # === End Modified Master Integration: shared build frame configuration ===

        # ---- Build plan ---------------------------------------------------
        # === Modified Master Integration: build-frame final locations ===
        # These are no longer arm-local IK coordinates. They are shared build
        # coordinates measured from the center of the LEGO build area. A center
        # placement is therefore [0.0, 0.0, z] for every arm. The selected arm's
        # local position/orientation is computed in build_to_arm_target().
        self.objectives = {
            0: {
                "red_block": [
                    {"location": [0.00, 0.00, 0.02], "angle": 0.0, "state": UNASSIGNED},
                    {"location": [0.00, 0.05, 0.02], "angle": 0.0, "state": UNASSIGNED},
                ],
                "blue_block": [
                    {"location": [0.00, -0.05, 0.02], "angle": 0.0, "state": UNASSIGNED},
                ],
            },
            1: {
                "red_block": [
                    {"location": [0.00, 0.00, 0.06], "angle": 0.0, "state": UNASSIGNED},
                ],
            },
        }
        # === End Modified Master Integration: build-frame final locations ===
        self.layer = 0

        # ---- Per-arm runtime bookkeeping ----------------------------------
        # Latest reported state for each arm (assume all start at idle/standby).
        self.arm_state = {n: ST_IDLE for n in self.arm_nums}
        # The objective dict currently reserved by each arm (or None).
        self.arm_assignment = {n: None for n in self.arm_nums}
        # True once the master has triggered a placement for this arm and is
        # waiting for it to come back to idle.
        self.arm_placing = {n: False for n in self.arm_nums}

        # ---- ROS interfaces (per arm) -------------------------------------
        self.block_class_pub = {}
        self.place_target_pub = {}
        # NOTE: the master both publishes to and subscribes from arm_state, and
        # so does the bridge (multi-publisher topic). This mirrors the
        # pseudocode; in a hardened system you'd likely split command vs status.
        self.state_pub = {}

        for n in self.arm_nums:
            ns = f"/arm_{n}"
            self.block_class_pub[n] = self.create_publisher(String, f"{ns}/block_class", 10)
            self.place_target_pub[n] = self.create_publisher(Float32MultiArray, f"{ns}/place_target", 10)
            self.state_pub[n] = self.create_publisher(String, f"{ns}/arm_state", 10)
            self.create_subscription(
                String, f"{ns}/arm_state",
                lambda msg, arm=n: self.arm_state_callback(arm, msg), 10,
            )

        # ---- Control loops ------------------------------------------------
        self.assign_timer = self.create_timer(1.0, self.assignment_loop)
        self.place_timer = self.create_timer(0.5, self.placement_loop)

        self.get_logger().info(
            f"Master node started for {self.num_arms} arm(s). Starting at layer {self.layer}."
        )

    # ======================================================================
    # State intake
    # ======================================================================
    def arm_state_callback(self, arm, msg):
        new_state = msg.data
        if new_state == self.arm_state[arm]:
            return
        self.get_logger().info(f"arm_{arm}: {self.arm_state[arm]} -> {new_state}")
        self.arm_state[arm] = new_state

        # An arm that was placing has now reported idle -> objective complete.
        if new_state == ST_IDLE and self.arm_placing[arm]:
            self.complete_assignment(arm)

    # ======================================================================
    # Step 1: assign block classes to free arms
    # ======================================================================
    def assignment_loop(self):
        if self.layer not in self.objectives:
            return  # build finished

        for n in self.arm_nums:
            # Only hand work to a genuinely free arm.
            if self.arm_state[n] != ST_IDLE or self.arm_assignment[n] is not None:
                continue

            objective = self.find_unassigned_objective()
            if objective is None:
                continue

            block_type, obj = objective
            obj["state"] = RESERVED  # reserve so no other arm grabs the same one
            self.arm_assignment[n] = obj

            msg = String()
            msg.data = block_type
            self.block_class_pub[n].publish(msg)
            self.get_logger().info(
                f"arm_{n}: assigned '{block_type}' -> location {obj['location']}"
            )

    def find_unassigned_objective(self):
        """Return (block_type, objective_dict) for the next thing to build, or None."""
        layer_plan = self.objectives.get(self.layer, {})
        for block_type, obj_list in layer_plan.items():
            for obj in obj_list:
                if obj["state"] == UNASSIGNED:
                    return block_type, obj
        return None

    # === Modified Master Integration: build frame -> arm local frame ===
    def build_to_arm_target(self, arm, obj):
        """Convert one shared build-frame objective into this arm's IK frame.

        obj["location"] is [x, y, z] in the shared build frame.
        Returned target is [x, y, z, angle] in the selected arm's local frame,
        ready to publish on /arm_{arm}/place_target.
        """
        frame = self.arm_frames[arm]
        origin = frame["origin"]
        yaw_deg = float(frame["yaw_deg"])
        yaw = math.radians(yaw_deg)
        c = math.cos(yaw)
        s = math.sin(yaw)

        build_x = float(obj["location"][0])
        build_y = float(obj["location"][1])
        build_z = float(obj["location"][2])

        # Translate from build origin to this arm base, then rotate by -yaw.
        dx = build_x - float(origin[0])
        dy = build_y - float(origin[1])
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        local_z = build_z - float(origin[2])

        # If obj["angle"] is measured in the build frame, subtract the arm yaw
        # so the bridge receives the equivalent wrist/placement angle locally.
        local_angle = normalize_angle_deg(float(obj.get("angle", 0.0)) - yaw_deg)

        return [local_x, local_y, local_z, local_angle]
    # === End Modified Master Integration: build frame -> arm local frame ===

    # ======================================================================
    # Step 2: serialise placement into the shared build zone (one arm at a time)
    # ======================================================================
    def placement_loop(self):
        # If any arm is in the build zone, wait for it to finish.
        if any(s == ST_PLACING for s in self.arm_state.values()):
            return

        # Find the first arm waiting at standby with a reserved objective.
        for n in self.arm_nums:
            if self.arm_state[n] == ST_STANDBY and self.arm_assignment[n] is not None:
                self.trigger_placement(n)
                return  # only start one per cycle

    def trigger_placement(self, arm):
        obj = self.arm_assignment[arm]

        # Mark the arm as placing *before any movement is made* (pseudocode).
        self.arm_placing[arm] = True
        self.arm_state[arm] = ST_PLACING
        self.publish_state(arm, ST_PLACING)

        # Hand the placement target to the bridge, which owns the motion +
        # DROP SEQUENCE (see logic-gap note at the top of this file).
        # === Modified Master Integration: publish arm-local placement target ===
        local_target = self.build_to_arm_target(arm, obj)
        target = Float32MultiArray()
        target.data = [float(v) for v in local_target]
        self.place_target_pub[arm].publish(target)
        self.get_logger().info(
            f"arm_{arm}: placement triggered -> build {obj['location']} "
            f"as local {[round(v, 4) for v in local_target[:3]]}, "
            f"angle {local_target[3]:.1f}"
        )
        # === End Modified Master Integration: publish arm-local placement target ===

    def complete_assignment(self, arm):
        obj = self.arm_assignment[arm]
        if obj is not None:
            obj["state"] = COMPLETE
            self.get_logger().info(f"arm_{arm}: objective complete at {obj['location']}")

        self.arm_assignment[arm] = None
        self.arm_placing[arm] = False

        self.maybe_advance_layer()

    def maybe_advance_layer(self):
        layer_plan = self.objectives.get(self.layer, {})
        all_done = all(
            obj["state"] == COMPLETE
            for obj_list in layer_plan.values()
            for obj in obj_list
        )
        if all_done:
            self.layer += 1
            if self.layer in self.objectives:
                self.get_logger().info(f"Layer complete -> advancing to layer {self.layer}.")
            else:
                self.get_logger().info("All layers complete. Build finished.")

    # ======================================================================
    # Helpers
    # ======================================================================
    def publish_state(self, arm, state):
        msg = String()
        msg.data = state
        self.state_pub[arm].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# ==========================================================================
# ORIGINAL PSEUDOCODE (kept for reference)
# ==========================================================================
# MISC
#     PICKUP SEQUENCE
#         Move the arm down by "vertical_offset+0.05"
#         Wait until motion is complete.
#         Move the arm up by "vertical_offset"
#     DROP SEQUENCE (For building)
#         Move to arm down by "vertical_offset"
#         Spin servo id 10 CW 3 times to release the block by spinning lead screw
#         Wait 0.5 seconds.
#         Spin servo id 10 CCW 3 times to reset the gripper for the next pickup
#
# Master Node
#     Startup conditions: All arms at global_standby
#     vars: Layer, Dictionary of block final locations
#           (Layer -> Block type -> list of final locations -> state of that
#            objective (unassigned, reserved, complete)), Global standby location
#     Inferred from Layer & Dictionary -> current block type(s) needed published
#     to /arm_{num}/block_class in a for loop for each arm. The object is marked
#     reserved but not removed from the dictionary until placement is complete to
#     prevent multiple arms from trying to pick the same block.
#
# Yolo_seg_node: subscribes /arm_{num}/block_class, publishes
#     /arm_{num}/yolo_seg/object_pos and /arm_{num}/yolo_seg/object_angle
#
# Vision_move_bridge node: subscribes object_pos/object_angle, immediately moves
#     to object (state "picking"), PICKUP SEQUENCE, moves to global_standby and
#     waits for trigger (state "standby").
#
# Master Node Step 2: if no arm is "placing", take an arm in "standby", set
#     "placing", DROP SEQUENCE, mark objective complete, return to standby, set
#     state "idle".
