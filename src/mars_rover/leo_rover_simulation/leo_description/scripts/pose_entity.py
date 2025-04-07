#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import sys

class PrintEntityPoseNode(Node):
    def __init__(self, entity_name):
        super().__init__('print_entity_pose')
        self.entity_name = entity_name
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )
        self.get_logger().info(f"Tracking entity: {self.entity_name}")

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index(self.entity_name)
            pose = msg.pose[idx]
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            orientation = pose.orientation
            yaw = self.get_yaw_from_orientation(orientation)

            self.get_logger().info(f"Entity '{self.entity_name}' Pose: x={x}, y={y}, z={z}, yaw={yaw}")
        except ValueError:
            self.get_logger().warn(f"Entity '{self.entity_name}' not found in /model_states")

    def get_yaw_from_orientation(self, orientation):
        import math
        # Extract the yaw from the quaternion orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 2:
        print("Usage: ros2 run deepmind_bot_gazebo pose_entity <entity_name>")
        sys.exit(1)

    entity_name = sys.argv[1]
    node = PrintEntityPoseNode(entity_name)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
