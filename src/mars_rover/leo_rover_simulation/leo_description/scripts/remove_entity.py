#!/usr/bin/env python3

import argparse
import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity

class RemoveEntityNode(Node):
    def __init__(self, args):
        super().__init__('remove_entity')
        parser = argparse.ArgumentParser(
            description='Remove an entity from Gazebo. Gazebo must be started with gazebo_ros_init, '
                        'gazebo_ros_factory and gazebo_ros_state for all functionalities to work')
        parser.add_argument('-entity', required=True, type=str, metavar='ENTITY_NAME',
                            help='Name of entity to remove')
        parser.add_argument('-gazebo_namespace', type=str, default='',
                            help='ROS namespace of gazebo offered ROS interfaces. '
                                 'Default is without any namespace')
        parser.add_argument('-timeout', type=float, default=30.0,
                            help='Number of seconds to wait for the delete service to become available')
        self.args = parser.parse_args(args[1:])

    def run(self):
        """
        Run node, removing the specified entity.

        Returns exit code, 1 for failure, 0 for success
        """
        success = self._remove_entity()
        if not success:
            self.get_logger().error('Removing entity failed. Exiting.')
            return 1
        return 0

    def _remove_entity(self, timeout=5.0):
        if timeout < 0:
            self.get_logger().error('remove_entity timeout must be greater than zero')
            return False
        self.get_logger().info(
            'Waiting for service %s/delete_entity, timeout = %.f' % (
                self.args.gazebo_namespace, timeout))
        self.get_logger().info('Waiting for service %s/delete_entity' % self.args.gazebo_namespace)
        client = self.create_client(DeleteEntity, '%s/delete_entity' % self.args.gazebo_namespace)
        if client.wait_for_service(timeout_sec=timeout):
            req = DeleteEntity.Request()
            req.name = self.args.entity
            self.get_logger().info('Calling service %s/delete_entity' % self.args.gazebo_namespace)
            srv_call = client.call_async(req)
            while rclpy.ok():
                if srv_call.done():
                    self.get_logger().info('Delete status: %s' % srv_call.result().status_message)
                    return srv_call.result().success
                rclpy.spin_once(self)
        self.get_logger().error(
            'Service %s/delete_entity unavailable. Was Gazebo started with GazeboRosFactory?' % (
                self.args.gazebo_namespace))
        return False

def main(args=sys.argv):
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    remove_entity_node = RemoveEntityNode(args_without_ros)
    remove_entity_node.get_logger().info('Remove Entity started')
    exit_code = remove_entity_node.run()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()