#!/usr/bin/env python3

import argparse
import math
import os
import sys
from urllib.parse import SplitResult, urlsplit

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from lxml import etree as ElementTree
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty

class SpawnEntityNode(Node):
    MODEL_DATABASE_TEMPLATE = """\
    <sdf version="1.6">
        <world name="default">
            <include>
                <uri>model://{}</uri>
            </include>
        </world>
    </sdf>"""

    def __init__(self, args):
        super().__init__('spawn_entity')
        parser = argparse.ArgumentParser(
            description='Spawn an entity in gazebo. Gazebo must be started with gazebo_ros_init,\
            gazebo_ros_factory and gazebo_ros_state for all functionalities to work')
        source = parser.add_mutually_exclusive_group(required=True)
        source.add_argument('-file', type=str, metavar='FILE_NAME',
                            help='Load entity xml from file')
        source.add_argument('-topic', type=str, metavar='TOPIC_NAME',
                            help='Load entity xml published on topic')
        source.add_argument('-database', type=str, metavar='ENTITY_NAME',
                            help='Load entity XML from specified entity in GAZEBO_MODEL_PATH \
                            or Gazebo Model Database')
        source.add_argument('-stdin', action='store_true', help='Load entity from stdin')
        parser.add_argument('-entity', required=True, type=str, metavar='ENTITY_NAME',
                            help='Name of entity to spawn')
        parser.add_argument('-reference_frame', type=str, default='',
                            help='Name of the model/body where initial pose is defined.\
                            If left empty or specified as "world", gazebo world frame is used')
        parser.add_argument('-gazebo_namespace', type=str, default='',
                            help='ROS namespace of gazebo offered ROS interfaces. \
                            Default is without any namespace')
        parser.add_argument('-robot_namespace', type=str, default='',
                            help='Change ROS namespace of gazebo-plugins')
        parser.add_argument('-timeout', type=float, default=30.0,
                            help='Number of seconds to wait for the spawn and delete services to \
                            become available')
        parser.add_argument('-unpause', action='store_true',
                            help='Unpause physics after spawning entity')
        parser.add_argument('-wait', type=str, metavar='ENTITY_NAME',
                            help='Wait for entity to exist')
        parser.add_argument('-spawn_service_timeout', type=float, metavar='TIMEOUT',
                            help="DEPRECATED: Use '-timeout' instead.")
        parser.add_argument('-x', type=float, default=0,
                            help='x component of initial position, meters')
        parser.add_argument('-y', type=float, default=0,
                            help='y component of initial position, meters')
        parser.add_argument('-z', type=float, default=0,
                            help='z component of initial position, meters')
        parser.add_argument('-R', type=float, default=0,
                            help='roll angle of initial orientation, radians')
        parser.add_argument('-P', type=float, default=0,
                            help='pitch angle of initial orientation, radians')
        parser.add_argument('-Y', type=float, default=0,
                            help='yaw angle of initial orientation, radians')
        parser.add_argument('-package_to_model', action='store_true', help='convert urdf \
                            <mesh filename="package://..." to <mesh filename="model://..."')
        parser.add_argument('-b', dest='bond', action='store_true', help='bond to gazebo \
                             and delete the entity when this program is interrupted')
        self.args = parser.parse_args(args[1:])

    def run(self):
        if self.args.wait:
            self.entity_exists = False

            def entity_cb(entity):
                self.entity_exists = self.args.wait in entity.name

            self.subscription = self.create_subscription(
                ModelStates, '%s/model_states' % self.args.gazebo_namespace, entity_cb, 10)

            self.get_logger().info(
                'Waiting for entity {} before proceeding.'.format(self.args.wait))

            while rclpy.ok() and not self.entity_exists:
                rclpy.spin_once(self)
                pass

        if self.args.file:
            self.get_logger().info('Loading entity XML from file %s' % self.args.file)
            if not os.path.exists(self.args.file):
                self.get_logger().error(f'Error: specified file {self.args.file} does not exist')
                return 1
            if not os.path.isfile(self.args.file):
                self.get_logger().error('Error: specified file %s is not a file', self.args.file)
                return 1
            try:
                f = open(self.args.file, 'r')
                entity_xml = f.read()
            except IOError as e:
                self.get_logger().error('Error reading file {}: {}'.format(self.args.file, e))
                return 1
            if entity_xml == '':
                self.get_logger().error('Error: file %s is empty', self.args.file)
                return 1
        elif self.args.topic:
            self.get_logger().info(
                'Loading entity published on topic %s' % self.args.topic)
            entity_xml = ''

            def entity_xml_cb(msg):
                nonlocal entity_xml
                entity_xml = msg.data

            latched_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subscription = self.create_subscription(
                String, self.args.topic, entity_xml_cb, latched_qos)

            while rclpy.ok() and entity_xml == '':
                self.get_logger().info('Waiting for entity xml on %s' % self.args.topic)
                rclpy.spin_once(self)
                pass

        elif self.args.database:
            self.get_logger().info('Loading entity XML from Gazebo Model Database')
            entity_xml = self.MODEL_DATABASE_TEMPLATE.format(self.args.database)
        elif self.args.stdin:
            self.get_logger().info('Loading entity XML from stdin')
            entity_xml = sys.stdin.read()
            if entity_xml == '':
                self.get_logger().error('Error: stdin buffer was empty')
                return 1

        try:
            xml_parsed = ElementTree.fromstring(entity_xml)
        except ElementTree.ParseError as e:
            self.get_logger().error('Invalid XML: {}'.format(e))
            return 1

        if self.args.package_to_model:
            for element in xml_parsed.iterfind('.//mesh'):
                filename_tag = element.get('filename')
                if filename_tag is None:
                    continue
                url = urlsplit(filename_tag)
                if url.scheme == 'package':
                    url = SplitResult('model', *url[1:])
                    element.set('filename', url.geturl())

        entity_xml = ElementTree.tostring(xml_parsed)

        initial_pose = Pose()
        initial_pose.position.x = float(self.args.x)
        initial_pose.position.y = float(self.args.y)
        initial_pose.position.z = float(self.args.z)

        q = quaternion_from_euler(self.args.R, self.args.P, self.args.Y)
        initial_pose.orientation.w = q[0]
        initial_pose.orientation.x = q[1]
        initial_pose.orientation.y = q[2]
        initial_pose.orientation.z = q[3]

        spawn_service_timeout = self.args.timeout
        if self.args.spawn_service_timeout is not None:
            self.get_logger().warning(
                "'-spawn_service_timeout' is deprecated, use '-timeout' instead")
            spawn_service_timeout = self.args.spawn_service_timeout
        success = self._spawn_entity(entity_xml, initial_pose, spawn_service_timeout)
        if not success:
            self.get_logger().error('Spawn service failed. Exiting.')
            return 1

        if self.args.unpause:
            client = self.create_client(Empty, '%s/unpause_physics' % self.args.gazebo_namespace)
            if client.wait_for_service(timeout_sec=self.args.timeout):
                self.get_logger().info(
                    'Calling service %s/unpause_physics' % self.args.gazebo_namespace)
                client.call_async(Empty.Request())
            else:
                self.get_logger().error('Service %s/unpause_physics unavailable. \
                                         Was Gazebo started with GazeboRosInit?' % (
                                             self.args.gazebo_namespace))

        if self.args.bond:
            self.get_logger().info(
                'Waiting for shutdown to delete entity [{}]'.format(self.args.entity))
            try:
                rclpy.spin(self)
            except KeyboardInterrupt:
                self.get_logger().info('Ctrl-C detected')
            self._delete_entity()

        return 0

    def _spawn_entity(self, entity_xml, initial_pose, timeout=5.0):
        if timeout < 0:
            self.get_logger().error('spawn_entity timeout must be greater than zero')
            return False
        self.get_logger().info(
            'Waiting for service %s/spawn_entity, timeout = %.f' % (
                self.args.gazebo_namespace, timeout))
        self.get_logger().info('Waiting for service %s/spawn_entity' % self.args.gazebo_namespace)
        client = self.create_client(SpawnEntity, '%s/spawn_entity' % self.args.gazebo_namespace)
        if client.wait_for_service(timeout_sec=timeout):
            req = SpawnEntity.Request()
            req.name = self.args.entity
            req.xml = str(entity_xml, 'utf-8')
            req.robot_namespace = self.args.robot_namespace
            req.initial_pose = initial_pose
            req.reference_frame = self.args.reference_frame
            self.get_logger().info('Calling service %s/spawn_entity' % self.args.gazebo_namespace)
            srv_call = client.call_async(req)
            while rclpy.ok():
                if srv_call.done():
                    self.get_logger().info('Spawn status: %s' % srv_call.result().status_message)
                    break
                rclpy.spin_once(self)
            return srv_call.result().success
        self.get_logger().error(
            'Service %s/spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?' % (
                self.args.gazebo_namespace))
        return False

    def _delete_entity(self):
        self.get_logger().info('Deleting entity [{}]'.format(self.args.entity))
        client = self.create_client(
            DeleteEntity, '%s/delete_entity' % self.args.gazebo_namespace)
        if client.wait_for_service(timeout_sec=self.args.timeout):
            req = DeleteEntity.Request()
            req.name = self.args.entity
            self.get_logger().info(
                'Calling service %s/delete_entity' % self.args.gazebo_namespace)
            srv_call = client.call_async(req)
            while rclpy.ok():
                if srv_call.done():
                    self.get_logger().info(
                        'Deleting status: %s' % srv_call.result().status_message)
                    break
                rclpy.spin_once(self)
        else:
            self.get_logger().error(
                'Service %s/delete_entity unavailable. ' +
                'Was Gazebo started with GazeboRosFactory?')

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def main(args=sys.argv):
    rclpy.init(args=args)
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    spawn_entity_node = SpawnEntityNode(args_without_ros)
    spawn_entity_node.get_logger().info('Spawn Entity started')
    exit_code = spawn_entity_node.run()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
