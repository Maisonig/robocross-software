#!/usr/bin/env python3

import cv2
import time
import math
import rclpy
import numpy as np
import ros2_numpy as rnp
from builtin_interfaces.msg import Time
from utils import *

from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, LaserScan, Image
from geometry_msgs.msg import TransformStamped, Polygon
from tf2_py import TransformException
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener

UNKNOWN_CELL = -1
FREE_CELL = 0
OBSTACLE_CELL = 100

focalX = 1451
focalY = 2312
centerX = 640
centerY = 360
cameraDefaultIntrinsics = [[3056.9775390625, 0.0, 1870.62060546875],
                           [0.0, 3056.9775390625, 1087.102294921875],
                           [0.0, 0.0, 1.0]]


def transform_image_coord(u, depth):
    dmx = np.sqrt((centerX - u) ** 2 + focalX ** 2)
    y = depth * (centerX - u) / dmx
    x = depth
    return x, y


class Mapping(Node):
    RELEVANT_DATA_TIMEOUT = 2.

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('frequency', 30)
        self.declare_parameter('scan_topic', '/front_camera/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_base_frame', 'chassis')
        self.declare_parameter('scan_frame', 'front_camera_link')
        self.declare_parameter('map_size', 100)
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_collision_radius', 1.)

        freq = self.get_parameter('frequency').get_parameter_value().integer_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

        self.robotBaseFrame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.scanFrame = self.get_parameter('scan_frame').get_parameter_value().string_value
        self.mapFrame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.mapSize = self.get_parameter('map_size').get_parameter_value().integer_value
        self.mapRes = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.robotColRadius = self.get_parameter('robot_collision_radius').get_parameter_value().double_value

        self.mainTimer = self.create_timer(1 / freq, self.main_timer_callback)
        self.scanSub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.odomSub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.mapPub = self.create_publisher(OccupancyGrid, map_topic, 10)
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)
        self.tfBroadcaster = TransformBroadcaster(self)

        self.odomData = Odometry()
        self.scanData = LaserScan()
        self.mapArray = np.zeros([int(self.mapSize / self.mapRes), int(self.mapSize / self.mapRes)], np.uint8) + 127

    def main_timer_callback(self):
        self.publish_tf()
        map_center = self.mapSize / 2 / self.mapRes
        if self.scanData != LaserScan() and self.odomData != Odometry():
            try:
                transform = self.tfBuffer.lookup_transform(self.mapFrame, self.scanFrame, Time()).transform
                sensor_yaw = euler_from_quaternion(
                    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)[2]
                sensor_x, sensor_y = transform.translation.x, transform.translation.y
                sensor_x, sensor_y = int(sensor_x / self.mapRes + map_center), int(sensor_y / self.mapRes + map_center)
                self.set_scan(self.scanData, sensor_x, sensor_y, sensor_yaw)
                self.publish_map()
            except TransformException as e:
                # self.get_logger().info(f'Could not transform {self.mapFrame} to {self.scanFrame}: {e}')
                pass
        else:
            # self.get_logger().info(f'Could not get odometry/laserscan')
            pass

    def set_scan(self, scan_msg, base_x, base_y, base_yaw):
        base_yaw += scan_msg.angle_min
        angles = [base_yaw + i * scan_msg.angle_increment for i in range(len(scan_msg.ranges))]

        non_empty_ranges = []
        non_empty_angles = []
        for rho, phi in zip(scan_msg.ranges, angles):
            if rho != 0.:
                non_empty_ranges.append(rho)
                non_empty_angles.append(phi)
            else:
                rho = scan_msg.range_max
                x, y = polar_to_decart(rho / self.mapRes, phi)
                x = int(x + base_x)
                y = int(y + base_y)
                cv2.line(self.mapArray, [base_x, base_y], [x, y], [255], 1)
        circles = []
        for rho, phi in zip(non_empty_ranges, non_empty_angles):
            x, y = polar_to_decart(rho / self.mapRes, phi)
            x = int(x + base_x)
            y = int(y + base_y)
            cv2.line(self.mapArray, [base_x, base_y], [x, y], [255], 1)
            circles.append([x, y])
        for c in circles:
            cv2.circle(self.mapArray, c, int(self.robotColRadius / self.mapRes), [70], -1)
        for c in circles:
            self.mapArray[c[1], c[0]] = 0

    def publish_map(self):
        oc_array = np.copy(self.mapArray)
        oc_array[oc_array == 0] = 100
        oc_array[oc_array == 255] = 0
        oc_array[oc_array == 127] = 255
        oc_array = np.array(oc_array, np.int8)
        grid = rnp.msgify(OccupancyGrid, oc_array)
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.mapFrame
        grid.info.resolution = self.mapRes
        grid.info.origin.position.x = -self.mapSize / 2
        grid.info.origin.position.y = -self.mapSize / 2
        self.mapPub.publish(grid)

    def publish_tf(self):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.mapFrame
        msg.child_frame_id = self.robotBaseFrame
        msg.transform.translation.x = self.odomData.pose.pose.position.x
        msg.transform.translation.y = self.odomData.pose.pose.position.y
        msg.transform.translation.z = self.odomData.pose.pose.position.z + 1.065
        msg.transform.rotation = self.odomData.pose.pose.orientation
        self.tfBroadcaster.sendTransform(msg)

    def scan_callback(self, msg):
        self.scanData = msg

    def odom_callback(self, msg):
        self.odomData = msg


def main():
    rclpy.init()
    node = Mapping("mapping")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
