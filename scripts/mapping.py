#!/usr/bin/env python3

import cv2
import time
import rclpy
import numpy as np
import ros2_numpy as rnp

from utils import *
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry


class Mapping(Node):

    UNKNOWN_CELL = 255
    FREE_CELL = 0
    OBSTACLE_CELL = 100

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('frequency', 30)
        self.declare_parameter('front_scan_topic', '/front_camera/scan')
        self.declare_parameter('rear_scan_topic', '/rear_camera/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_base_frame', 'chassis')
        self.declare_parameter('map_size', 150)
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_collision_radius', 2.)
        self.declare_parameter('front_scan_position', [2.2, 0.0, 0.0])
        self.declare_parameter('rear_scan_position', [-2.2, 0.0, 0.0])

        freq = self.get_parameter('frequency').get_parameter_value().integer_value
        front_scan_topic = self.get_parameter('front_scan_topic').get_parameter_value().string_value
        rear_scan_topic = self.get_parameter('rear_scan_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        map_topic = self.get_parameter('map_topic').get_parameter_value().string_value

        self.robotBaseFrame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.mapFrame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.mapSize = self.get_parameter('map_size').get_parameter_value().integer_value
        self.mapRes = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.robotColRadius = self.get_parameter('robot_collision_radius').get_parameter_value().double_value
        self.frontScanPos = self.get_parameter('front_scan_position').get_parameter_value().double_array_value
        self.rearScanPos = self.get_parameter('rear_scan_position').get_parameter_value().double_array_value

        self.mainTimer = self.create_timer(1 / freq, self.main_timer_callback)
        self.frontScanSub = self.create_subscription(LaserScan, front_scan_topic, self.front_scan_callback, 10)
        self.rearScanSub = self.create_subscription(LaserScan, rear_scan_topic, self.rear_scan_callback, 10)
        self.odomSub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.mapPub = self.create_publisher(OccupancyGrid, map_topic, 10)
        self.tfBroadcaster = TransformBroadcaster(self)

        self.odomData = Odometry()
        self.frontScanData = LaserScan()
        self.rearScanData = LaserScan()
        self.mapArray = np.zeros([int(self.mapSize / self.mapRes), int(self.mapSize / self.mapRes)], np.uint8) + 255

    def main_timer_callback(self):
        self.publish_tf()

        start_time = time.time()

        self.set_scan(self.frontScanData, self.frontScanPos)
        self.set_scan(self.rearScanData, self.rearScanPos)
        self.set_obstacles()

        # print(1 / (time.time() - start_time))

        self.publish_map()

    def set_obstacles(self):
        indexes = np.where(self.mapArray == 0)
        ys, xs = indexes[0], indexes[1]
        for a, b in zip(xs, ys):
            cv2.circle(self.mapArray, [a, b], int(self.robotColRadius / self.mapRes), [70], -1)
        for a, b in zip(xs, ys):
            try:
                self.mapArray[b, a] = 0
            except IndexError:
                self.get_logger().info('Robot sensor vision is out of bounds')
                break

    def set_scan(self, scan_msg, scan_pos):
        if scan_msg != LaserScan() and self.odomData != Odometry():
            map_center = self.mapSize / 2 / self.mapRes
            rho, th = decart_to_polar(scan_pos[0] / self.mapRes, scan_pos[1] / self.mapRes)
            q = self.odomData.pose.pose.orientation
            base_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)[2]
            sensor_x, sensor_y = polar_to_decart(rho, base_yaw)
            base_x = int(map_center + self.odomData.pose.pose.position.x / self.mapRes + sensor_x)
            base_y = int(map_center + self.odomData.pose.pose.position.y / self.mapRes + sensor_y)
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
            for rho, phi in zip(non_empty_ranges, non_empty_angles):
                x, y = polar_to_decart(rho / self.mapRes, phi)
                x = int(x + base_x)
                y = int(y + base_y)
                cv2.line(self.mapArray, [base_x, base_y], [x, y], [255], 1)
                try:
                    self.mapArray[y, x] = 0
                except IndexError:
                    self.get_logger().info('Robot sensor vision is out of bounds')
                    break
        else:
            pass

    def publish_map(self):
        oc_array = np.copy(self.mapArray)
        oc_array[oc_array == 0] = self.OBSTACLE_CELL
        oc_array[oc_array == 255] = self.FREE_CELL
        oc_array[oc_array == 127] = self.UNKNOWN_CELL
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

    def front_scan_callback(self, msg):
        self.frontScanData = msg

    def rear_scan_callback(self, msg):
        self.rearScanData = msg

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
