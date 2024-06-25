#!/usr/bin/env python3

import cv2
import rclpy
import imutils
import numpy as np
import ros2_numpy as rnp
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from math import asin, atan2, degrees
from nav_msgs.msg import OccupancyGrid, Odometry


def polar_to_decart(rho, phi):
    return rho * np.cos(phi), rho * np.sin(phi)


def decart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan2(y, x)


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    x_ = degrees(atan2(t0, t1))
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    y_ = degrees(asin(t2))
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    z_ = degrees(atan2(t3, t4))
    return x_, y_, z_


def get_combined_image(glob, loc):
    mask = np.copy(loc)
    mask[mask == 0] = 255
    mask[mask == 127] = 0
    fg = cv2.bitwise_or(loc, loc, mask=mask)
    mask = cv2.bitwise_not(mask)
    bk = cv2.bitwise_or(glob, glob, mask=mask)
    return cv2.bitwise_or(fg, bk)


def rotate_uint8_grid(grid, phi):
    o_grid = np.copy(grid)
    o_grid[o_grid != 0] = 1
    o_grid[o_grid == 0] = 255
    o_grid[o_grid == 1] = 0
    o_rotated = imutils.rotate(o_grid, phi)
    o_rotated[o_rotated <= 127] = 0
    o_rotated[o_rotated > 127] = 128

    f_grid = np.copy(grid)
    f_grid[f_grid != 255] = 0
    f_rotated = imutils.rotate(f_grid, angle=phi)
    f_rotated[f_rotated <= 127] = 0
    f_rotated[f_rotated > 127] = 255
    rotated = o_rotated + f_rotated
    rotated[rotated == 0] = 127
    rotated[rotated == 128] = 0
    return rotated


def rotate_uint8_grid1(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result


class GlobalMapper(Node):
    UNKNOWN_CELL = -1
    FREE_CELL = 0
    OBSTACLE_CELL = 100

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.declare_parameter('frequency', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('odom_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('local_grid_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('global_grid_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('global_grid_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('robot_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('grid_size', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('grid_resolution', rclpy.Parameter.Type.DOUBLE)

        params = [
            rclpy.Parameter('frequency', rclpy.Parameter.Type.INTEGER, 30),
            rclpy.Parameter('odom_topic', rclpy.Parameter.Type.STRING, '/odom'),
            rclpy.Parameter('local_grid_topic', rclpy.Parameter.Type.STRING, '/local_map'),
            rclpy.Parameter('global_grid_topic', rclpy.Parameter.Type.STRING, '/global_map'),
            rclpy.Parameter('global_grid_frame', rclpy.Parameter.Type.STRING, 'map'),
            rclpy.Parameter('robot_frame', rclpy.Parameter.Type.STRING, 'chassis'),
            rclpy.Parameter('grid_size', rclpy.Parameter.Type.INTEGER, 70),
            rclpy.Parameter('grid_resolution', rclpy.Parameter.Type.DOUBLE, 0.1),
        ]

        # self.set_parameters(params)

        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        local_grid_topic = self.get_parameter('local_grid_topic').get_parameter_value().string_value
        global_grid_topic = self.get_parameter('global_grid_topic').get_parameter_value().string_value
        self.globalGridFrame = self.get_parameter('global_grid_frame').get_parameter_value().string_value
        self.robotFrame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.gridSize = self.get_parameter('grid_size').get_parameter_value().integer_value
        self.gridRes = self.get_parameter('grid_resolution').get_parameter_value().double_value

        self.tfPublisher = TransformBroadcaster(self)

        self.localMapSub = self.create_subscription(OccupancyGrid,
                                                    local_grid_topic,
                                                    self.local_map_callback,
                                                    10)
        self.odomSub = self.create_subscription(Odometry,
                                                odom_topic,
                                                self.odom_callback,
                                                10)
        self.globalMapPub = self.create_publisher(OccupancyGrid,
                                                  global_grid_topic,
                                                  10)
        self.localOccupancyTimer = self.create_timer(1 / frequency, self.occupancy_timer_callback)

        self.globalMapSize = int(self.gridSize / self.gridRes)

        self.odomData = Odometry()
        self.globalMap = np.zeros((self.globalMapSize, self.globalMapSize), np.uint8) + 127
        self.localMap = None
        self.localMapMsg = OccupancyGrid()

    def occupancy_timer_callback(self):
        try:
            x_offset = int(self.odomData.pose.pose.position.x / self.gridRes + (self.globalMapSize / 2) -
                           (self.localMapMsg.info.width / 2))
            y_offset = int(self.odomData.pose.pose.position.y / self.gridRes + (self.globalMapSize / 2) -
                           (self.localMapMsg.info.height / 2))
            a, b, c = quaternion_to_euler(self.odomData.pose.pose.orientation.x,
                                          self.odomData.pose.pose.orientation.y,
                                          self.odomData.pose.pose.orientation.z,
                                          self.odomData.pose.pose.orientation.w)
            rotated = rotate_uint8_grid(self.localMap, -c)
            y1, y2 = y_offset, y_offset + rotated.shape[0]
            x1, x2 = x_offset, x_offset + rotated.shape[1]
            g = self.globalMap[y1:y2, x1:x2]
            g = get_combined_image(g, rotated)
            self.globalMap[y1:y2, x1:x2] = g
            self.publish_tf()
            self.publish_global_map()
        except Exception as e:
            print(e)

    def publish_global_map(self):
        oc_array = np.copy(self.globalMap)
        oc_array = cv2.resize(oc_array, [int(self.globalMapSize), int(self.globalMapSize)])
        oc_array[oc_array < 127] = 100
        oc_array[oc_array > 127] = 0
        oc_array[oc_array == 127] = 255
        oc_array = np.array(oc_array, np.int8)
        grid = rnp.msgify(OccupancyGrid, oc_array)
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.globalGridFrame
        grid.info.resolution = self.gridRes
        grid.info.origin.position.x = -self.globalMapSize / (2 / self.gridRes)
        grid.info.origin.position.y = -self.globalMapSize / (2 / self.gridRes)
        self.globalMapPub.publish(grid)

    def publish_tf(self):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.globalGridFrame
        msg.child_frame_id = self.robotFrame
        msg.transform.translation.x = self.odomData.pose.pose.position.x
        msg.transform.translation.y = self.odomData.pose.pose.position.y
        msg.transform.translation.z = self.odomData.pose.pose.position.z + 1.065
        msg.transform.rotation = self.odomData.pose.pose.orientation
        self.tfPublisher.sendTransform(msg)

    def local_map_callback(self, msg):
        g = np.array(rnp.numpify(msg), np.uint8)
        g[g == 255] = 127
        g[g == 0] = 255
        g[g == 100] = 0
        self.localMap = g
        self.localMapMsg = msg

    def odom_callback(self, msg):
        self.odomData = msg


def main(args=None):
    rclpy.init(args=args)
    node = GlobalMapper("mapper")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
