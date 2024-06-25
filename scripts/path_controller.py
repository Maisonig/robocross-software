#!/usr/bin/env python3
import array
import sys
import time

import cv2
import rclpy
import numpy as np
import depthai as dai
import ros2_numpy as rnp
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, PointField


def quaternion_to_euler(x, y, z, w):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2 > +1.0, +1.0, t2)
    # t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2 < -1.0, -1.0, t2)
    # t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z


class PathController(Node):
    FPS = 30

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.pathSub = self.create_subscription(Path,
                                                '/path',
                                                self.path_callback,
                                                10)

        self.odomSub = self.create_subscription(Odometry,
                                                '/odom',
                                                self.odom_callback,
                                                10)
        self.cmdPub = self.create_publisher(Twist,
                                            'cmd_vel',
                                            10)

        self.mainTimer = self.create_timer(0.067, self.timer_callback)

        self.pathData = Path()
        self.odomData = Odometry()

        self.oldPath = Path()
        self.oldPathCounter = 0

    def path_callback(self, msg):
        self.pathData = msg

    def odom_callback(self, msg):
        self.odomData = msg

    def timer_callback(self):
        if self.pathData != Path() and self.odomData != Odometry():
            if len(self.pathData.poses) > 3:
                pose = self.pathData.poses[2]
                vel = 0.3
                z = pose.pose.orientation.z

                x0, y0, z0 = quaternion_to_euler(self.odomData.pose.pose.orientation.x,
                                                 self.odomData.pose.pose.orientation.y,
                                                 self.odomData.pose.pose.orientation.z,
                                                 self.odomData.pose.pose.orientation.w)
                z0 = np.deg2rad(z0)
                z2 = int(z / np.pi)
                z3 = z2 * np.pi
                z = z - z3
                steer = z0 - z
                msg = Twist()
                msg.linear.x = 2.5
                msg.angular.z = -2 * steer
                self.cmdPub.publish(msg)
            else:
                msg = Twist()
                self.cmdPub.publish(msg)
        else:
            msg = Twist()
            self.cmdPub.publish(msg)


def main():
    rclpy.init()
    node = PathController("path_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
