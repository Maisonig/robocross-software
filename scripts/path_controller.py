#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node
from utils import euler_from_quaternion
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose


def pid_control_steer(pos: Pose, path: list[PoseStamped]):
    pass


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
        if self.pathData.poses != [] and self.odomData != Odometry():
            pose = self.odomData.pose
            path = self.pathData.poses

            # if len(self.pathData.poses) > 3:
                # pose = self.pathData.poses[0]
                # z = pose.pose.orientation.z
                #
                # x0, y0, z0 = euler_from_quaternion(self.odomData.pose.pose.orientation.x,
                #                                    self.odomData.pose.pose.orientation.y,
                #                                    self.odomData.pose.pose.orientation.z,
                #                                    self.odomData.pose.pose.orientation.w)
                # z3 = int(z / np.pi) * np.pi
                # z = z - z3
                # steer = z0 - z

                # steer = z0 - z
                # msg = Twist()
                # msg.linear.x = 1.
                # msg.angular.z = 3 * -steer
                # self.cmdPub.publish(msg)
        #     else:
        #         msg = Twist()
        #         self.cmdPub.publish(msg)
        # else:
        #     msg = Twist()
        #     self.cmdPub.publish(msg)


def main():
    rclpy.init()
    node = PathController("path_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
