#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node
from utils import euler_from_quaternion
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose


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

        self.pSteeringRatio = 2.0
        self.dSteeringRatio = 0.0

        self.speed = 1.

    def path_callback(self, msg):
        self.pathData = msg

    def odom_callback(self, msg):
        self.odomData = msg

    def timer_callback(self):
        if self.pathData.poses != [] and self.odomData != Odometry():
            pose = self.odomData.pose.pose
            path = self.pathData.poses
            steer = self.pd_control_steer(pose, path)
            speed = self.speed
            print(speed)
        else:
            speed = 0.
            steer = 0.
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = steer
        self.cmdPub.publish(msg)

    def pd_control_steer(self, pose: Pose, path: list[PoseStamped]):
        angle = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        goal_pose = path[0].pose
        goal_angle = goal_pose.orientation.z
        delta_angle = goal_angle - angle

        p_ = self.pSteeringRatio * delta_angle

        i = 2
        d_ = self.dSteeringRatio * delta_angle
        if len(path) > 2:
            for p in path[1:]:
                th = euler_from_quaternion(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z,
                                           p.pose.orientation.w)[2]
                dt = th - angle
                d_ += dt * (self.dSteeringRatio / i)
                if i == 5:
                    break
        pd_ = p_ - d_
        return pd_


def main():
    rclpy.init()
    node = PathController("path_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
