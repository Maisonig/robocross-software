#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from utils import euler_from_quaternion, decart_to_polar
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose


class PathController(Node):
    FPS = 30

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('frequency', 30)
        self.declare_parameter('p_steering_ratio', 2.5)
        self.declare_parameter('p_speed_ratio', 1.)
        self.declare_parameter('average_speed', 0.0)

        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value

        self.averageSpeed = self.get_parameter('average_speed').get_parameter_value().double_value
        self.pSteeringRatio = self.get_parameter('p_steering_ratio').get_parameter_value().double_value
        self.pSpeedRatio = self.get_parameter('p_speed_ratio').get_parameter_value().double_value

        self.pathSub = self.create_subscription(Path,
                                                path_topic,
                                                self.path_callback,
                                                10)

        self.odomSub = self.create_subscription(Odometry,
                                                odom_topic,
                                                self.odom_callback,
                                                10)
        self.cmdPub = self.create_publisher(Twist,
                                            cmd_topic,
                                            10)

        self.mainTimer = self.create_timer(1 / frequency, self.timer_callback)

        self.pathData = Path()
        self.odomData = Odometry()

    def path_callback(self, msg):
        self.pathData = msg

    def odom_callback(self, msg):
        self.odomData = msg

    def timer_callback(self):
        if self.pathData.poses != [] and self.odomData != Odometry():
            pose = self.odomData.pose.pose
            path = self.pathData.poses
            steer = self.p_control_steer(pose, path)
            speed = self.p_control_speed(pose, path)
            if speed < 0:
                steer = steer
        else:
            speed = 0.
            steer = 0.
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = steer
        self.cmdPub.publish(msg)

    def p_control_steer(self, pose: Pose, path: list[PoseStamped]):
        angle = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        goal_pose = path[0].pose
        goal_angle = goal_pose.orientation.z
        delta_angle = goal_angle - angle
        p_ = self.pSteeringRatio * delta_angle
        return p_

    def p_control_speed(self, pose: Pose, path: list[PoseStamped]):
        robot_x, robot_y = pose.position.x, pose.position.y
        try:
            goal_x, goal_y = path[1].pose.position.x, path[1].pose.position.y
        except:
            return 0.

        transformed_goal_x, transformed_goal_y = goal_x - robot_x, goal_y - robot_y
        rho, th = decart_to_polar(transformed_goal_x, transformed_goal_y)

        angle = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
        direction = abs(angle - th)
        if direction > 2.5:
            return -self.averageSpeed
        else:
            return self.averageSpeed


def main():
    rclpy.init()
    node = PathController("path_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
