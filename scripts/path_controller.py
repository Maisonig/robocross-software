#!/usr/bin/env python3
import time
import rclpy
import numpy as np

from rclpy.node import Node
from math import sqrt, atan2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from rcl_interfaces.srv import GetParameters


class PathController(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('frequency', 30)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.001)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('average_speed', 1.5)
        self.declare_parameter('path_state_service_node', 'path_mapping')

        path_state_service = self.get_parameter('path_state_service_node').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value

        # PID parameters
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.integral = 0.0
        self.prev_error = 0.0

        self.averageSpeed = self.get_parameter('average_speed').get_parameter_value().double_value

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

        # Client to get path state and finder type from other node
        # Robot moves only if path state is True
        # Finder type determines the movement kinematic
        self.cli = self.create_client(GetParameters, f'/{path_state_service}/get_parameters')
        self.req = GetParameters.Request()
        # Timer for client requesting
        self.service_timer = self.create_timer(0.33, self.service_timer_callback)
        self.pathState = False
        self.req.names = ['path_state']
        # First request must be after path_state_service node initialization for time synchronization
        time.sleep(5)
        self.future = self.cli.call_async(self.req)

    def service_timer_callback(self):
        if not self.cli.wait_for_service(timeout_sec=2.0):
            self.pathState = False
            self.get_logger().info('service not available, waiting again...')
        else:
            if self.future.done():
                self.pathState = self.future.result().values[0].bool_value
                self.future.set_result(None)
                self.future = self.cli.call_async(self.req)

    def path_callback(self, msg):
        self.pathData = msg

    def odom_callback(self, msg):
        self.odomData = msg

    def get_closest_point(self):
        if not self.pathData or not self.odomData:
            return None

        min_dist = float('inf')
        closest_idx = 0
        current_x = self.odomData.pose.pose.position.x
        current_y = self.odomData.pose.pose.position.y

        for i, pose in enumerate(self.pathData.poses):
            dx = pose.pose.position.x - current_x
            dy = pose.pose.position.y - current_y
            dist = sqrt(dx ** 2 + dy ** 2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def pid_controller(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y ** 2 + q.z ** 2)
        return atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        if not self.pathData or not self.odomData:
            return

        if self.pathState:
            # Find target point
            closest_idx = self.get_closest_point()
            target_idx = min(closest_idx + 5, len(self.pathData.poses) - 1)
            target_pose = self.pathData.poses[target_idx].pose

            # Calculate position errors
            current_yaw = self.quaternion_to_yaw(self.odomData.pose.pose.orientation)
            dx = target_pose.position.x - self.odomData.pose.pose.position.x
            dy = target_pose.position.y - self.odomData.pose.pose.position.y
            target_yaw = atan2(dy, dx)

            # Calculate steering error
            error = target_yaw - current_yaw
            error = np.arctan2(np.sin(error), np.cos(error))  # Normalize to [-pi, pi]

            # Calculate steering command
            angular_z = self.pid_controller(error)

            # Publish command
            cmd = Twist()
            cmd.linear.x = 1.  # Constant speed
            cmd.angular.z = angular_z
        else:
            cmd = Twist()
        self.cmdPub.publish(cmd)


def main():
    rclpy.init()
    node = PathController("path_controller")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
