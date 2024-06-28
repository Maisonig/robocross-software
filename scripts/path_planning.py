#!/usr/bin/env python3

import time
import rclpy
import numpy as np
import ros2_numpy as rnp

from rclpy.node import Node
from utils import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from pathfinding import Grid, AstarFinder
from nav_msgs.msg import OccupancyGrid, Odometry, Path


def is_in_goal(current: Odometry, goal: PoseStamped, goal_rad: float):
    x1, y1 = current.pose.pose.position.x, current.pose.pose.position.y
    x2, y2 = goal.pose.position.x, goal.pose.position.y
    if x2 - goal_rad > x1 > x2 + goal_rad:
        if y2 - goal_rad > y1 > y2 + goal_rad:
            return True
    return False


def remap_robot_coord(current: Odometry, map_array: np.ndarray, map_resolution: float):
    x = int(map_array.shape[0] - map_array.shape[0] / 2 + current.pose.pose.position.x / map_resolution)
    y = int(map_array.shape[1] - map_array.shape[1] / 2 + current.pose.pose.position.y / map_resolution)
    a, b, th = euler_from_quaternion(current.pose.pose.orientation.x,
                                     current.pose.pose.orientation.y,
                                     current.pose.pose.orientation.z,
                                     current.pose.pose.orientation.w, )
    return x, y, th


def remap_goal_coord(goal: PoseStamped, map_array: np.ndarray, map_resolution: float):
    x = int(map_array.shape[0] - map_array.shape[0] / 2 + goal.pose.position.x / map_resolution)
    y = int(map_array.shape[1] - map_array.shape[1] / 2 + goal.pose.position.y / map_resolution)
    return x, y


class PathPlanner(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('frequency', 30)
        self.declare_parameter('global_grid_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('robot_base_frame', 'map')
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('robot_collision_radius', 1.0)
        self.declare_parameter('goal_radius', 3.)
        self.declare_parameter('steering_value', 0.12)
        self.declare_parameter('path_discrete', 0.5)
        self.declare_parameter('timeout', 1.0)

        global_grid_topic = self.get_parameter('global_grid_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value

        self.robotBaseFrame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.gridRes = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.robotCollisionRad = self.get_parameter('robot_collision_radius').get_parameter_value().double_value
        self.goalRad = self.get_parameter('goal_radius').get_parameter_value().double_value
        self.steeringVal = self.get_parameter('steering_value').get_parameter_value().double_value
        self.pathDiscrete = self.get_parameter('path_discrete').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        self.globalMapSub = self.create_subscription(OccupancyGrid,
                                                     global_grid_topic,
                                                     self.global_map_callback,
                                                     10)
        self.odomSub = self.create_subscription(Odometry,
                                                odom_topic,
                                                self.odom_callback,
                                                10)
        self.goalSub = self.create_subscription(PoseStamped,
                                                goal_topic,
                                                self.goal_callback,
                                                10)
        self.pathPub = self.create_publisher(Path,
                                             path_topic,
                                             10)

        self.mainTimer = self.create_timer(1 / frequency, self.timer_callback)

        self.odomData = Odometry()
        self.mapData = OccupancyGrid()
        self.goalData = PoseStamped()

        self.grid = Grid(np.zeros((1, 1)), self.steeringVal, self.pathDiscrete / self.gridRes)
        self.finder = AstarFinder(self.robotCollisionRad / self.gridRes,
                                  self.timeout,
                                  self.goalRad / self.gridRes)

    def global_map_callback(self, msg):
        self.mapData = msg

    def odom_callback(self, msg):
        self.odomData = msg

    def goal_callback(self, msg):
        self.goalData = msg

    def timer_callback(self):
        # Check that the destination has been received
        if self.goalData != PoseStamped():
            # Check that the robot is not at the destination
            if not is_in_goal(self.odomData, self.goalData, self.goalRad):
                # Check that the occupancy grid has been received
                if self.mapData != OccupancyGrid():
                    # Convert the occupancy grid to a numpy array
                    map_array = np.array(rnp.numpify(self.mapData), np.uint8)
                    map_array[map_array == 100] = 255
                    # Refresh grid for finder
                    self.grid.init_grid(map_array)
                    # Convert the odometry to array coordinates
                    x1, y1, th1 = remap_robot_coord(self.odomData, map_array, self.gridRes)
                    # Convert the goal pose to array coordinates
                    x2, y2 = remap_goal_coord(self.goalData, map_array, self.gridRes)
                    # Search the path by hybrid astar
                    path = self.finder.get_path(self.grid, (x1, y1, th1), (x2, y2))
                    # If the path exist
                    if type(path) is list:
                        # Publish the path in the ros2 topic
                        self.publish_path(path, map_array.shape)
                        # self.get_logger().info(f"{th1}:::::::{path[0][2]}")
                    else:
                        # Publish an empty path in the ros2 topic
                        self.publish_path(None, map_array.shape)
                        # Log the error from finder algorithm
                        self.get_logger().warn(path)
                else:
                    self.get_logger().warn('No global map detected!')

    def publish_path(self, path, map_shape):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robotBaseFrame
        if path:
            for pose in path:
                p = PoseStamped()
                p.pose.position.x = float(pose[0]) * self.gridRes - map_shape[0] * self.gridRes / 2
                p.pose.position.y = float(pose[1]) * self.gridRes - map_shape[1] * self.gridRes / 2
                p.pose.orientation.z = float(pose[2])
                msg.poses.append(p)
        self.pathPub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner("path_planning")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
