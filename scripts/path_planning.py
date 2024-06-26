#!/usr/bin/env python3
import time

import rclpy
import numpy as np
import ros2_numpy as rnp
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from math import asin, atan2, degrees
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from pathfinding import Grid, AstarFinder


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


def is_in_goal(current: Odometry, goal: PoseStamped, goal_rad: float):
    x1, y1 = current.pose.pose.position.x, current.pose.pose.position.y
    x2, y2 = goal.pose.position.x, goal.pose.position.y
    if x2 - goal_rad > x1 > x2 + goal_rad:
        if y2 - goal_rad > y1 > y2 + goal_rad:
            return True
    return False


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
        # Проверить задан ли пункт назначения
        if self.goalData != PoseStamped():
            # Проверить не находится ли робот в пункте назначения
            if not is_in_goal(self.odomData, self.goalData, self.goalRad):
                # Проверить получена ли карта навигации
                if self.mapData != OccupancyGrid():
                    # Получить глобальную сетку препятствий
                    g = np.array(rnp.numpify(self.mapData), np.uint8)
                    g[g == 255] = 127
                    g[g == 0] = 0
                    g[g == 100] = 255
                    self.grid.init_grid(g)
                    # Получить текущие координаты робота
                    x1 = int(g.shape[0] - g.shape[0] / 2 + self.odomData.pose.pose.position.x * 10)
                    y1 = int(g.shape[1] - g.shape[1] / 2 + self.odomData.pose.pose.position.y * 10)
                    a, b, th = quaternion_to_euler(self.odomData.pose.pose.orientation.x,
                                                   self.odomData.pose.pose.orientation.y,
                                                   self.odomData.pose.pose.orientation.z,
                                                   self.odomData.pose.pose.orientation.w, )
                    th = np.deg2rad(th)
                    # Получить координаты голевой позиции
                    x2 = int(g.shape[0] - g.shape[0] / 2 + self.goalData.pose.position.x * 10)
                    y2 = int(g.shape[1] - g.shape[1] / 2 + self.goalData.pose.position.y * 10)
                    # Поиск пути
                    st = time.time()
                    path = self.finder.get_path(self.grid, (x1, y1, th), (x2, y2))
                    # self.get_logger().info(str(time.time() - st))
                    # Если путь найден
                    if type(path) is list:
                        self.publish_path(path, g.shape)
                    else:
                        self.publish_path(None, g.shape)
                        self.get_logger().warn(path)
                else:
                    self.get_logger().warn('No global map detected!')
            else:
                self.get_logger().info("Goal reached!")
                self.publish_path(None, (0, 0))
        else:
            pass

    def publish_path(self, path, grid_shape):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.robotBaseFrame
        if path:
            for pose in path:
                p = PoseStamped()
                p.pose.position.x = float(pose[0]) * self.gridRes - grid_shape[0] * self.gridRes / 2
                p.pose.position.y = float(pose[1]) * self.gridRes - grid_shape[1] * self.gridRes / 2
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
