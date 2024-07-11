#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from nav_msgs.msg import Odometry


def is_in_goal(current: Odometry, goal: PoseStamped, goal_rad: float):
    x1, y1 = current.pose.pose.position.x, current.pose.pose.position.y
    x2, y2 = goal.pose.position.x, goal.pose.position.y
    if x2 - goal_rad < x1 < x2 + goal_rad:
        if y2 - goal_rad < y1 < y2 + goal_rad:
            return True
    return False


class WaypointFollowing(Node):

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('frequency', 30)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('goal_radius', 5.0)
        self.declare_parameter('waypoints', [-5., -5., 0.,
                                             20.6453, -9.80267, 0.,
                                             24.3003, -31.9225, 0.,
                                             13.7962, -46.5684, 0.,
                                             -3.74152, -51.6982, 0.,
                                             -0.0843143, -0.171923, 0.])
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('start_following', False)

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        points = self.get_parameter('waypoints').get_parameter_value().double_array_value
        self.mapFrame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.goalRad = self.get_parameter('goal_radius').get_parameter_value().double_value
        self.isStart = False
        self.waypoints = []
        j = 0
        for i in range(int(len(points) / 3)):
            self.waypoints.append([points[j], points[j + 1], points[j + 2]])
            j += 3

        self.odomSub = self.create_subscription(Odometry,
                                                odom_topic,
                                                self.odom_callback,
                                                10)
        self.goalPub = self.create_publisher(PoseStamped,
                                             goal_topic,
                                             10)

        self.mainTimer = self.create_timer(1 / frequency, self.timer_callback)

        self.odomData = Odometry()
        self.goalData = PoseStamped()

        self.pubFlg = False

    def odom_callback(self, msg):
        self.odomData = msg

    def goal_callback(self, msg):
        self.goalData = msg

    def timer_callback(self):
        if self.isStart:
            if not self.pubFlg:
                self.goalData.header.stamp = self.get_clock().now().to_msg()
                self.goalData.header.frame_id = self.mapFrame
                self.goalData.pose.position.x = self.waypoints[0][0]
                self.goalData.pose.position.y = self.waypoints[0][1]
                self.goalData.pose.position.z = self.waypoints[0][2]
                self.goalPub.publish(self.goalData)
                self.pubFlg = True
            else:
                if is_in_goal(self.odomData, self.goalData, self.goalRad):
                    self.get_logger().info("Goal reached!")
                    self.pubFlg = False
                    self.waypoints.pop(0)
                    if len(self.waypoints) == 0:
                        self.get_logger().info("All waypoints reached!")
                        self.destroy_node()
                        exit()
        else:
            self.isStart = self.get_parameter('start_following').get_parameter_value().bool_value
            if self.isStart:
                self.get_logger().info('Starting waypoint following')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowing("waypoint_following")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
