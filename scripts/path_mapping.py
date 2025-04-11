#!/usr/bin/env python3
import time
from zlib import compress

import cv2
import rclpy
import ros2_numpy as rnp
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from util.utils import *
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, CompressedImage
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import PoseStamped
from util.astar import AstarGrid, AstarFinder
from util.hybrid_astar import HybridAstarGrid, HybridAstarFinder
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path


def is_in_goal(current: Odometry, goal: PoseStamped, goal_rad: float):
    x1, y1 = current.pose.pose.position.x, current.pose.pose.position.y
    x2, y2 = goal.pose.position.x, goal.pose.position.y
    # z1 = euler_from_quaternion(current.pose.pose.orientation.x,
    #                            current.pose.pose.orientation.y,
    #                            current.pose.pose.orientation.z,
    #                            current.pose.pose.orientation.w)[2]
    # z2 = euler_from_quaternion(goal.pose.orientation.x,
    #                            goal.pose.orientation.y,
    #                            goal.pose.orientation.z,
    #                            goal.pose.orientation.w)[2]
    if x2 - goal_rad < x1 < x2 + goal_rad:
        if y2 - goal_rad < y1 < y2 + goal_rad:
            # dz = abs(z2 - z1)
            # if dz < 0.05:
            return True
    return False


def remap_robot_coord(current: Odometry, map_array: np.ndarray, map_resolution: float):
    x = int(map_array.shape[0] - map_array.shape[0] / 2 + current.pose.pose.position.x / map_resolution)
    y = int(map_array.shape[1] - map_array.shape[1] / 2 + current.pose.pose.position.y / map_resolution)
    a, b, c = euler_from_quaternion(current.pose.pose.orientation.x,
                                     current.pose.pose.orientation.y,
                                     current.pose.pose.orientation.z,
                                     current.pose.pose.orientation.w, )
    return x, y, c


def remap_goal_coord(goal: PoseStamped, map_array: np.ndarray, map_resolution: float):
    x = int(map_array.shape[0] - map_array.shape[0] / 2 + goal.pose.position.x / map_resolution)
    y = int(map_array.shape[1] - map_array.shape[1] / 2 + goal.pose.position.y / map_resolution)
    a, b, c = euler_from_quaternion(goal.pose.orientation.x,
                                     goal.pose.orientation.y,
                                     goal.pose.orientation.z,
                                     goal.pose.orientation.w)
    return x, y, c


class PathMapping(Node):

    UNKNOWN_CELL = 255
    FREE_CELL = 0
    OBSTACLE_CELL = 100

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('frequency', 30)

        # In this section, the topics that the node subscribes to receive lidar data
        self.declare_parameter('front_scan_topic', '/front_camera/scan/scan')
        self.declare_parameter('rear_scan_topic', '/rear_camera/scan/scan')
        self.declare_parameter('left_scan_topic', '/left_camera/scan/scan')
        self.declare_parameter('right_scan_topic', '/right_camera/scan/scan')

        # In this section, the topics that the node subscribes to receive environmental recognition data
        self.declare_parameter('detections_topic', '/detections')

        # This section defines the parameters of the lidar location relative to the robot in the format [x, y, theta], meters, radians
        self.declare_parameter('front_scan_position', [0.3, 0.0, 0.0])
        self.declare_parameter('rear_scan_position', [-2.2, 0.0, 3.14])
        self.declare_parameter('left_scan_position', [0.0, 2.2, 1.57])
        self.declare_parameter('right_scan_position', [0.0, -2.2, -1.57])

        # In this section, the topics to publish node messages such as:
        # - "OccupancyGrid" message for a global environment map with the dimensions specified in the <global_map_size> parameter;
        self.declare_parameter('global_map_topic', '/map')
        # - "OccupancyGrid" message for the local environment map located around the robot, with the dimensions specified in the <local_map_size> parameter;
        self.declare_parameter('local_map_topic', '/local_map')
        # - "Path" message of the robot's route.
        self.declare_parameter('path_topic', '/path')

        # In this section, the topics that the node subscribes to get odometry and a goal destination
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('goal_topic', '/goal_pose')

        # In this section, the names of the frames are assigned:
        # - <map_frame> to bind the global environment map
        self.declare_parameter('map_frame', 'map')
        # - <odom_frame> for linking with a global map frame
        self.declare_parameter('odom_frame', 'odom')
        # - <path_base_frame> the frame relative to which the route will be calculated, usually starting from the zero point of the robot's start, that is, <map_frame>
        self.declare_parameter('path_base_frame', 'map')

        # This section describes the parameters of the environment maps. The resolution for both maps is the same.
        self.declare_parameter('global_map_size', 1500) # The size of the global environment map, meters
        self.declare_parameter('local_map_size', 60) # Size of the local environment map, meters
        self.declare_parameter('map_infiltration_radius', 2.5) # A parameter that defines the dangerous approach zones around each obstacle, meters
        self.declare_parameter('map_resolution', 0.1) # By default, the value 1.0 is equal to a map with a resolution of 1 meter. Accordingly, 0.1 is a map with a resolution of 1 decimeter.
        self.declare_parameter('publish_global_map', False) # If false, the map will be published 1 time at the start, if true, the map will be published with a frequency <frequency>

        # This section describes the parameters of the route planner to the goal destination
        self.declare_parameter('finder_type','hybrid_astar') # Type of path planner (astar, hybrid_astar, omni_hybrid_astar), description of the planners on the gitHub repository page
        self.declare_parameter('path_collision_radius', 1.5) # Simplified collision avoidance model, the parameter defines the radius of the circumscription circle around the robot, meters
        self.declare_parameter('goal_radius', 0.5) # The radius of the circle that will be considered the area of reaching the goal point must be 2 or more times larger than <path_discrete>, otherwise there may be problems with building a path, meters
        self.declare_parameter('path_discrete', 0.5) # The discreteness value of the path, the minimum value of the distance between two points of the constructed path, meters
        self.declare_parameter('steering_value', 0.33) # Maximum steering angle for the hybrid_astar planner, radians
        self.declare_parameter('timeout', 1.0) # The maximum time to build a path, if the planner does not have time, the path will not be published

        self.declare_parameter('path_state', False) # The path state, if true, path is reached. Parameter service for other nodes

        # Getting the parameters needed only during initialization
        freq = self.get_parameter('frequency').get_parameter_value().integer_value

        front_scan_topic = self.get_parameter('front_scan_topic').get_parameter_value().string_value
        rear_scan_topic = self.get_parameter('rear_scan_topic').get_parameter_value().string_value
        left_scan_topic = self.get_parameter('left_scan_topic').get_parameter_value().string_value
        right_scan_topic = self.get_parameter('right_scan_topic').get_parameter_value().string_value

        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value

        global_map_topic = self.get_parameter('global_map_topic').get_parameter_value().string_value
        local_map_topic = self.get_parameter('local_map_topic').get_parameter_value().string_value
        path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        goal_topic = self.get_parameter('goal_topic').get_parameter_value().string_value

        # Getting parameters
        self.mapFrame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.odomFrame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.pathBaseFrame = self.get_parameter('path_base_frame').get_parameter_value().string_value

        self.frontScanPos = self.get_parameter('front_scan_position').get_parameter_value().double_array_value
        self.rearScanPos = self.get_parameter('rear_scan_position').get_parameter_value().double_array_value
        self.leftScanPos = self.get_parameter('left_scan_position').get_parameter_value().double_array_value
        self.rightScanPos = self.get_parameter('right_scan_position').get_parameter_value().double_array_value

        self.globalMapSize = self.get_parameter('global_map_size').get_parameter_value().integer_value
        self.localMapSize = self.get_parameter('local_map_size').get_parameter_value().integer_value
        self.mapInfRadius = self.get_parameter('map_infiltration_radius').get_parameter_value().double_value
        self.mapRes = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.pubGlobalMap = self.get_parameter('publish_global_map').get_parameter_value().bool_value

        self.pathCollisionRad = self.get_parameter('path_collision_radius').get_parameter_value().double_value
        self.goalRad = self.get_parameter('goal_radius').get_parameter_value().double_value
        self.steeringVal = self.get_parameter('steering_value').get_parameter_value().double_value
        self.pathDiscrete = self.get_parameter('path_discrete').get_parameter_value().double_value
        self.finderType = self.get_parameter('finder_type').get_parameter_value().string_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Main timer for mapping and path planning
        self.mainTimer = self.create_timer(1 / freq, self.main_timer_callback)

        # Publishers
        self.globalMapPub = self.create_publisher(CompressedImage, global_map_topic, 10)
        self.localMapPub = self.create_publisher(OccupancyGrid, local_map_topic, 10)

        # Subscribers
        self.frontScanSub = self.create_subscription(LaserScan, front_scan_topic, self.front_scan_callback, 10)
        self.rearScanSub = self.create_subscription(LaserScan, rear_scan_topic, self.rear_scan_callback, 10)
        self.leftScanSub = self.create_subscription(LaserScan, left_scan_topic, self.left_scan_callback, 10)
        self.rightScanSub = self.create_subscription(LaserScan, right_scan_topic, self.right_scan_callback, 10)

        self.detectionsSub = self.create_subscription(LaserScan, detections_topic, self.detections_callback, 10)

        self.odomSub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.goalSub = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)
        self.pathPub = self.create_publisher(Path, path_topic, 10)

        # Transform broadcaster
        self.tfBroadcaster = StaticTransformBroadcaster(self)

        # Empty msgs initialization
        self.odomData = Odometry()
        self.goalData = None  # PoseStamped()

        self.frontScanData = LaserScan()
        self.rearScanData = LaserScan()
        self.leftScanData = LaserScan()
        self.rightScanData = LaserScan()

        self.detectionsMsg = LaserScan()

        self.globalMapData = OccupancyGrid()
        self.localMapData = OccupancyGrid()

        #
        self.globalMapArray = np.zeros([int(self.globalMapSize / self.mapRes), int(self.globalMapSize / self.mapRes)], np.uint8)
        self.astar_grid = AstarGrid(np.zeros((1, 1)), self.pathDiscrete / self.mapRes)
        self.astar_finder = AstarFinder(self.pathCollisionRad / self.mapRes,
                                  self.timeout,
                                  self.goalRad / self.mapRes)

        self.hybridAstarGrid = HybridAstarGrid(np.zeros((1, 1)), self.steeringVal, self.pathDiscrete / self.mapRes)
        self.hybridAstarFinder = HybridAstarFinder(self.pathCollisionRad / self.mapRes,
                                                   self.timeout,
                                                   self.goalRad / self.mapRes)


    def goal_callback(self, msg):
        self.get_logger().info(f"\n Got goal pose: \n stamp: {round(msg.header.stamp.sec)}"
                               f"\n pose: {msg.pose.position}"
                               f"\n orientation: {msg.pose.orientation}")
        self.goalData = msg

    def odom_callback(self, msg):
        self.odomData = msg

    def front_scan_callback(self, msg):
        self.frontScanData = msg

    def rear_scan_callback(self, msg):
        self.rearScanData = msg

    def left_scan_callback(self, msg):
        self.leftScanData = msg

    def right_scan_callback(self, msg):
        self.rightScanData = msg

    def detections_callback(self, msg):
        self.detectionsMsg = msg

    def check_goal(self):
        if not is_in_goal(self.odomData, self.goalData, self.goalRad):
            path_state = [rclpy.parameter.Parameter('path_state', rclpy.parameter.Parameter.Type.BOOL, True)]
            self.set_parameters(path_state)
            return True
        else:
            path_state = [rclpy.parameter.Parameter('path_state', rclpy.parameter.Parameter.Type.BOOL, False)]
            self.get_logger().info("Goal Destination is reached")
            self.goalData = None
            self.set_parameters(path_state)
            return False

    def calc_path(self, f_type: str, robot_coord: tuple[int, int, any], goal_coord: tuple[int, int, any]):

        if self.check_goal():
            map_array = np.copy(self.globalMapArray)
            path = None

            if f_type == "astar":
                self.astar_grid.init_grid(map_array)
                path = self.astar_finder.get_path(self.astar_grid, robot_coord, goal_coord)
            if f_type == "hybrid_astar":
                self.hybridAstarGrid.init_grid(map_array)
                path = self.hybridAstarFinder.get_path(self.hybridAstarGrid, robot_coord, goal_coord)



            # If the planner returns the path, then there are no errors, return the path.
            if type(path) is list:
                return path
            # Else, the planner sends an error resulting from the construction of the path.
            else:
                self.get_logger().warn(path)


    def main_timer_callback(self):
        # Get finder type name
        finder_type = self.get_parameter('finder_type').get_parameter_value().string_value
        # Calc robot coord relative to the coordinate axis of the ndarray numpy array
        remapped_robot_coord = remap_robot_coord(self.odomData, self.globalMapArray, self.mapRes)

        # Set scan data on map
        # self.set_scan(self.frontScanData, self.frontScanPos)
        # self.set_scan(self.rearScanData, self.rearScanPos)
        # self.set_scan(self.leftScanData, self.leftScanPos)
        # self.set_scan(self.rightScanData, self.rightScanPos)
        # self.set_obstacles(remapped_robot_coord[0], remapped_robot_coord[1])

        # Send map frame static transform
        self.set_map_frame()
        # Send local map to topic
        # self.publish_local_map(remapped_robot_coord[0], remapped_robot_coord[1])
        # Send global map to topic if flag is enable
        if not self.pubGlobalMap:
            a = time.time()
            self.publish_global_map()
            v = (time.time() - a)
            print(v)

        if self.goalData:
            # Calc goal coord relative to the coordinate axis of the ndarray numpy array
            remapped_goal_coord = remap_goal_coord(self.goalData, self.globalMapArray, self.mapRes)
            # Calc path
            path = self.calc_path(finder_type, remapped_robot_coord, remapped_goal_coord)

            if path:
                self.publish_path(path, self.globalMapArray.shape)

    def publish_path(self, path, map_shape):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.pathBaseFrame
        for pose in path:
            p = PoseStamped()
            p.pose.position.x = float(pose[0]) * self.mapRes - map_shape[0] * self.mapRes / 2
            p.pose.position.y = float(pose[1]) * self.mapRes - map_shape[1] * self.mapRes / 2
            if len(pose) == 3:
                q = quaternion_from_euler(0, 0, pose[2])
                p.pose.orientation.x = q[0]
                p.pose.orientation.y = q[1]
                p.pose.orientation.z = q[2]
                p.pose.orientation.w = q[3]
            msg.poses.append(p)
        self.pathPub.publish(msg)

    def set_obstacles(self, x, y):
        size = int(self.localMapSize / 2 / self.mapRes)
        indexes = np.where(self.globalMapArray[y - size:y + size, x - size:x + size] == 255)
        ys, xs = indexes[0], indexes[1]
        for a, b in zip(xs, ys):
            cv2.circle(self.globalMapArray[y - size:y + size, x - size:x + size], [a, b], int(self.mapInfRadius / self.mapRes), [70], -1)
        for a, b in zip(xs, ys):
            try:
                self.globalMapArray[y - size:y + size, x - size:x + size][b, a] = 255
            except IndexError:
                self.get_logger().warn('Robot sensor vision is out of global map bounds')
                break

    def set_scan(self, scan_msg, scan_pos):
        if scan_msg != LaserScan() and self.odomData != Odometry():
            map_center = self.globalMapSize / 2 / self.mapRes
            rho, th = decart_to_polar(scan_pos[0] / self.mapRes, scan_pos[1] / self.mapRes)
            q = self.odomData.pose.pose.orientation
            base_th = euler_from_quaternion(q.x, q.y, q.z, q.w)[2]
            sensor_x, sensor_y = polar_to_decart(rho, base_th + scan_pos[2])
            base_x = int(map_center + self.odomData.pose.pose.position.x / self.mapRes + sensor_x)
            base_y = int(map_center + self.odomData.pose.pose.position.y / self.mapRes + sensor_y)
            base_yaw = base_th + scan_pos[2]
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
                    cv2.line(self.globalMapArray, [base_x, base_y], [x, y], [self.FREE_CELL], 1)
            for rho, phi in zip(non_empty_ranges, non_empty_angles):
                x, y = polar_to_decart(rho / self.mapRes, phi)
                x = int(x + base_x)
                y = int(y + base_y)
                cv2.line(self.globalMapArray, [base_x, base_y], [x, y], [self.FREE_CELL], 1)
                try:
                    self.globalMapArray[y, x] = 255
                except IndexError:
                    self.get_logger().warn('Robot sensor vision is out of global map bounds')
                    break

    def set_map_frame(self):
        t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.mapFrame
        t.child_frame_id = self.odomFrame
        t.transform.translation.z = 0.7
        self.tfBroadcaster.sendTransform(t)

    def publish_global_map(self):
        oc_array = np.copy(self.globalMapArray)
        oc_array[oc_array == 255] = self.OBSTACLE_CELL
        oc_array = np.array(oc_array, np.int8)
        grid = rnp.msgify(OccupancyGrid, oc_array)
        # g = oc_array.tobytes()
        # compressed = compress(g)
        # compressed = list(np.frombuffer(compressed, dtype=np.int8))
        # grid = OccupancyGrid()
        # grid.data = compressed
        # grid.info.width = oc_array.shape[0]
        # grid.info.height = oc_array.shape[1]
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.mapFrame
        grid.info.resolution = self.mapRes
        grid.info.origin.position.x = -self.globalMapSize / 2
        grid.info.origin.position.y = -self.globalMapSize / 2
        self.globalMapPub.publish(grid)

    def publish_local_map(self, robot_x, robot_y):
        oc_array = np.copy(self.globalMapArray)
        size = int(self.localMapSize / 2 / self.mapRes)
        oc_array = oc_array[robot_y - size:robot_y + size, robot_x - size:robot_x + size]

        oc_array[oc_array == 255] = self.OBSTACLE_CELL

        oc_array = np.array(oc_array, np.int8)
        grid = rnp.msgify(OccupancyGrid, oc_array)
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.odomFrame
        grid.info.resolution = self.mapRes
        grid.info.origin.position.x = self.odomData.pose.pose.position.x - size * self.mapRes
        grid.info.origin.position.y = self.odomData.pose.pose.position.y - size * self.mapRes
        # grid.info.origin.position.z = -0.7
        self.localMapPub.publish(grid)

def main():
    rclpy.init()
    node = PathMapping("path_mapping")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
