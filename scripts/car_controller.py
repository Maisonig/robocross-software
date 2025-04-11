#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from math import atan2, hypot, pi, cos, sin
import numpy as np
from simple_pid import PID


class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')

        # Параметры
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('wheelbase', 2.5),
                                    ('max_steer_angle', 0.6),
                                    ('lookahead_distance', 3.0),
                                    ('target_speed', 1.0),
                                    ('pid_steer', [0.8, 0.001, 0.1]),
                                    ('reverse_threshold', 0.5),
                                ])

        # Константы
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steer = self.get_parameter('max_steer_angle').value
        self.lookahead = self.get_parameter('lookahead_distance').value
        self.target_speed = self.get_parameter('target_speed').value

        # PID регуляторы
        self.steer_pid = PID(*self.get_parameter('pid_steer').value)
        self.steer_pid.output_limits = (-self.max_steer, self.max_steer)

        # Состояние
        self.current_pose = None
        self.path = None
        self.reverse_mode = False
        self.current_target_idx = 0

        # Подписки и публикации
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg):
        self.path = msg.poses
        self.current_target_idx = 0
        self.get_logger().info('New path received with {} points'.format(len(self.path)))

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y ** 2 + q.z ** 2)
        return atan2(siny_cosp, cosy_cosp)

    def find_target_point(self):
        if not self.path or not self.current_pose:
            return None

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        # Поиск ближайшей точки
        distances = [hypot(p.pose.position.x - current_x,
                           p.pose.position.y - current_y)
                     for p in self.path]
        closest_idx = np.argmin(distances)

        # Выбор точки с упреждением
        target_idx = min(closest_idx + int(self.lookahead / 0.1), len(self.path) - 1)
        return self.path[target_idx].pose.position

    def calculate_errors(self, target_pos):
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        dx = target_pos.x - self.current_pose.position.x
        dy = target_pos.y - self.current_pose.position.y

        # Переход в систему координат автомобиля
        rotated_x = dx * cos(current_yaw) + dy * sin(current_yaw)
        rotated_y = -dx * sin(current_yaw) + dy * cos(current_yaw)

        # Ошибка положения и угла
        cte = rotated_y
        heading_error = atan2(rotated_y, rotated_x)

        return cte, heading_error

    def check_reverse_condition(self, target_pos):
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        dx = target_pos.x - self.current_pose.position.x
        dy = target_pos.y - self.current_pose.position.y
        target_angle = atan2(dy, dx)

        angle_diff = abs(target_angle - current_yaw)
        if angle_diff > pi / 2 + self.get_parameter('reverse_threshold').value:
            return True
        return False

    def control_loop(self):
        if not self.path or not self.current_pose:
            return

        target_pos = self.find_target_point()
        if not target_pos:
            return

        # Проверка необходимости реверса
        self.reverse_mode = self.check_reverse_condition(target_pos)

        # Расчет ошибок
        cte, heading_error = self.calculate_errors(target_pos)

        # ПИД регулятор для угла поворота
        steer_angle = self.steer_pid(heading_error + cte * 0.1)

        # Формирование сообщения
        cmd = Twist()
        cmd.linear.x = self.target_speed * (-1 if self.reverse_mode else 1)
        cmd.angular.z = steer_angle if self.reverse_mode else -steer_angle

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = CarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()