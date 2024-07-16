#!/usr/bin/env python3

import time
import array
import rclpy
import numpy as np
import ros2_numpy as rnp

from rclpy.node import Node
from utils import decart_to_polar
from sensor_msgs.msg import PointCloud2, LaserScan


class CloudToScan(Node):
    """
    Класс ноды, отвечающий за преобразование облака точек в линию точек.

    По заданным параметрам минимальных и максимальных значений координат трех осей отсекаются точки из облака,
    находящиеся за пределами диапазонов. Создается N - массив углов согласно количеству заданных лучей, минимальному
    углу и максимальному. Оставшиеся значения облака точек переводятся двумерные полярные координаты. Углы точек,
    соответствующие углам массива.

    Основные параметры класса для среды ROS2:
        * субскрайбер на топик облака точек;
        * паблишер топика лазерного сканирования;
        * таймер для выполнения обработки.

    Основные параметры ROS2:
        * pointcloud_topic - название топика облака точек;
        * scan_topic - название топика лазерного сканирования;
        * x_max - максимальная координата региона интереса в облаке точек по оси х;
        * x_min - минимальная координата региона интереса в облаке точек по оси х;
        * y_max - максимальная координата региона интереса в облаке точек по оси у;
        * y_min - минимальная координата региона интереса в облаке точек по оси у;
        * z_max - максимальная координата региона интереса в облаке точек по оси z;
        * z_min - минимальная координата региона интереса в облаке точек по оси z;
        * angle_max - верхний диагазон углов лазерного сканирования;
        * angle_min - нижний диапазон углов лазерного сканирования;
        * rays_number - число лучей (углов) лазерного сканирования;
        * frame_id - название звена лазера для привязки;
        * frequency - частота работы таймера.
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('pointcloud_topic', '/front_camera/points')
        self.declare_parameter('scan_topic', '/front_camera/scan')
        self.declare_parameter('frequency', 30)
        self.declare_parameter('x_max', 20.0)
        self.declare_parameter('x_min', 0.7)
        self.declare_parameter('y_max', 100.)
        self.declare_parameter('y_min', -100.)
        self.declare_parameter('z_max', 0.0)
        self.declare_parameter('z_min', -0.3)
        self.declare_parameter('angle_max', 0.7)
        self.declare_parameter('angle_min', -0.7)
        self.declare_parameter('rays_number', 300)
        self.declare_parameter('frame_id', 'front_camera_link')

        self.xMax = self.get_parameter('x_max').get_parameter_value().double_value
        self.xMin = self.get_parameter('x_min').get_parameter_value().double_value
        self.yMax = self.get_parameter('y_max').get_parameter_value().double_value
        self.yMin = self.get_parameter('y_min').get_parameter_value().double_value
        self.zMax = self.get_parameter('z_max').get_parameter_value().double_value
        self.zMin = self.get_parameter('z_min').get_parameter_value().double_value
        self.angleMax = self.get_parameter('angle_max').get_parameter_value().double_value
        self.angleMin = self.get_parameter('angle_min').get_parameter_value().double_value
        self.raysNum = self.get_parameter('rays_number').get_parameter_value().integer_value
        self.frameId = self.get_parameter('frame_id').get_parameter_value().string_value

        self.angleIncrement = (abs(self.angleMin) + abs(self.angleMax)) / self.raysNum

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        cloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value

        self.pcData = PointCloud2()
        self.pcSub = self.create_subscription(PointCloud2,
                                              cloud_topic,
                                              self.pc_callback,
                                              10)
        self.scanData = np.zeros([self.raysNum, 2], np.float32)

        angle = self.angleMin
        for i in range(self.raysNum):
            self.scanData[i, 0] = angle
            angle = angle + self.angleIncrement

        self.scanPub = self.create_publisher(LaserScan,
                                             scan_topic,
                                             10)
        self.scanTimer = self.create_timer(1 / frequency,
                                           self.timer_callback)

    def pc_callback(self, msg):
        self.pcData = msg

    def pointcloud_to_scan(self):
        data = rnp.numpify(self.pcData)
        data = np.array(data['xyz'], dtype=np.float32)
        condition = data[:, 2] < self.zMax
        data = data[condition]
        condition = data[:, 2] > self.zMin
        data = data[condition]
        condition = data[:, 1] < self.yMax
        data = data[condition]
        condition = data[:, 1] > self.yMin
        data = data[condition]
        condition = data[:, 0] < self.xMax
        data = data[condition]
        condition = data[:, 0] > self.xMin
        data = data[condition]
        for x, y, z in data:
            rad, a = decart_to_polar(x, y)
            try:
                i = np.where(self.scanData[:, 0] > a)[0][0]
                if self.scanData[i, 1] == 0:
                    self.scanData[i, 1] = rad
            except IndexError:
                pass

    def timer_callback(self):
        if self.pcData != PointCloud2():
            self.pointcloud_to_scan()
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frameId
            msg.angle_min = self.angleMin
            msg.angle_max = self.angleMax
            msg.angle_increment = self.angleIncrement
            msg.range_min = self.xMin
            msg.range_max = self.xMax
            msg.ranges = array.array('f', list(self.scanData[:, 1]))
            self.scanPub.publish(msg)
            self.scanData[:, 1] = 0

    def control_message(self, msg):
        if self.get_clock().now().to_msg().sec - 3 > msg.header.stamp.sec:
            self.get_logger().warn(f"Dropping the message {type(msg).__name__} because the data is too old."
                                   f" The last message has a stamp {msg.header.stamp.sec}.")
            time.sleep(1)
        else:
            return True


def main():
    rclpy.init()
    node = CloudToScan("cloud_to_scan")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
