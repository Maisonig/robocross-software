#!/usr/bin/env python3

import array
import rclpy
import numpy as np
import ros2_numpy as rnp

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan


def decart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan2(y, x)


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
        !! Без указания значений параметров при запуске ноды, она будет разрушаться.
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

        self.declare_parameter('pointcloud_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('scan_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('frequency', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('x_max', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('x_min', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('y_max', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('y_min', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('z_max', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('z_min', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('angle_max', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('angle_min', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('rays_number', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('frame_id', rclpy.Parameter.Type.STRING)

        params = [
            rclpy.Parameter('pointcloud_topic', rclpy.Parameter.Type.STRING, '/front_camera/points'),
            rclpy.Parameter('scan_topic', rclpy.Parameter.Type.STRING, '/front_camera/scan'),
            rclpy.Parameter('frequency', rclpy.Parameter.Type.INTEGER, 30),
            rclpy.Parameter('x_max', rclpy.Parameter.Type.DOUBLE, 10.),
            rclpy.Parameter('x_min', rclpy.Parameter.Type.DOUBLE, 0.6),
            rclpy.Parameter('y_max', rclpy.Parameter.Type.DOUBLE, 20.),
            rclpy.Parameter('y_min', rclpy.Parameter.Type.DOUBLE, -20.),
            rclpy.Parameter('z_max', rclpy.Parameter.Type.DOUBLE, 0.0),
            rclpy.Parameter('z_min', rclpy.Parameter.Type.DOUBLE, -0.1),
            rclpy.Parameter('angle_max', rclpy.Parameter.Type.DOUBLE, 0.785),
            rclpy.Parameter('angle_min', rclpy.Parameter.Type.DOUBLE, -0.785),
            rclpy.Parameter('rays_number', rclpy.Parameter.Type.INTEGER, 60),
            rclpy.Parameter('frame_id', rclpy.Parameter.Type.STRING, 'front_camera_link'),
        ]

        # self.set_parameters(params)

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
        try:
            data = rnp.numpify(self.pcData)
            data = np.array(data['xyz'], dtype=np.float32)
            condition = data[:, 2] < self.zMax
            data = data[condition]
            condition = data[:, 2] > self.zMin
            data = data[condition]
            condition = data[:, 0] < self.xMax
            data = data[condition]
            condition = data[:, 0] > self.xMin
            data = data[condition]
            for x, y, z in data:
                rad, a = decart_to_polar(x, y)
                i = np.where(self.scanData[:, 0] > a)[0][0]
                if self.scanData[i, 1] == 0:
                    self.scanData[i, 1] = rad
        except (ValueError, IndexError):
            pass

    def timer_callback(self):
        # start_time = time.time()
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
        # print(1.0 / (time.time() - start_time))


def main():
    rclpy.init()
    node = CloudToScan("cloud_to_scan")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
