import array
import sys
import time

import cv2
import rclpy
import numpy as np
import depthai as dai
import ros2_numpy as rnp

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, PointField


class CameraDriver(Node):
    FPS = 30

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.declare_parameter('camera_id', rclpy.Parameter.Type.STRING)
        params = [rclpy.Parameter('camera_id', rclpy.Parameter.Type.STRING, '19443010C1354D1300')]
        self.set_parameters(params)

        self.declare_parameter('pointcloud_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('image_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('frequency', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('camera_frame', rclpy.Parameter.Type.STRING)
        params = [rclpy.Parameter('frequency', rclpy.Parameter.Type.DOUBLE, 0.067),
                  rclpy.Parameter('pointcloud_topic', rclpy.Parameter.Type.STRING, '/camera/points/raw'),
                  rclpy.Parameter('image_topic', rclpy.Parameter.Type.STRING, '/camera/image/color/raw'),
                  rclpy.Parameter('camera_frame', rclpy.Parameter.Type.STRING, 'camera_link')]
        self.set_parameters(params)

        self.pcPub = self.create_publisher(PointCloud2,
                                           self.get_parameter('pointcloud_topic').get_parameter_value().string_value,
                                           10)
        self.imagePub = self.create_publisher(Image,
                                              self.get_parameter('image_topic').get_parameter_value().string_value,
                                              10)
        self.mainTimer = self.create_timer(1 / self.get_parameter('frequency').get_parameter_value().double_value,
                                           self.run)
        self.cameraFrameId = self.get_parameter('camera_frame').get_parameter_value().string_value

        self.mxId = self.get_parameter('camera_id').get_parameter_value().string_value

        self.pipeline = dai.Pipeline()
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.depth = self.pipeline.create(dai.node.StereoDepth)
        self.pointCloud = self.pipeline.create(dai.node.PointCloud)
        self.sync = self.pipeline.create(dai.node.Sync)
        self.xOut = self.pipeline.create(dai.node.XLinkOut)
        self.xOut.input.setBlocking(False)

        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        self.camRgb.setIspScale(1, 3)
        self.camRgb.setFps(self.FPS)

        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoLeft.setCamera("left")
        self.monoLeft.setFps(self.FPS)
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoRight.setCamera("right")
        self.monoRight.setFps(self.FPS)

        self.depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        self.depth.setLeftRightCheck(True)
        self.depth.setExtendedDisparity(False)
        self.depth.setSubpixel(True)
        self.depth.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        self.monoLeft.out.link(self.depth.left)
        self.monoRight.out.link(self.depth.right)
        self.depth.depth.link(self.pointCloud.inputDepth)
        self.camRgb.isp.link(self.sync.inputs["rgb"])
        self.pointCloud.outputPointCloud.link(self.sync.inputs["pcl"])
        self.sync.out.link(self.xOut.input)
        self.xOut.setStreamName("out")

        devices = dai.Device.getAllConnectedDevices()
        for device in devices:
            if device.mxid == self.mxId:
                self.deviceInfo = dai.DeviceInfo(self.get_parameter('camera_id').get_parameter_value().string_value)
            else:
                self.connect()

    def array_to_pointcloud(self, np_array):
        msg = PointCloud2()
        msg.header.frame_id = self.cameraFrameId
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = np_array["xyz"].shape[0]
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = sys.byteorder != 'little'
        msg.is_dense = not np.isnan(np_array["xyz"]).any()
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        memory_view = memoryview(np_array["xyz"].astype(np.float32).tobytes())
        if memory_view.nbytes > 0:
            array_bytes = memory_view.cast("B")
        else:
            # Casting raises a TypeError if the array has no elements
            array_bytes = b""
        as_array = array.array("B")
        as_array.frombytes(array_bytes)
        msg.data = as_array

        return msg

    def connect(self):
        while True:
            devices = dai.Device.getAllConnectedDevices()
            for device in devices:
                if device.mxid == self.mxId:
                    self.deviceInfo = dai.DeviceInfo(self.mxId)
                    self.get_logger().info(f"Found required device (id: {self.mxId}). Connecting...")
                    return True
                self.get_logger().info(f"Found device (id: {self.mxId})")
            self.get_logger().error(f"Required device (id: {self.mxId}) disconnected! Reconnecting in 5 seconds...")
            time.sleep(5)

    def reconnect(self):
        self.get_logger().error(f"Device (id: {self.mxId}) crashed!")
        self.connect()

    def run(self):
        try:
            device = dai.Device(self.pipeline, self.deviceInfo, dai.UsbSpeed(dai.UsbSpeed.HIGH.value))
            queue = device.getOutputQueue(name="out", maxSize=4, blocking=False)
            while True:
                in_message = queue.get()
                in_color = in_message["rgb"]
                in_point_cloud = in_message["pcl"]
                bgr_frame = in_color.getCvFrame()
                rgb_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
                img_msg = rnp.image.numpy_to_image(rgb_frame, 'rgb8')
                self.imagePub.publish(img_msg)
                if in_point_cloud:
                    points = in_point_cloud.getPoints().astype(np.float32) / 1000
                    condition_matrix = points[:, 2] > 0
                    points = points[condition_matrix]
                    points[:, 0] = points[:, 0] * -1

                    data = np.zeros(points.shape[0], dtype=[('xyz', np.float32, (3,))])
                    data['xyz'] = points
                    pc_msg = self.array_to_pointcloud(data)
                    self.pcPub.publish(pc_msg)
        except RuntimeError:
            self.deviceInfo = None
            self.reconnect()


def main():
    rclpy.init()
    node = CameraDriver("camera_driver")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
