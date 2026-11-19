"""Deterministic optical test scene; intrinsics are synthetic and test-only."""
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def scene(ids):
    image = np.full((480, 640), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
    for offset, tag in enumerate(ids):
        x = 100 + offset * 240
        image[160:320, x:x + 160] = draw(dictionary, tag, 160)
    return image


class TestCamera(Node):
    def __init__(self):
        super().__init__('test_camera')
        scenario = self.declare_parameter('scenario', 'two_tags').value
        scenarios = {'two_tags': [0, 1], 'one_tag': [0], 'blank': [], 'missing_info': [0],
                     'wrong_resolution': [0], 'stale': [0]}
        self.frame = scene(scenarios[scenario])
        self.scenario = scenario
        self.bridge = CvBridge()
        self.image = self.create_publisher(Image, '/camera/image_raw', 2)
        self.info = self.create_publisher(CameraInfo, '/camera/camera_info', 2)
        self.jpeg = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 2)
        self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        image = self.bridge.cv2_to_imgmsg(self.frame, 'mono8')
        image.header.stamp = self.get_clock().now().to_msg()
        if self.scenario == 'stale':
            image.header.stamp.sec -= 2
        image.header.frame_id = 'camera_optical_frame'
        if self.scenario != 'missing_info':
            info = CameraInfo(
                header=image.header, width=320 if self.scenario == 'wrong_resolution' else 640, height=480,
                distortion_model='plumb_bob', d=[0.] * 5,
                k=[500., 0., 320., 0., 500., 240., 0., 0., 1.],
                r=[1., 0., 0., 0., 1., 0., 0., 0., 1.],
                p=[500., 0., 320., 0., 0., 500., 240., 0., 0., 0., 1., 0.])
            self.info.publish(info)
        self.image.publish(image)
        jpeg = self.bridge.cv2_to_compressed_imgmsg(self.frame, dst_format='jpg')
        jpeg.header = image.header
        self.jpeg.publish(jpeg)


def main():
    rclpy.init()
    node = TestCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
