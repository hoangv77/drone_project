"""Gate AprilTag input on synchronized camera calibration and fresh image time."""
from copy import deepcopy
import time

from apriltag_msgs.msg import AprilTagDetectionArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from .calibration import calibration_error, stamp_seconds


class CameraGate(Node):
    def __init__(self):
        super().__init__('camera_gate')
        self.optical = self.declare_parameter('optical_frame', 'camera_optical_frame').value
        self.body = self.declare_parameter('body_frame', 'base_link').value
        self.max_age = self.declare_parameter('max_image_age_s', 0.5).value
        self.timeout = self.declare_parameter('disconnect_timeout_s', 2.0).value
        if not self.optical or not self.body or self.optical == self.body or min(self.max_age, self.timeout) <= 0:
            raise ValueError('Expected distinct frame names and positive timeouts')
        image_topic = self.declare_parameter('image_topic', '/camera/image_raw').value
        info_topic = self.declare_parameter('info_topic', '/camera/camera_info').value
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.image_input = message_filters.Subscriber(
            self, Image, image_topic, qos_profile=qos_profile_sensor_data)
        self.info_input = message_filters.Subscriber(
            self, CameraInfo, info_topic, qos_profile=qos_profile_sensor_data)
        self.image_input.registerCallback(self.received)
        self.sync = message_filters.TimeSynchronizer([self.image_input, self.info_input], 4)
        self.sync.registerCallback(self.paired)
        self.rectified = self.create_publisher(Image, '/vision/image_rect', 2)
        self.camera_info = self.create_publisher(CameraInfo, '/vision/camera_info', 2)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(AprilTagDetectionArray, '/vision/detections', self.detected, 10)
        self.last_image = self.last_valid = self.last_detection = None
        self.frame_count = self.detection_count = 0
        self.resolution = ''
        self.last_rate_at = time.monotonic()
        self.last_header = None
        self.latency = None
        self.error = ''
        self.map_key = None
        self.create_timer(1.0, self.report, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))

    def received(self, image):
        self.last_image = time.monotonic()
        self.frame_count += 1
        self.resolution = f'{image.width}x{image.height}'

    def paired(self, image, info):
        stamp = stamp_seconds(image.header.stamp)
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        self.error = calibration_error(image, info, self.optical)
        if not self.error and (stamp <= 0 or not -0.05 <= age <= self.max_age):
            self.error = 'Stale or future-dated image stamp'
        if self.error:
            return
        try:
            key = (image.width, image.height, info.distortion_model, *info.k, *info.d, *info.r, *info.p)
            if key != self.map_key:
                k = np.array(info.k).reshape(3, 3)
                r = np.array(info.r).reshape(3, 3)
                p = np.array(info.p).reshape(3, 4)[:, :3]
                remap = (cv2.fisheye.initUndistortRectifyMap
                         if info.distortion_model == 'equidistant' else cv2.initUndistortRectifyMap)
                self.maps = remap(k, np.array(info.d), r, p, (image.width, image.height), cv2.CV_16SC2)
                self.map_key = key
            pixels = self.bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
            rect = self.bridge.cv2_to_imgmsg(cv2.remap(pixels, *self.maps, cv2.INTER_LINEAR), 'mono8')
        except (cv2.error, CvBridgeError, ValueError) as exc:
            self.error = f'Rectification failed: {exc}'
            return
        rect.header = image.header
        rect_info = deepcopy(info)
        rect_info.k = np.array(info.p).reshape(3, 4)[:, :3].flatten().tolist()
        rect_info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        rect_info.d = [0.] * 5
        rect_info.distortion_model = 'plumb_bob'
        self.camera_info.publish(rect_info)
        self.rectified.publish(rect)
        self.last_header = rect.header
        self.last_valid = time.monotonic()

    def detected(self, array):
        self.last_detection = time.monotonic()
        self.detection_count = len(array.detections)
        self.latency = self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(array.header.stamp)

    def report(self):
        now = time.monotonic()
        connected = self.last_image is not None and now - self.last_image <= self.timeout
        calibrated = self.last_valid is not None and now - self.last_valid <= self.timeout and not self.error
        detecting = self.last_detection is not None and now - self.last_detection <= self.timeout
        transform = bool(calibrated and self.buffer.can_transform(
            self.body, self.optical, Time.from_msg(self.last_header.stamp)))
        message = 'Active'
        if not connected:
            message = 'Camera stream unavailable'
        elif not calibrated:
            message = self.error or 'Missing synchronized CameraInfo'
        elif not detecting:
            message = 'Detector output unavailable'
        elif not transform:
            message = 'Body-to-optical transform unavailable'
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK if message == 'Active' else DiagnosticStatus.WARN,
            name='drone_vision/camera', hardware_id=self.optical, message=message,
            values=[KeyValue(key=key, value=str(value)) for key, value in {
                'camera_connected': connected, 'calibration_valid': calibrated,
                'frame_rate_hz': round(self.frame_count / (now - self.last_rate_at), 2),
                'resolution': self.resolution, 'detector_active': detecting,
                'detections': self.detection_count if detecting else 0,
                'detection_latency_s': round(self.latency, 4) if detecting else 'unavailable',
                'body_transform_available': transform,
            }.items()],
        )
        self.frame_count = 0
        self.last_rate_at = now
        self.diagnostics.publish(DiagnosticArray(
            header=self.last_header if calibrated else self._diagnostic_header(), status=[status]))

    def _diagnostic_header(self):
        from std_msgs.msg import Header
        return Header(stamp=self.get_clock().now().to_msg())


def main():
    rclpy.init()
    node = CameraGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
