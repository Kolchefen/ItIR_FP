#!/usr/bin/env python3

# Face detector node.
# Subscribes to the TurtleBot 4's OAK-D color stream, runs an OpenCV Haar
# cascade face detector on each frame, and publishes a debounced Bool flag
# to /face_detected for the reactive controller to consume.
#
# Intended to run on a laptop (not the Pi) so the detector doesn't steal
# CPU from Nav2 / MPPI on the robot. Only the image stream comes in and a
# Bool goes out over the network.

from collections import deque

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class FaceDetector(Node):
    # Publishes /face_detected (Bool) based on Haar cascade results over a
    # rolling window of recent frames.

    def __init__(self):
        super().__init__('face_detector')

        # Parameters (override via ros2 run ... --ros-args -p name:=value)
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('flag_topic', '/face_detected')
        self.declare_parameter('window_size', 5)       # N frames in rolling buffer
        self.declare_parameter('min_hits', 2)          # K hits in window to assert True
        self.declare_parameter('min_face_size', 40)    # pixels, smallest face to accept
        self.declare_parameter('scale_factor', 1.2)    # Haar pyramid step
        self.declare_parameter('min_neighbors', 5)     # Haar detection stringency
        self.declare_parameter('show_preview', False)  # cv2.imshow for debug

        image_topic = self.get_parameter('image_topic').value
        flag_topic = self.get_parameter('flag_topic').value
        self.window_size = self.get_parameter('window_size').value
        self.min_hits = self.get_parameter('min_hits').value
        self.min_face_size = self.get_parameter('min_face_size').value
        self.scale_factor = self.get_parameter('scale_factor').value
        self.min_neighbors = self.get_parameter('min_neighbors').value
        self.show_preview = self.get_parameter('show_preview').value

        # Rolling window of recent per-frame detections (True/False)
        self.hits = deque(maxlen=self.window_size)
        self.last_published = None  # suppress duplicate publishes

        # Load Haar cascade from opencv-python's bundled data
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self.cascade = cv2.CascadeClassifier(cascade_path)
        if self.cascade.empty():
            self.get_logger().fatal(f'Failed to load Haar cascade: {cascade_path}')
            raise RuntimeError('Haar cascade load failed')

        self.bridge = CvBridge()

        # Camera image stream is BEST_EFFORT / VOLATILE on TB4
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, sensor_qos)
        self.flag_pub = self.create_publisher(Bool, flag_topic, 10)

        self.get_logger().info(
            f'FaceDetector started. image={image_topic} flag={flag_topic} '
            f'window={self.window_size} min_hits={self.min_hits}'
        )

    def image_callback(self, msg):
        # Convert ROS image to OpenCV BGR.
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge conversion failed: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)  # helps Haar in uneven lighting

        faces = self.cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=(self.min_face_size, self.min_face_size),
        )
        detected_this_frame = len(faces) > 0
        self.hits.append(detected_this_frame)

        # Debounce: assert True only if >= min_hits detections in the window.
        detected = sum(self.hits) >= self.min_hits

        # Only publish on state change to keep the topic quiet.
        if detected != self.last_published:
            self.flag_pub.publish(Bool(data=detected))
            self.last_published = detected
            self.get_logger().info(
                f'/face_detected -> {detected} '
                f'(window hits: {sum(self.hits)}/{len(self.hits)})'
            )

        if self.show_preview:
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = f'detected={detected}'
            cv2.putText(frame, label, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if detected else (0, 0, 255), 2)
            cv2.imshow('face_detector', frame)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = FaceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_preview:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
