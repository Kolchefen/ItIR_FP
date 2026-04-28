
#!/usr/bin/env python3

from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


def imgmsg_to_bgr8(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    if encoding in ('bgr8', 'rgb8'):
        frame = buf.reshape(msg.height, msg.width, 3)
        if encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    if encoding == 'mono8':
        gray = buf.reshape(msg.height, msg.width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    if encoding in ('bgra8', 'rgba8'):
        frame = buf.reshape(msg.height, msg.width, 4)
        code = cv2.COLOR_BGRA2BGR if encoding == 'bgra8' else cv2.COLOR_RGBA2BGR
        return cv2.cvtColor(frame, code)

    raise ValueError(f'Unsupported image encoding: {msg.encoding}')


class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')

        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')

        # The executor scaffold expects this parameter to point to /face_detected.
        self.declare_parameter('flag_bool', '/face_detected')

        self.declare_parameter('window_size', 5)
        self.declare_parameter('min_hits', 2)
        self.declare_parameter('min_face_size', 40)
        self.declare_parameter('scale_factor', 1.2)
        self.declare_parameter('min_neighbors', 5)
        self.declare_parameter('show_preview', False)

        self.image_topic = self.get_parameter('image_topic').value
        self.flag_topic = self.get_parameter('flag_bool').value

        self.window_size = int(self.get_parameter('window_size').value)
        self.min_hits = int(self.get_parameter('min_hits').value)
        self.min_face_size = int(self.get_parameter('min_face_size').value)
        self.scale_factor = float(self.get_parameter('scale_factor').value)
        self.min_neighbors = int(self.get_parameter('min_neighbors').value)
        self.show_preview = bool(self.get_parameter('show_preview').value)

        self.validate_parameters()

        # Stores recent True/False detections so one bad frame doent flip the result
        self.detection_history = deque(maxlen=self.window_size)
        self.last_state = None

        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

        if self.face_cascade.empty():
            self.get_logger().fatal(f'Could not load Haar cascade: {cascade_path}')
            raise RuntimeError('Failed to load Haar cascade')

        # OAK-D image streams usually use best-effort QoS, so the subscriber should match it.
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.image_subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos
        )

        self.face_publisher = self.create_publisher(
            Bool,
            self.flag_topic,
            10
        )

        self.get_logger().info(
            f'Face detector started: {self.image_topic} -> {self.flag_topic}'
        )

    def validate_parameters(self):
        if self.window_size <= 0:
            raise ValueError('window_size must be greater than 0')

        if self.min_hits <= 0:
            raise ValueError('min_hits must be greater than 0')

        if self.min_hits > self.window_size:
            raise ValueError('min_hits cannot be greater than window_size')

        if self.min_face_size <= 0:
            raise ValueError('min_face_size must be greater than 0')

        if self.scale_factor <= 1.0:
            raise ValueError('scale_factor must be greater than 1.0')

        if self.min_neighbors < 0:
            raise ValueError('min_neighbors cannot be negative')

    def image_callback(self, msg):
        try:
            frame = imgmsg_to_bgr8(msg)
        except Exception as error:
            self.get_logger().warn(f'Image conversion failed: {error}')
            return

        faces = self.detect_faces(frame)

        self.detection_history.append(len(faces) > 0)
        face_detected = self.is_face_confirmed()

        self.publish_face_state(face_detected)

        if self.show_preview:
            self.show_debug_preview(frame, faces, face_detected)

    def detect_faces(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Help with uneven lighting
        gray = cv2.equalizeHist(gray)

        return self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=(self.min_face_size, self.min_face_size)
        )

    def is_face_confirmed(self):
        # Requires several detections inside the rolling window before publishing True.
        return sum(self.detection_history) >= self.min_hits

    def publish_face_state(self, face_detected):
        self.face_publisher.publish(Bool(data=face_detected))

        if face_detected != self.last_state:
            hits = sum(self.detection_history)
            total = len(self.detection_history)

            self.get_logger().info(
                f'{self.flag_topic} -> {face_detected} ({hits}/{total})'
            )

            self.last_state = face_detected

    def show_debug_preview(self, frame, faces, face_detected):
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        cv2.putText(
            frame,
            f'face_detected={face_detected}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0) if face_detected else (0, 0, 255),
            2
        )

        cv2.imshow('face_detector', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

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