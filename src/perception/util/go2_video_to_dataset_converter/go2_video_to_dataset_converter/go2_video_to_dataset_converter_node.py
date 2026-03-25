#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class Go2VideoToDatasetConverterNode(Node):
    def __init__(self):
        super().__init__('go2_video_to_dataset_converter')

        self.publisher_ = self.create_publisher(
            Image,
            '/front_camera/image_raw',
            10
        )

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/raw_frame',
            self.callback,
            qos
        )

        self.bridge = CvBridge()
        self.get_logger().info('Converter started. Listening on /camera/raw_frame')

        # Настройка выходного файла
        self.output_video_path = 'dataset_video.mp4'
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(self.output_video_path, self.fourcc, 30, (1920, 1080))

    def callback(self, msg: CompressedImage):
        if len(msg.data) == 0:
            self.get_logger().warn('Empty frame')
            return

        self.get_logger().info(f'Frame received. Size: {len(msg.data)} bytes')

        try:
            jpg_data = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(jpg_data, cv2.IMREAD_COLOR)
            
            if frame is None:
                self.get_logger().error('Failed to decode JPEG')
                return
            
            self.writer.write(frame)

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'
            
            self.publisher_.publish(img_msg)
            self.get_logger().info(f'Published: {frame.shape[1]}x{frame.shape[0]}')
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = Go2VideoToDatasetConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__': 
    main()