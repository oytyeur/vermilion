#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy 
from unitree_go.msg import Go2FrontVideoData
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class Go2VideoConverterNode(Node):
    def __init__(self):
        super().__init__('go2_video_converter')

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
        self.get_logger().warn('Конвертер ЗАПУЩЕН.')

    def callback(self, msg: Go2FrontVideoData):
        self.get_logger().warn(f'Кадр получен. Размер video720p: {len(msg.video720p)} байт')
        
        if len(msg.video720p) == 0:
            self.get_logger().error('Пустой кадр')
            return

        try:
            jpg_data = np.frombuffer(msg.video720p, dtype=np.uint8)
            frame = cv2.imdecode(jpg_data, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().error('Не удалось декодировать JPEG')
                return

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'

            self.publisher_.publish(img_msg)

            self.get_logger().info(f'Изображение опубликовано: {frame.shape[1]}x{frame.shape[0]}')

        except Exception as e:
            self.get_logger().error(f'Ошибка обработки: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = Go2VideoConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__': 
    main()