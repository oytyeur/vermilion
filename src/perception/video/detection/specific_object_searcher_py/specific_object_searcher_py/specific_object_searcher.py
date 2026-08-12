#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from vision_msgs.msg import Detection2DArray
from tf2_ros import Buffer, TransformListener
import cv2
import numpy as np
import os
from datetime import datetime
import traceback

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


class SpecificObjectSearcher(Node):
    def __init__(self):
        super().__init__('specific_object_searcher_node')
        
        self.declare_parameter('network_interface', '')
        self.declare_parameter('target_class', 'redball')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('video_timeout', 3.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        
        network_interface = self.get_parameter('network_interface').value
        self.target_class = self.get_parameter('target_class').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        video_timeout = self.get_parameter('video_timeout').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        if network_interface:
            ChannelFactoryInitialize(0, network_interface)
            self.get_logger().info(f'Network interface: {network_interface}')
        else:
            ChannelFactoryInitialize(0)
            self.get_logger().info('Network interface: auto')
        
        self.video_client = VideoClient()
        self.video_client.SetTimeout(video_timeout)
        self.video_client.Init()
        self.get_logger().info('Video client initialized')
        
        today = datetime.now().strftime('%Y-%m-%d')
        self.save_directory = os.path.expanduser(
            f'~/specific_object_searcher/found_objects/{today}'
        )
        os.makedirs(self.save_directory, exist_ok=True)
        self.get_logger().info(f'Save directory: {self.save_directory}')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.initial_pose_set = False
        self.initial_transform = None
        self.callback_count = 0
        
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            qos
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('SpecificObjectSearcher started')
        self.get_logger().info(f'Target class: {self.target_class}')
        self.get_logger().info('=' * 60)
    
    def get_current_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().info(f'{e}')
            return None
    
    def get_image(self):
        try:
            code, data = self.video_client.GetImageSample()
            if code != 0:
                return None
            return cv2.imdecode(np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().info(f'{e}')
            return None
    
    def overlay_info(self, frame, transform):
        if not self.initial_transform:
            return
        
        rel_x = transform.transform.translation.x - self.initial_transform.transform.translation.x
        rel_y = transform.transform.translation.y - self.initial_transform.transform.translation.y
        rel_z = transform.transform.translation.z - self.initial_transform.transform.translation.z
        q = transform.transform.rotation
        
        lines = [
            f"Target: {self.target_class}",
            f"Rel Pos: x={rel_x:.2f} y={rel_y:.2f} z={rel_z:.2f}",
            f"Rot: x={q.x:.2f} y={q.y:.2f} z={q.z:.2f} w={q.w:.2f}"
        ]
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale, thickness, color, spacing = 0.6, 2, (0, 0, 255), 25
        
        max_width = max(cv2.getTextSize(line, font, font_scale, thickness)[0][0] for line in lines)
        start_x, start_y = frame.shape[1] - max_width - 20, 30
        
        roi = (start_x - 10, start_y - 25, max_width + 20, len(lines) * spacing + 20)
        overlay = frame.copy()
        cv2.rectangle(overlay, (roi[0], roi[1]), (roi[0] + roi[2], roi[1] + roi[3]), (0, 0, 0), cv2.FILLED)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (start_x, start_y + i * spacing), font, font_scale, color, thickness)
    
    def save_frame(self, frame):
        timestamp = datetime.now().strftime('%H%M%S_%f')[:-3]
        filename = os.path.join(self.save_directory, f"{self.target_class}_{timestamp}.jpg")
        
        if cv2.imwrite(filename, frame):
            self.get_logger().info(f'Saved: {filename}')
    
    def detection_callback(self, msg):
        
        frame = self.get_image()
        if frame is None:
            return
        
        for detection in msg.detections:
            if not detection.results:

                continue
            
            class_id = detection.results[0].hypothesis.class_id
            score = detection.results[0].hypothesis.score
            
            if class_id == self.target_class and score >= self.conf_threshold:
                self.get_logger().info(f'Found {self.target_class} (confidence: {score:.3f})')
                
                transform = self.get_current_transform()
                if transform:
                    if not self.initial_pose_set:
                        self.initial_transform = transform
                        self.initial_pose_set = True
                        pos = self.initial_transform.transform.translation
                        self.get_logger().info(f'Initial position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}')
                    
                    self.overlay_info(frame, transform)
                    self.save_frame(frame)
                
                break


def main(args=None):
    rclpy.init(args=args)
    node = SpecificObjectSearcher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()