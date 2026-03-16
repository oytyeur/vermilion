#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch, cv2 
import numpy as np

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter('image_topic', '/camera/raw_frame')
        self.declare_parameter('detection_threshold', 0.75)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('enable_tracking', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.threshold = self.get_parameter('detection_threshold').value
        self.publish_annotated = self.get_parameter('publish_annotated_image').value
        self.model_name = self.get_parameter('model').value
        self.device = self.get_parameter('device').value
        self.enable_tracking = self.get_parameter('enable_tracking').value

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.get_logger().info(f"Loading YOLO model: {self.model_name}")
        self.get_logger().info(f"Device: {self.device}")
        self.get_logger().info(f"Tracking: {'ON' if self.enable_tracking else 'OFF'}")

        try:
            self.model = YOLO(self.model_name)
            self.model.to(self.device)
            self.class_names = self.model.model.names
            self.get_logger().info(f"Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            rclpy.shutdown()
            return

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.listener_callback_compressed,
            qos_profile
        )

        self.detections_pub = self.create_publisher(Detection2DArray, '/detections', qos_profile)

        self.annotated_pub = None
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, '/camera/annotated_image', qos_profile)

        self.get_logger().info("Нода запущена корректно")

        self.counted_ids = set()
        self.current_frame_ids = set()

        self.unique_people_count = 1
        self.unique_chairs_count = 1

    def listener_callback_compressed(self, msg: CompressedImage):
        try:
            jpg_data = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = cv2.imdecode(jpg_data, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().warn('Failed to decode image')
                return

            from std_msgs.msg import Header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera_link'
            
            self.process_frame(cv_image, header)
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")

    def process_frame(self, cv_image, header):
        infer_args = {
            'source': cv_image,
            'verbose': False,
            'imgsz': 640,
        }

        try:
            if self.enable_tracking:
                infer_args['tracker'] = 'bytetrack.yaml'
                results = self.model.track(**infer_args)[0]
            else:
                results = self.model(**infer_args)[0]
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            return

        detections_msg = Detection2DArray()
        detections_msg.header = header

        annotated_boxes = []
        annotated_labels = []
        frame_people_count = 0

        if results.boxes is not None:

            self.current_frame_ids.clear()

            for box in results.boxes:
                confidence = float(box.conf.item())
                if confidence < self.threshold:
                    continue

                class_id = int(box.cls.item())
                class_name = self.class_names[class_id]

                if class_name not in ['person', 'chair']:
                    continue

                is_new_object = False
                track_id = None

                if self.enable_tracking and hasattr(box, 'id') and box.id is not None:
                    track_id = int(box.id.item())
                    self.current_frame_ids.add(track_id)

                    if track_id not in self.counted_ids:
                        self.counted_ids.add(track_id)
                        self.get_logger().info(f'Уникальных объектов: {self.unique_people_count} людей')
                        is_new_object = True
                    if is_new_object:
                        if class_name == 'person':
                            self.unique_people_count += 1
                        elif class_name == 'chair':
                            self.unique_chairs_count += 1


                else:
                    xyxy = box.xyxy[0].cpu().numpy()
                    pos_id = (
                        int(xyxy[0] // 10),
                        int(xyxy[1] // 10)
                    )
                    self.current_frame_ids.add(pos_id)

                    if pos_id not in self.counted_ids:
                        self.counted_ids.add(pos_id)
                        is_new_object = True

                xyxy = box.xyxy[0].cpu().numpy()
                x_min, y_min, x_max, y_max = xyxy
                center_x = float((x_min + x_max) / 2)
                center_y = float((y_min + y_max) / 2)
                size_x = float(x_max - x_min)
                size_y = float(y_max - y_min)

                detection = Detection2D()
                detection.header = header
                detection.bbox.center.position.x = center_x
                detection.bbox.center.position.y = center_y
                detection.bbox.center.theta = 0.0
                detection.bbox.size_x = size_x
                detection.bbox.size_y = size_y

                obj_hypothesis = ObjectHypothesisWithPose()
                obj_hypothesis.hypothesis.class_id = class_name
                obj_hypothesis.hypothesis.score = confidence
                detection.results.append(obj_hypothesis)

                detections_msg.detections.append(detection)
                annotated_boxes.append(xyxy)
                annotated_labels.append(class_name)
            
            for old_id in list(self.counted_ids):
                if old_id not in self.current_frame_ids:
                    self.counted_ids.discard(old_id)

        self.detections_pub.publish(detections_msg)

        # self.get_logger().info(
        #     f'Уникальных объектов: {self.unique_people_count} людей, {self.unique_chairs_count} стульев')


        if self.annotated_pub:
            try:
                from ultralytics.utils.plotting import Annotator
                annotator = Annotator(cv_image, line_width=2)
                for box, label in zip(annotated_boxes, annotated_labels):
                    annotator.box_label(box, label=label, color=(255, 128, 0))
                annotated_frame = annotator.result()

                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                annotated_msg.header = header
                self.annotated_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish annotated image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()