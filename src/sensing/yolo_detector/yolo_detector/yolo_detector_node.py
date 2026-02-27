#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # Объявление параметров
        self.declare_parameter('image_topic', '/camera/image_raw') 
        self.declare_parameter('detection_threshold', 0.5)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('enable_tracking', True)
        self.declare_parameter('target_classes', '')
        self.declare_parameter('qos_depth', 1)
        self.declare_parameter('reliability', 'best_effort')

        # Получение параметров
        self.image_topic = self.get_parameter('image_topic').value
        self.threshold = self.get_parameter('detection_threshold').value
        self.publish_annotated = self.get_parameter('publish_annotated_image').value
        self.model_name = self.get_parameter('model').value
        self.device = self.get_parameter('device').value
        self.enable_tracking = self.get_parameter('enable_tracking').value
        self.target_classes = self.get_parameter('target_classes').value
        qos_depth = self.get_parameter('qos_depth').value
        reliability_str = self.get_parameter('reliability').value

        if isinstance(self.target_classes, str):
            self.target_classes = [cls.strip() for cls in self.target_classes.split(',') if cls.strip()]

        # Настройка QoS для подписки
        reliability = (
            QoSReliabilityPolicy.RELIABLE
            if reliability_str.lower() == 'reliable'
            else QoSReliabilityPolicy.BEST_EFFORT
        )

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=reliability,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Логирование
        self.get_logger().info(f"Loading YOLO model: {self.model_name}")
        self.get_logger().info(f"Device: {self.device}")
        self.get_logger().info(f"Threshold: {self.threshold}")
        self.get_logger().info(f"Tracking: {'ON' if self.enable_tracking else 'OFF'}")
        self.get_logger().info(f"Target classes: {self.target_classes if self.target_classes else 'all'}")
        self.get_logger().info(f"QoS: {reliability_str.upper()}, depth={qos_depth}")

        # Загрузка модели
        try:
            self.model = YOLO(self.model_name)
            self.model.to(self.device)
            self.class_names = self.model.model.names
            self.get_logger().info(f"Model '{self.model_name}' loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            rclpy.shutdown()
            return

        # Инициализация
        self.bridge = CvBridge()

        # Подписка с QoS
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.listener_callback,
            qos_profile
        )
        self.subscription

        # Публикация детекций
        self.detections_pub = self.create_publisher(Detection2DArray, 'detected_objects', qos_profile)

        # Публикация аннотированного изображения
        self.annotated_pub = None
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(Image, 'annotated_image', qos_profile)

        self.get_logger().info("🟢 YOLO detector is running and listening...")

    def listener_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Настройка аргументов для инференса
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
        detections_msg.header = msg.header

        # Для аннотации
        annotated_boxes = []
        annotated_labels = []

        # <--- ДОБАВЛЕНА ПРОВЕРКА на None
        if results.boxes is not None:
            for box in results.boxes:
                confidence = float(box.conf.item())
                if confidence < self.threshold:
                    continue

                class_id = int(box.cls.item())
                class_name = self.class_names[class_id]

                # Фильтр по классам
                if self.target_classes and class_name not in self.target_classes:
                    continue

                # Вычисление bounding box
                xyxy = box.xyxy[0].cpu().numpy()
                x_min, y_min, x_max, y_max = xyxy
                center_x = float((x_min + x_max) / 2)
                center_y = float((y_min + y_max) / 2)
                size_x = float(x_max - x_min)
                size_y = float(y_max - y_min)

                # Формирование class_id с ID
                display_name = class_name
                if self.enable_tracking and hasattr(box, 'id') and box.id is not None:
                    track_id = int(box.id.item())
                    display_name = f"{class_name}#{track_id}"

                # Создание Detection2D
                detection = Detection2D()
                detection.header = msg.header
                detection.bbox.center.position.x = center_x
                detection.bbox.center.position.y = center_y
                detection.bbox.center.theta = 0.0
                detection.bbox.size_x = size_x
                detection.bbox.size_y = size_y

                obj_hypothesis = ObjectHypothesisWithPose()
                obj_hypothesis.hypothesis.class_id = display_name
                obj_hypothesis.hypothesis.score = confidence
                detection.results.append(obj_hypothesis)

                detections_msg.detections.append(detection)

                annotated_boxes.append(xyxy)
                annotated_labels.append(display_name)

        # Публикация детекций
        self.detections_pub.publish(detections_msg)

        # Публикация аннотированного изображения
        if self.annotated_pub:
            try:
                from ultralytics.utils.plotting import Annotator
                annotator = Annotator(cv_image, line_width=2)
                for box, label in zip(annotated_boxes, annotated_labels):
                    annotator.box_label(box, label=label, color=(255, 128, 0))
                annotated_frame = annotator.result()

                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                annotated_msg.header = msg.header
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