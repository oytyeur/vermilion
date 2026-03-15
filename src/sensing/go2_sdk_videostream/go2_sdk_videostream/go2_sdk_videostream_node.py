import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
       
        self.declare_parameter('network_interface', '')
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('frame_id', 'camera_link')
        
        network_interface = self.get_parameter('network_interface').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        if network_interface:
            ChannelFactoryInitialize(0, network_interface)
            self.get_logger().info(f'Сетевой интерфейс определен чикипучно: {network_interface}')
        else:
            ChannelFactoryInitialize(0)
            self.get_logger().info(f'Сетевой интерфейс определен не чикипучно (автовыбор)')

        self.client = VideoClient()
        self.client.SetTimeout(3.0)
        self.client.Init()
        
        self.publisher_ = self.create_publisher(
            CompressedImage, 
            '/camera/raw_frame', 
            10
        )
        
        self.timer = self.create_timer(
            1.0 / publish_rate, 
            self.timer_callback
        )
        
        self.get_logger().info(f'Нода работает чикипучно на {publish_rate} Hz')

    def timer_callback(self):
        try:
            code, data = self.client.GetImageSample()
            
            if code != 0:
                self.get_logger().warn(f'Ошибка {code}')
                return
            
            msg = CompressedImage()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = bytes(data) if isinstance(data, list) else data
            
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Ошибка в timer_callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FramePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()