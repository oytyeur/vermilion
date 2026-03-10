import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, MultiArrayDimension, MultiArrayLayout

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient


class FramePublisher(Node):
    def __init__(self):
        super().__init__('frame_publisher')
       
        self.declare_parameter('network_interface', 'enp0s31f6')
        self.declare_parameter('publish_rate', 10.0)
        
        network_interface = self.get_parameter('network_interface').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        if network_interface:
            ChannelFactoryInitialize(0, network_interface)
            self.get_logger().info(f'Сетевой интерфейс определен чикипучно')
        else:
            ChannelFactoryInitialize(0)
            self.get_logger().info(f'Сетевой интерфейс определен не чикипучно')

        self.client = VideoClient()
        self.client.SetTimeout(3.0)
        self.client.Init()
        
        self.publisher_ = self.create_publisher(
            UInt8MultiArray, 
            '/camera/raw_frame', 
            10
        )
        
        self.timer = self.create_timer(
            1.0 / publish_rate, 
            self.timer_callback
        )
        
        self.get_logger().info(f'Нода работает чикипучно')

    def timer_callback(self):
        try:
            code, data = self.client.GetImageSample()
            
            if code != 0:
                self.get_logger().warn(f'Пупупу')
                return
            
            msg = UInt8MultiArray()
            msg.layout = MultiArrayLayout(
                dim=[
                    MultiArrayDimension(label='height', size=0, stride=0),
                    MultiArrayDimension(label='width', size=0, stride=0),
                    MultiArrayDimension(label='channel', size=3, stride=0)
                ],
                data_offset=0
            )
            
            if isinstance(data, bytes):
                msg.data = list(data)
            else:
                msg.data = data
            
            self.publisher_.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Пупупу по причине: {e}')


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