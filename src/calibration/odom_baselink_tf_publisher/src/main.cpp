#include "rclcpp/rclcpp.hpp"
#include "../include/odom_baselink_tf_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<OdomBaselinkTFPublisher>(options));
  rclcpp::shutdown();
  return 0;
}