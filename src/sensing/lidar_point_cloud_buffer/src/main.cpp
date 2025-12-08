#include "rclcpp/rclcpp.hpp"
#include "../include/lidar_point_cloud_buffer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<LidarPointCloudBuffer>(options));
  rclcpp::shutdown();
  return 0;
}