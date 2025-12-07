#include "rclcpp/rclcpp.hpp"
#include "../include/lidar_points_to_baselink_transformer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<LidarPointsToBaselinkTransformer>(options));
  rclcpp::shutdown();
  return 0;
}