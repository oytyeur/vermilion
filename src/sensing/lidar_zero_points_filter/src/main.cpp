#include "rclcpp/rclcpp.hpp"
#include "../include/lidar_zero_points_filter.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<LidarZeroPointsFilter>(options));
  rclcpp::shutdown();
  return 0;
}