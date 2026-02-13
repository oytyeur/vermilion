#include "rclcpp/rclcpp.hpp"
#include "../include/high_level_motion_controller.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<HighLevelMotionController>(options));
  rclcpp::shutdown();
  return 0;
}