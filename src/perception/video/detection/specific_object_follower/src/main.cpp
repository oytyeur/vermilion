#include "rclcpp/rclcpp.hpp"
#include "../include/specific_object_follower.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<SpecificObjectFollower>(options));
  rclcpp::shutdown();
  return 0;
}