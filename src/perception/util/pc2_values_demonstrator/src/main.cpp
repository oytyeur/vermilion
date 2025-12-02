#include "rclcpp/rclcpp.hpp"
#include "../include/pc2_values_demonstrator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<PC2ValuesDemonstrator>(options));
  rclcpp::shutdown();
  return 0;
}