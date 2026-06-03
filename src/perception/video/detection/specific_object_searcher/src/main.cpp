#include "rclcpp/rclcpp.hpp"
#include "../include/specific_object_searcher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<SpecificObjectSearcher>(options));
  rclcpp::shutdown();
  return 0;
}