#include "rclcpp/rclcpp.hpp"
#include "../include/distance_tracker.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<DistanceTracker>(options));
  rclcpp::shutdown();
  return 0;
}