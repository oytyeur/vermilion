#include "rclcpp/rclcpp.hpp"
#include "../include/people_counter.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<PeopleCounter>(options));
  rclcpp::shutdown();
  return 0;
}