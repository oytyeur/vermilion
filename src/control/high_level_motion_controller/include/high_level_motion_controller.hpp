#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_sport_client.h"
// #include "unitree_api/msg/request.hpp"

using Twist = geometry_msgs::msg::Twist;

class HighLevelMotionController : public rclcpp::Node {
    public:
        explicit HighLevelMotionController(const rclcpp::NodeOptions& node_options);

    private:
        rclcpp::Subscription<Twist>::SharedPtr cmd_twist_sub_;

        void cmdTwistCallback(const Twist::ConstSharedPtr cmd_twist);

        SportClient sport_client_;
        unitree_api::msg::Request req_;

};