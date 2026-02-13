#include "../include/high_level_motion_controller.hpp"

HighLevelMotionController::HighLevelMotionController(const rclcpp::NodeOptions& node_options) : 
    rclcpp::Node("high_level_motion_controller_node", node_options),
    sport_client_(this) {
    
    this->cmd_twist_sub_ = 
        this->create_subscription<Twist>("input",
                                         rclcpp::SensorDataQoS().keep_last(1),
                                         std::bind(&HighLevelMotionController::cmdTwistCallback,
                                                   this,
                                                   std::placeholders::_1));
}

void HighLevelMotionController::cmdTwistCallback(const Twist::ConstSharedPtr twist) {
    float vx = twist->linear.x;
    float wz = twist->angular.z;

    if (vx == 0.0 and wz == 0.0) {
        this->sport_client_.StopMove(this->req_);
        return;
    }

    this->sport_client_.Move(this->req_, vx, 0.0, wz);

    RCLCPP_INFO(this->get_logger(), "CMD VEL: vx %f, wz %f", vx, wz);
}