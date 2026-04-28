#include "distance_tracker.hpp"
#include <cmath>
#include <functional>

DistanceTracker::DistanceTracker() : DistanceTracker(rclcpp::NodeOptions()) {}

DistanceTracker::DistanceTracker(const rclcpp::NodeOptions &options)
    : Node("distance_tracker", options) {
    this->declare_parameter<std::string>("odom_topic", "/odom");
    std::string odom_topic = this->get_parameter("odom_topic").as_string();

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic,
        10,
        std::bind(&DistanceTracker::odomCallback,
                  this,
                  std::placeholders::_1));

    last_x_ = 0.0;
    last_y_ = 0.0;
    total_distance_ = 0.0;
    initialized_ = false;

    RCLCPP_INFO(this->get_logger(), "Node started");
}

void DistanceTracker::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double cur_x = msg->pose.pose.position.x;
    double cur_y = msg->pose.pose.position.y;

    if (!std::isfinite(cur_x) || !std::isfinite(cur_y)) return;

    if (!initialized_) {
        last_x_ = cur_x;
        last_y_ = cur_y;
        initialized_ = true;
        return;
    }

    double step_distance = std::hypot(cur_x - last_x_, cur_y - last_y_);
    total_distance_ += step_distance;

    RCLCPP_INFO(this->get_logger(), "Пройденное расстояние: %.2f м", total_distance_);

    last_x_ = cur_x;
    last_y_ = cur_y;
}
