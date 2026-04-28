#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class DistanceTracker : public rclcpp::Node {
public:
    DistanceTracker();
    explicit DistanceTracker(const rclcpp::NodeOptions &options);

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    double last_x_;
    double last_y_;
    double total_distance_;
    bool initialized_;
};
