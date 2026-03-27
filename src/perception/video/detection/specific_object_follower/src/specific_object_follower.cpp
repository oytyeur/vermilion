#include "../include/specific_object_follower.hpp"

SpecificObjectFollower::SpecificObjectFollower(const rclcpp::NodeOptions& options) :
    rclcpp::Node("specific_object_follower_node", options) {

    this->detection_sub_ = 
        this->create_subscription<Detection2DArr>("input",
                                                  rclcpp::SensorDataQoS().keep_last(1),
                                                  std::bind(&SpecificObjectFollower::detectionArrayCallback,
                                                            this,
                                                            std::placeholders::_1));
    
    this->image_pub_ = 
        this->create_publisher<Image>("output",
                                      rclcpp::QoS{1});

    this->conf_threshold = 
        this->declare_parameter("conf_threshold", 0.8);
}

void SpecificObjectFollower::detectionArrayCallback(const Detection2DArr::ConstSharedPtr msg) {
    std::vector<Detection2D> det_arr = msg->detections;

    RCLCPP_INFO(this->get_logger(), "%li", det_arr.size());
}