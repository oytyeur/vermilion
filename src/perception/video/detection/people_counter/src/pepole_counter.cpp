#include "../include/people_counter.hpp"

PeopleCounter::PeopleCounter(const rclcpp::NodeOptions& node_options) :
    rclcpp::Node("people_counter_node", node_options) {

    this->detection_sub_ =
        this->create_subscription<Detection2DArr>("input",
                                                  rclcpp::SensorDataQoS().keep_last(1),
                                                  std::bind(&PeopleCounter::detectionArrayCallback,
                                                            this,
                                                            std::placeholders::_1));
    
    this->image_pub_ = 
        this->create_publisher<Image>("output",
                                      rclcpp::QoS(1));
    
    this->conf_threshold = 
        this->declare_parameter("conf_threshold", 0.2);
    
        
};
    
void PeopleCounter::detectionArrayCallback(const Detection2DArr::ConstSharedPtr detect_msg) {
    std::vector<Detection2D> det_arr = detect_msg->detections;

    RCLCPP_INFO(this->get_logger(), "%li", det_arr.size());

}