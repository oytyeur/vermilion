#include "../include/lidar_points_to_baselink_transformer.hpp"

LidarPointsToBaselinkTransformer::LidarPointsToBaselinkTransformer(const rclcpp::NodeOptions& options) :
    rclcpp::Node("lidar_points_to_baselink_transformer_node", options) {
    
    this->pc2_sub_ = 
        this->create_subscription<PC2Type>("input",
                                           rclcpp::SensorDataQoS().keep_last(1),
                                           std::bind(&LidarPointsToBaselinkTransformer::pointcloudCallback,
                                                     this,
                                                     std::placeholders::_1));

    this->pc2_pub_ = 
        this->create_publisher<PC2Type>("output",
                                        rclcpp::QoS{1});
    
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);
}

void LidarPointsToBaselinkTransformer::pointcloudCallback(PC2Type::ConstSharedPtr pc2_msg) {

    PC2Type out_pc2_msg;
    pcl_ros::transformPointCloud("base_link", *pc2_msg, out_pc2_msg, *this->tf_buffer_);

    this->pc2_pub_->publish(out_pc2_msg);
}