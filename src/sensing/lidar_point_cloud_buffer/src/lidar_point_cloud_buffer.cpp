#include "../include/lidar_point_cloud_buffer.hpp"

LidarPointCloudBuffer::LidarPointCloudBuffer(const rclcpp::NodeOptions& options) :
    rclcpp::Node("lidar_point_cloud_buffer_node", options) {
    
    this->pc_sub_ = 
        this->create_subscription<PC2Type>("input",
                                           rclcpp::SensorDataQoS().keep_last(1),
                                           std::bind(&LidarPointCloudBuffer::pcCallback,
                                                     this,
                                                     std::placeholders::_1));
                                                     
    this->buffer_time_ = 
        this->declare_parameter("buffer_time", 0.5);
    
    this->buffered_pc_pub_ = 
        this->create_publisher<PC2Type>("output",
                                        rclcpp::QoS{1});
    
}


void LidarPointCloudBuffer::pcCallback(PC2Type::ConstSharedPtr pc_in) {
    this->pc_buffer_.push_back(*pc_in);

    while (true) {
        if (this->pc_buffer_.size() < 2) {
            return;
        }

        double dt = 
            static_cast<double>(this->pc_buffer_.back().header.stamp.sec - this->pc_buffer_.front().header.stamp.sec)
            +
            static_cast<double>(this->pc_buffer_.back().header.stamp.nanosec * 1.0e-9) - static_cast<double>(this->pc_buffer_.front().header.stamp.nanosec * 1.0e-9);
        if(dt > this->buffer_time_) {
            this->pc_buffer_.pop_front();
            continue;
        } else {
            break;
        }
    }

    PC2Type out_pc;
    for (const auto& cloud : this->pc_buffer_) {
        pcl::concatenatePointCloud(out_pc, cloud, out_pc);
    }

    out_pc.header.stamp = this->pc_buffer_.back().header.stamp;

    this->buffered_pc_pub_->publish(out_pc);
}
