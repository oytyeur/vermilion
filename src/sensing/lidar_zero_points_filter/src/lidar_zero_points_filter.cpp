#include "../include/lidar_zero_points_filter.hpp"

LidarZeroPointsFilter::LidarZeroPointsFilter(const rclcpp::NodeOptions& node_options) :
    rclcpp::Node("lidar_zero_points_filter_node", node_options) {

    this->pc2_sub_ = 
        this->create_subscription<PC2Type>("input",
                                           rclcpp::SensorDataQoS().keep_last(1),
                                           std::bind(&LidarZeroPointsFilter::rawPCCallback,
                                                     this,
                                                     std::placeholders::_1));
                                                     
    this->filtered_pc2_pub_ = 
        this->create_publisher<PC2Type>("output",
                                        rclcpp::QoS{1});
}

void LidarZeroPointsFilter::rawPCCallback(PC2Type::ConstSharedPtr pc2_msg) {
    pcl::PointCloud<pcl::PointXYZI> input_pcl_cloud;
    pcl::fromROSMsg(*pc2_msg, input_pcl_cloud);

    pcl::PointCloud<pcl::PointXYZI> output_pcl_cloud;
    output_pcl_cloud.reserve(input_pcl_cloud.size()); // Вычесть 10000, вроде это постоянное кол-во нулевых точек

    for (const auto& pt : input_pcl_cloud) {
        if (pt.x == 0.0 and pt.y == 0.0) {
            continue;
        } else {
            output_pcl_cloud.push_back(pt);
        }
    }

    PC2Type out_pc2_msg;
    pcl::toROSMsg(output_pcl_cloud, out_pc2_msg);
    out_pc2_msg.header.stamp = pc2_msg->header.stamp;
    out_pc2_msg.header.frame_id = pc2_msg->header.frame_id;

    this->filtered_pc2_pub_->publish(out_pc2_msg);
}