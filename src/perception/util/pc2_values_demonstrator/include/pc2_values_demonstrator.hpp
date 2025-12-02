#ifndef PC2_VALUES_DEMONSTRATOR_HPP_
#define PC2_VALUES_DEMONSTRATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <algorithm>

using PC2Type = sensor_msgs::msg::PointCloud2;

class PC2ValuesDemonstrator : public rclcpp::Node {
    public:
        explicit PC2ValuesDemonstrator(const rclcpp::NodeOptions& options);

    private:
        rclcpp::Subscription<PC2Type>::SharedPtr pc2_sub_;

        void pc2Callback(const PC2Type::SharedPtr msg);
};

#endif
