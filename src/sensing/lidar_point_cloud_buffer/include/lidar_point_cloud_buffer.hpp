#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <deque>
#include <chrono>

using PC2Type = sensor_msgs::msg::PointCloud2;

using namespace std::chrono_literals;


class LidarPointCloudBuffer : public rclcpp::Node {
    public:
        explicit LidarPointCloudBuffer(const rclcpp::NodeOptions& options);

    private:
        rclcpp::Subscription<PC2Type>::SharedPtr pc_sub_;
        rclcpp::Publisher<PC2Type>::SharedPtr buffered_pc_pub_;

        void pcCallback(PC2Type::ConstSharedPtr pc_in);

        double buffer_time_;

        std::deque<PC2Type> pc_buffer_;
};