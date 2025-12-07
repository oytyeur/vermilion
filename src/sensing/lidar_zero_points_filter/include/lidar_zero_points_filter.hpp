#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"

using PC2Type = sensor_msgs::msg::PointCloud2;

class LidarZeroPointsFilter : public rclcpp::Node {
    public:
        explicit LidarZeroPointsFilter(const rclcpp::NodeOptions& node_options);

    private:
        void rawPCCallback(PC2Type::ConstSharedPtr pc2_msg);

        rclcpp::Subscription<PC2Type>::SharedPtr pc2_sub_;
        rclcpp::Publisher<PC2Type>::SharedPtr filtered_pc2_pub_;
};