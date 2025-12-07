#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "pcl_ros/transforms.hpp"
// #include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using TF2Type = geometry_msgs::msg::TransformStamped;
using PC2Type = sensor_msgs::msg::PointCloud2;

class LidarPointsToBaselinkTransformer : public rclcpp::Node {
    public:
        explicit LidarPointsToBaselinkTransformer(const rclcpp::NodeOptions& options);

    private:
        rclcpp::Subscription<PC2Type>::SharedPtr pc2_sub_;
        rclcpp::Publisher<PC2Type>::SharedPtr pc2_pub_;

        void pointcloudCallback(PC2Type::ConstSharedPtr pc2_msg);

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};