#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using Image = sensor_msgs::msg::Image;
using Detection2D = vision_msgs::msg::Detection2D;
using Detection2DArr = vision_msgs::msg::Detection2DArray;
using Twist = geometry_msgs::msg::Twist;

class SpecificObjectFollower : public rclcpp::Node {
    public:
        explicit SpecificObjectFollower(const rclcpp::NodeOptions& options);

    private:
        rclcpp::Subscription<Detection2DArr>::SharedPtr detection_sub_;
        rclcpp::Publisher<Image>::SharedPtr image_pub_;

        void detectionArrayCallback(const Detection2DArr::ConstSharedPtr msg);

        double conf_threshold;
};