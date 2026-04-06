#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "ros2_sport_client.h"

using Image = sensor_msgs::msg::Image;
using Detection2D = vision_msgs::msg::Detection2D;
using Detection2DArr = vision_msgs::msg::Detection2DArray;
using HypoWithPose = vision_msgs::msg::ObjectHypothesisWithPose;
// using Twist = geometry_msgs::msg::Twist;

class SpecificObjectFollower : public rclcpp::Node {
    public:
        explicit SpecificObjectFollower(const rclcpp::NodeOptions& options);

    private:
        rclcpp::Subscription<Detection2DArr>::SharedPtr detection_sub_;
        // rclcpp::Publisher<Image>::SharedPtr image_pub_;
        SportClient sport_client_;
        unitree_api::msg::Request req_;

        void detectionArrayCallback(const Detection2DArr::ConstSharedPtr msg);

        double conf_threshold;
        std::string spec_obj_name;

        const double MIN_X = 0.0;
        const double MAX_X = 1920.0;
        const double CENTRE_X = MAX_X * 0.5;
        const double abs_w_max = M_PI / 3 ;
        const double Kp = abs_w_max / (CENTRE_X - MIN_X);

};