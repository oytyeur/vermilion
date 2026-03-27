#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "sensor_msgs/msg/image.hpp"

using Image = sensor_msgs::msg::Image;
using Detection2DArr = vision_msgs::msg::Detection2DArray;
using Detection2D = vision_msgs::msg::Detection2D;

class PeopleCounter : public rclcpp::Node {
    public:
        explicit PeopleCounter(const rclcpp::NodeOptions& node_options);
    
    private:
        rclcpp::Subscription<Detection2DArr>::SharedPtr detection_sub_;
        rclcpp::Publisher<Image>::SharedPtr image_pub_;

        void detectionArrayCallback(const Detection2DArr::ConstSharedPtr detect_msg);

        double conf_threshold = 0.2;

};