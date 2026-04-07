#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <unordered_set>
#include <string>
#include <functional>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>

using Image = sensor_msgs::msg::Image;
using CompressedImage = sensor_msgs::msg::CompressedImage;
using Detection2DArr = vision_msgs::msg::Detection2DArray;
using Detection2D = vision_msgs::msg::Detection2D;

class PeopleCounter : public rclcpp::Node {
    public:
        explicit PeopleCounter(const rclcpp::NodeOptions& node_options);
    
    private:
        void imageCallback(const CompressedImage::ConstSharedPtr& img_msg);
        void updatePersonCounter(const Detection2DArr::ConstSharedPtr detect_msg);

        rclcpp::Subscription<CompressedImage>::SharedPtr image_sub_;
        rclcpp::Subscription<Detection2DArr>::SharedPtr detection_sub_;
        rclcpp::Publisher<Image>::SharedPtr image_pub_;

        double conf_threshold_;

        std::unordered_set<std::string> current_det_ids_;
        std::unordered_set<std::string> counted_ids_;
        int unique_persons_count_ = 0;

        std::mutex imgage_mutex_;
        CompressedImage::ConstSharedPtr cv_image_;

};