#ifndef SPECIFIC_OBJECT_SEARCHER_HPP
#define SPECIFIC_OBJECT_SEARCHER_HPP

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "unitree/robot/go2/video/video_client.hpp"
#include <string>
#include <functional>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <fstream>
#include <vector>
#include <memory>

using Detection2DArr = vision_msgs::msg::Detection2DArray;
using Detection2D = vision_msgs::msg::Detection2D;

class SpecificObjectSearcher : public rclcpp::Node {
public:
    explicit SpecificObjectSearcher(const rclcpp::NodeOptions& node_options);
    ~SpecificObjectSearcher();

private:
    void detectionCallback(const Detection2DArr::ConstSharedPtr detect_msg);
    
    bool initializeVideoClient();
    bool getCurrentTransform(geometry_msgs::msg::TransformStamped& transform);
    void overlayPositionInfo(cv::Mat& frame);
    void saveFrameWithInfo(const cv::Mat& frame);
    
    rclcpp::Subscription<Detection2DArr>::SharedPtr detection_sub_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    std::unique_ptr<unitree::robot::go2::VideoClient> video_client_;
    
    std::string network_interface_;
    std::string target_class_;
    double conf_threshold_;
    std::string save_directory_;
    int video_timeout_;
    std::string base_frame_;
    std::string odom_frame_;
    
    geometry_msgs::msg::TransformStamped initial_transform_;
    bool initial_pose_set_;
    bool target_found_;
    
    std::vector<uint8_t> current_image_data_;
};

#endif