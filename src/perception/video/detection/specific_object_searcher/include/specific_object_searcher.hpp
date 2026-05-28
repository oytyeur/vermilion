#ifndef SPECIFIC_OBJECT_SEARCH_HPP
#define SPECIFIC_OBJECT_SEARCH_HPP

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
using Odometry = nav_msgs::msg::Odometry;

class SpecificObjectSearch : public rclcpp::Node {
public:
    explicit SpecificObjectSearch(const rclcpp::NodeOptions& node_options);
    ~SpecificObjectSearch();

private:
    void detectionCallback(const Detection2DArr::ConstSharedPtr detect_msg);
    void odomCallback(const Odometry::ConstSharedPtr odom_msg);
    
    bool initializeVideoClient();
    void processDetections(const std::vector<Detection2D>& detections);
    void overlayPositionInfo(cv::Mat& frame);
    void saveFrameWithInfo(const cv::Mat& frame);
    
    rclcpp::Subscription<Detection2DArr>::SharedPtr detection_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    
    std::unique_ptr<unitree::robot::go2::VideoClient> video_client_;
    
    std::string network_interface_;
    std::string target_class_;
    double conf_threshold_;
    std::string save_directory_;
    int video_timeout_;
    
    Odometry initial_odom_;
    Odometry current_odom_;
    bool initial_pose_set_;
    bool target_found_;
    
    std::vector<uint8_t> current_image_data_;
};

#endif