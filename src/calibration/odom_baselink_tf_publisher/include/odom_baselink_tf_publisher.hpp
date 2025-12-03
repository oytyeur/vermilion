#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using PoseStamped = geometry_msgs::msg::PoseStamped;
using TFStamped = geometry_msgs::msg::TransformStamped;

class OdomBaselinkTFPublisher : public rclcpp::Node {
  public:
    explicit OdomBaselinkTFPublisher(const rclcpp::NodeOptions& options);

  private:
    void poseCallback(PoseStamped::ConstSharedPtr pose);

    rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};