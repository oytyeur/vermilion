#include "../include/odom_baselink_tf_publisher.hpp"

OdomBaselinkTFPublisher::OdomBaselinkTFPublisher(const rclcpp::NodeOptions& options) : 
    rclcpp::Node("odom_baselink_tf_publisher_node", options) {
    
    this->pose_sub_ = 
        this->create_subscription<PoseStamped>("input",
                                               rclcpp::SensorDataQoS().keep_last(1),
                                               std::bind(&OdomBaselinkTFPublisher::poseCallback, 
                                                         this,
                                                         std::placeholders::_1));

    this->tf_broadcaster_ = 
        std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void OdomBaselinkTFPublisher::poseCallback(PoseStamped::ConstSharedPtr pose) {
    TFStamped tf;
    tf.header.stamp = pose->header.stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";

    // translation
    tf.transform.translation.x = pose->pose.position.x;
    tf.transform.translation.y = pose->pose.position.y;
    tf.transform.translation.z = pose->pose.position.z;

    // rotation
    tf.transform.rotation = pose->pose.orientation;


    this->tf_broadcaster_->sendTransform(tf);
}