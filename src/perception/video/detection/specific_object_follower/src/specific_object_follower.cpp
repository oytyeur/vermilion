#include "../include/specific_object_follower.hpp"

SpecificObjectFollower::SpecificObjectFollower(const rclcpp::NodeOptions& options) :
    rclcpp::Node("specific_object_follower_node", options),
    sport_client_(this) {

    this->detection_sub_ = 
        this->create_subscription<Detection2DArr>("input",
                                                  rclcpp::SensorDataQoS().keep_last(1),
                                                  std::bind(&SpecificObjectFollower::detectionArrayCallback,
                                                            this,
                                                            std::placeholders::_1));
    
    // this->image_pub_ = 
    //     this->create_publisher<Image>("output",
    //                                   rclcpp::QoS{1});



    this->conf_threshold = 
        this->declare_parameter("conf_threshold", 0.8);
    this->spec_obj_name = 
        this->declare_parameter("spec_obj_name", "redball");
}

void SpecificObjectFollower::detectionArrayCallback(const Detection2DArr::ConstSharedPtr msg) {
    std::vector<Detection2D> det_arr = msg->detections;
    
    bool has_valid_obj = false;
    double v = 0.0;
    double side = -1.0;
    double w = 0.0;
    double x = -1.0;
    
    Detection2D valid_detection;
    HypoWithPose hypo_and_pose;
    hypo_and_pose.hypothesis.score = 0.0;
    valid_detection.results.push_back(hypo_and_pose);
    
    if (det_arr.empty()) {
        this->sport_client_.StopMove(this->req_);
    } else {
        
        for (const auto& det : det_arr) {
            const auto object_hypo = det.results.at(0).hypothesis;
            if (object_hypo.class_id != this->spec_obj_name or object_hypo.score < this->conf_threshold) {
                continue;
            }
    
            if (object_hypo.score > valid_detection.results.at(0).hypothesis.score) {
                valid_detection = det;
                has_valid_obj = true;
            }
        }

        // RCLCPP_INFO(this->get_logger(), "S: %f", side);
    }

    if (!has_valid_obj) {
        this->sport_client_.StopMove(this->req_);
    } else {
        x = valid_detection.bbox.center.position.x;
        w = (this->CENTRE_X - x) * this->Kp_w;

        side = std::max(valid_detection.bbox.size_x, valid_detection.bbox.size_y);
        v = std::max(0.0, std::min(this->abs_v_max, (this->GOAL_S - side) * this->Kp_v));
        this->sport_client_.Move(this->req_, v, 0.0, w);
    }

    RCLCPP_INFO(this->get_logger(), "S: %f, V: %f", side, v);
}