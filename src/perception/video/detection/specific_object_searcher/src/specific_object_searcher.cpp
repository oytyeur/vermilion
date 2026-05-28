#include "../include/specific_object_searcher.hpp"


SpecificObjectSearch::SpecificObjectSearch(const rclcpp::NodeOptions& node_options) :
    rclcpp::Node("specific_object_search_node", node_options) {

    this->declare_parameter<std::string>("network_interface", "eth0");
    this->declare_parameter<std::string>("target_class", "red ball");
    this->declare_parameter<double>("conf_threshold", 0.5);
    this->declare_parameter<int>("video_timeout", 3);
    
    network_interface_ = this->get_parameter("network_interface").as_string();
    target_class_ = this->get_parameter("target_class").as_string();
    conf_threshold_ = this->get_parameter("conf_threshold").as_double();
    video_timeout_ = this->get_parameter("video_timeout").as_int();
    
    save_directory_ = "~/found_objects";
    
    if (!initializeVideoClient()) {
        RCLCPP_ERROR(this->get_logger(), "Не удалось инициализировать видео клиент Unitree");
        return;
    }
    
    this->detection_sub_ = 
        this->create_subscription<Detection2DArr>("detections",
                                                  rclcpp::SensorDataQoS().keep_last(1),
                                                  std::bind(&SpecificObjectSearch::detectionCallback,
                                                           this,
                                                           std::placeholders::_1));

    this->odom_sub_ = 
        this->create_subscription<Odometry>("/utlidar/robot_odom",
                                            rclcpp::SensorDataQoS().keep_last(1),
                                            std::bind(&SpecificObjectSearch::odomCallback,
                                                     this,
                                                     std::placeholders::_1));
    
    initial_pose_set_ = false;
    target_found_ = false;
}

SpecificObjectSearch::~SpecificObjectSearch() {
    if (video_client_) {
        video_client_.reset();
    }
}

bool SpecificObjectSearch::initializeVideoClient() {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
        
        video_client_ = std::make_unique<unitree::robot::go2::VideoClient>();
        video_client_->SetTimeout(static_cast<float>(video_timeout_));
        video_client_->Init();
        
        RCLCPP_INFO(this->get_logger(), "Видео клиент инициализирован чикипучно");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Ошибка инициализации видео клиента: %s", e.what());
        return false;
    }
}

void SpecificObjectSearch::odomCallback(const Odometry::ConstSharedPtr odom_msg) {
    current_odom_ = *odom_msg;
    
    if (!initial_pose_set_) {
        initial_odom_ = *odom_msg;
        initial_pose_set_ = true;
    }
}

void SpecificObjectSearch::detectionCallback(const Detection2DArr::ConstSharedPtr detect_msg) {
    std::vector<Detection2D> detections = detect_msg->detections;

    try {
        int ret = video_client_->GetImageSample(current_image_data_);
        
        if (ret != 0 || current_image_data_.empty()) {
            return;
        }
        
        cv::Mat frame = cv::imdecode(current_image_data_, cv::IMREAD_COLOR);

        if (frame.empty()) {
            return;
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Ошибка при полученииизображения: %s", e.what());
    }
    
    for (const auto& det : detections) {
        if (det.results.empty()) continue;
        
        std::string class_id = det.results[0].hypothesis.class_id;
        double score = det.results[0].hypothesis.score;
        
        if (class_id == target_class_ && score >= conf_threshold_) {
            
            overlayPositionInfo(frame);
            saveFrameWithInfo(frame);

            break;
        }
    }
}

void SpecificObjectSearch::overlayPositionInfo(cv::Mat& frame) {
    double rel_x = current_odom_.pose.pose.position.x - initial_odom_.pose.pose.position.x;
    double rel_y = current_odom_.pose.pose.position.y - initial_odom_.pose.pose.position.y;
    double rel_z = current_odom_.pose.pose.position.z - initial_odom_.pose.pose.position.z;
    
    auto& q = current_odom_.pose.pose.orientation;
    
    std::string object_text = "Object: " + target_class_;
    std::string position_text = "Position: x=" + std::to_string(rel_x).substr(0, 5) + 
                                " y=" + std::to_string(rel_y).substr(0, 5) + 
                                " z=" + std::to_string(rel_z).substr(0, 5);
    std::string quat_text = "Quat: x=" + std::to_string(q.x).substr(0, 4) + 
                           " y=" + std::to_string(q.y).substr(0, 4) + 
                           " z=" + std::to_string(q.z).substr(0, 4) + 
                           " w=" + std::to_string(q.w).substr(0, 4);
    
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.7;
    int thickness = 2;
    cv::Scalar color(0, 0, 255);
    int line_spacing = 30;
    
    int baseline;
    cv::Size text_size = cv::getTextSize(object_text, font_face, font_scale, thickness, &baseline);
    
    int start_x = frame.cols - text_size.width - 20;
    int start_y = 30;
    
    cv::Rect roi(start_x - 10, start_y - 25, text_size.width + 20, line_spacing * 3 + 10);
    cv::Mat overlay = frame.clone();
    cv::rectangle(overlay, roi, cv::Scalar(0, 0, 0), cv::FILLED);
    cv::addWeighted(overlay, 0.5, frame, 0.5, 0, frame);
    
    cv::putText(frame, object_text, 
                cv::Point(start_x, start_y), 
                font_face, font_scale, color, thickness);
    
    cv::putText(frame, position_text, 
                cv::Point(start_x, start_y + line_spacing), 
                font_face, font_scale, color, thickness);
    
    cv::putText(frame, quat_text, 
                cv::Point(start_x, start_y + line_spacing * 2), 
                font_face, font_scale, color, thickness);
}

void SpecificObjectSearch::saveFrameWithInfo(const cv::Mat& frame) {
    auto now = this->get_clock()->now();
    auto timestamp = std::to_string(now.seconds()) + "_" + 
                    std::to_string(now.nanoseconds() % 1000000000);
    
    std::string safe_class_name = target_class_;
    std::replace(safe_class_name.begin(), safe_class_name.end(), ' ', '_');
    
    std::string filename = save_directory_ + "/" + safe_class_name + "_" + timestamp + ".jpg";
    
    bool success = cv::imwrite(filename, frame);
    
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Фрейм сохранен: %s", filename.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Не удалось сохранить фрейм: %s", filename.c_str());
    }
}