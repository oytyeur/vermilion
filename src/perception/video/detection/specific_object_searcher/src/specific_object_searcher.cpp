#include "../include/specific_object_searcher.hpp"

SpecificObjectSearcher::SpecificObjectSearcher(const rclcpp::NodeOptions& node_options) :
    rclcpp::Node("specific_object_searcher_node", node_options) {

    this->declare_parameter<std::string>("network_interface", "eth0");
    this->declare_parameter<std::string>("target_class", "red ball");
    this->declare_parameter<double>("conf_threshold", 0.5);
    this->declare_parameter<int>("video_timeout", 3);
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<std::string>("odom_frame", "odom");
    
    network_interface_ = this->get_parameter("network_interface").as_string();
    target_class_ = this->get_parameter("target_class").as_string();
    conf_threshold_ = this->get_parameter("conf_threshold").as_double();
    video_timeout_ = this->get_parameter("video_timeout").as_int();
    base_frame_ = this->get_parameter("base_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    
    save_directory_ = "~/found_objects";
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    if (!initializeVideoClient()) {
        RCLCPP_ERROR(this->get_logger(), "Не удалось инициализировать видео клиент sdk");
        return;
    }
    
    this->detection_sub_ = 
        this->create_subscription<Detection2DArr>("detections",
                                                  rclcpp::SensorDataQoS().keep_last(1),
                                                  std::bind(&SpecificObjectSearcher::detectionCallback,
                                                           this,
                                                           std::placeholders::_1));
    
    initial_pose_set_ = false;
    target_found_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Нода SpecificObjectSearcher запущена чикипучно");
}

SpecificObjectSearcher::~SpecificObjectSearcher() {
    if (video_client_) {
        video_client_.reset();
    }
}

bool SpecificObjectSearcher::initializeVideoClient() {
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
        
        video_client_ = std::make_unique<unitree::robot::go2::VideoClient>();
        video_client_->SetTimeout(static_cast<float>(video_timeout_));
        video_client_->Init();
        
        RCLCPP_INFO(this->get_logger(), "Видео клиент sdk инициализирован чикипучно");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Ошибка инициализации видео клиента: %s", e.what());
        return false;
    }
}

bool SpecificObjectSearcher::getCurrentTransform(geometry_msgs::msg::TransformStamped& transform) {
    try {
        transform = tf_buffer_->lookupTransform(
            odom_frame_, 
            base_frame_,
            tf2::TimePointZero);
        return true;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Не удалось получить TF: %s", ex.what());
        return false;
    }
}

void SpecificObjectSearcher::detectionCallback(const Detection2DArr::ConstSharedPtr detect_msg) {
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
        
        for (const auto& det : detections) {
            if (det.results.empty()) continue;
            
            std::string class_id = det.results[0].hypothesis.class_id;
            double score = det.results[0].hypothesis.score;
            
            if (class_id == target_class_ && score >= conf_threshold_) {
                
                RCLCPP_INFO(this->get_logger(), "Обнаружен %s с уверенностью %.3f!", 
                            target_class_.c_str(), score);
                
                geometry_msgs::msg::TransformStamped current_transform;
                if (!getCurrentTransform(current_transform)) {
                    RCLCPP_WARN(this->get_logger(), "Не удалось получить текущую позицию через TF");
                    return;
                }
                
                if (!initial_pose_set_) {
                    initial_transform_ = current_transform;
                    initial_pose_set_ = true;
                    
                    RCLCPP_INFO(this->get_logger(), "Начальная позиция установлена чикипучно");
                    RCLCPP_INFO(this->get_logger(), "Позиция: x=%.3f, y=%.3f, z=%.3f", 
                                initial_transform_.transform.translation.x,
                                initial_transform_.transform.translation.y,
                                initial_transform_.transform.translation.z);
                    RCLCPP_INFO(this->get_logger(), "Ориентация: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                                initial_transform_.transform.rotation.x,
                                initial_transform_.transform.rotation.y,
                                initial_transform_.transform.rotation.z,
                                initial_transform_.transform.rotation.w);
                }
                
                overlayPositionInfo(frame);
                saveFrameWithInfo(frame);
                
                if (!target_found_) {
                    target_found_ = true;
                }
                
                break;
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Ошибка при получении изображения: %s", e.what());
    }
}

void SpecificObjectSearcher::overlayPositionInfo(cv::Mat& frame) {
    geometry_msgs::msg::TransformStamped current_transform;
    if (!getCurrentTransform(current_transform)) {
        RCLCPP_WARN(this->get_logger(), "Не удалось получить позицию для фрейма");
        return;
    }
    
    double rel_x = current_transform.transform.translation.x - initial_transform_.transform.translation.x;
    double rel_y = current_transform.transform.translation.y - initial_transform_.transform.translation.y;
    double rel_z = current_transform.transform.translation.z - initial_transform_.transform.translation.z;
    
    auto& q = current_transform.transform.rotation;
    
    std::string object_text = "Object: " + target_class_;
    
    char pos_buffer[100];
    snprintf(pos_buffer, sizeof(pos_buffer), "Pos: x=%.2f y=%.2f z=%.2f", rel_x, rel_y, rel_z);
    std::string position_text = pos_buffer;
    
    char quat_buffer[100];
    snprintf(quat_buffer, sizeof(quat_buffer), "Quat: x=%.2f y=%.2f z=%.2f w=%.2f", 
             q.x, q.y, q.z, q.w);
    std::string quat_text = quat_buffer;
    
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

void SpecificObjectSearcher::saveFrameWithInfo(const cv::Mat& frame) {
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