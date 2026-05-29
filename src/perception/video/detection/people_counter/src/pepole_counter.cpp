#include "../include/people_counter.hpp"

PeopleCounter::PeopleCounter(const rclcpp::NodeOptions& node_options) :
    rclcpp::Node("people_counter_node", node_options) {

    this->image_sub_ = 
        this->create_subscription<CompressedImage>("raw_frame",
                                                   rclcpp::SensorDataQoS().keep_last(1),
                                                   std::bind(&PeopleCounter::imageCallback,
                                                            this,
                                                            std::placeholders::_1));


    this->detection_sub_ =
        this->create_subscription<Detection2DArr>("input",
                                                  rclcpp::SensorDataQoS().keep_last(1),
                                                  std::bind(&PeopleCounter::updatePersonCounter,
                                                            this,
                                                            std::placeholders::_1));

    
    this->image_pub_ = 
        this->create_publisher<Image>("output",
                                      rclcpp::QoS(1));
    
    this->conf_threshold_ = 
        this->declare_parameter<double>("conf_threshold", 0.2);
    
};

void PeopleCounter::imageCallback(const CompressedImage::ConstSharedPtr& img_msg) {
    // std::lock_guard<std::mutex> lock(image_mutex_);
    cv_image_ = img_msg;
}
    
void PeopleCounter::updatePersonCounter(const Detection2DArr::ConstSharedPtr detect_msg) {
    std::vector<Detection2D> det_arr = detect_msg->detections;
    // std::lock_guard<std::mutex> lock(image_mutex_);
    cv_bridge::CvImagePtr cv_ptr;

    if (!cv_image_) return;

    try {
        cv_ptr = cv_bridge::toCvCopy(cv_image_, "bgr8");
    } catch (cv_bridge::Exception& e) {
        return;
    }

    cv::Mat& frame = cv_ptr->image;

    current_det_ids_.clear();

    for (const auto& det : det_arr) {

        std::string track_id = det.id;
        
        if (det.results.empty()) continue;
        if (det.results[0].hypothesis.class_id != "person") continue;
        if (det.results[0].hypothesis.score < conf_threshold_) continue;
        if (track_id.empty()) continue;

        current_det_ids_.insert(track_id);

        if (counted_ids_.find(track_id) == counted_ids_.end()) {
            counted_ids_.insert(track_id);

            ++unique_persons_count_;
            RCLCPP_INFO(this->get_logger(), "Появления в кадре: %d", unique_persons_count_);
        }

        float cx = det.bbox.center.position.x;
        float cy = det.bbox.center.position.y;
        float w = det.bbox.size_x;
        float h = det.bbox.size_y;

        int x1 = std::max(0, static_cast<int>(cx - w / 2.0f));
        int y1 = std::max(0, static_cast<int>(cy - h / 2.0f));
        int x2 = std::min(frame.cols, static_cast<int>(cx + w / 2.0f));
        int y2 = std::min(frame.rows, static_cast<int>(cy + h / 2.0f));

        cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 100, 0), 3);

        std::string label = "person" + std::to_string(det.results[0].hypothesis.score);
        if (!track_id.empty()) label += "  ID: " + track_id;

        cv::putText(frame, label, cv::Point(x1 + 5, y1 + 20), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 100, 0), 2);
        
    }

    image_pub_->publish(*cv_ptr->toImageMsg());

    for (auto it = counted_ids_.begin(); it != counted_ids_.end();) {
        if (current_det_ids_.find(*it) == current_det_ids_.end()) {
            it = counted_ids_.erase(it);
        } else {
            ++it;
        }
    }


}