#include "../include/pc2_values_demonstrator.hpp"

PC2ValuesDemonstrator::PC2ValuesDemonstrator(const rclcpp::NodeOptions& options) : 
    rclcpp::Node("pc2_values_demonstrator_node", options) {

    this->pc2_sub_ = 
        this->create_subscription<PC2Type>("input", 
                                           rclcpp::SensorDataQoS().keep_last(1), 
                                           std::bind(&PC2ValuesDemonstrator::pc2Callback, 
                                                     this,
                                                     std::placeholders::_1));

}

void PC2ValuesDemonstrator::pc2Callback(const PC2Type::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    auto points = pcl_cloud.points;
    // double x = points.at(1000).x;
    // size_t sz = points.size();
    auto minmax_x = std::minmax_element(points.begin(), points.end(), [](pcl::PointXYZI p1, pcl::PointXYZI p2) {
                                                                          return p1.x < p2.x;
                                                                      });
    pcl::PointXYZI p1_x = *(minmax_x.first);
    pcl::PointXYZI p2_x = *(minmax_x.second);

    auto minmax_y = std::minmax_element(points.begin(), points.end(), [](pcl::PointXYZI p1, pcl::PointXYZI p2) {
                                                                          return p1.y < p2.y;
                                                                      });
    pcl::PointXYZI p1_y = *(minmax_y.first);
    pcl::PointXYZI p2_y = *(minmax_y.second);                                                                  

    RCLCPP_INFO(this->get_logger(), "GOT IT: (X) %f, %f / (Y) %f, %f", p1_x.x, p2_x.x, p1_y.y, p2_y.y);
    // RCLCPP_INFO(this->get_logger(), "GOT IT: %f", x);
}