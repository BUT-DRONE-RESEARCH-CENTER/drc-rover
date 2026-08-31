#ifndef DRONE_FOLLOW__ARUCO_DETECTOR_HPP_
#define DRONE_FOLLOW__ARUCO_DETECTOR_HPP_

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <drone_follow/msg/aruco_marker.hpp>

namespace drone_follow {

class ArucoDetectorNode : public rclcpp::Node {
public:
    explicit ArucoDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    // Parametry uzlu
    double marker_size_{0.1};
    int dictionary_id_{0};

    // Vnitřní stav kalibrace
    bool camera_info_received_{false};
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // ROS 2 komunikace
    rclcpp::Publisher<drone_follow::msg::ArucoMarker>::SharedPtr aruco_marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace drone_follow

#endif  // DRONE_FOLLOW__ARUCO_DETECTOR_HPP_