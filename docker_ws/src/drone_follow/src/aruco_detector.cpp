#include "drone_follow/aruco_detector.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace drone_follow {

ArucoDetectorNode::ArucoDetectorNode(const rclcpp::NodeOptions & options)
: Node("aruco_detector_node", options) {
    // Declare and retrieve node parameters
    this->declare_parameter("marker_size", 0.138);
    this->declare_parameter("dictionary_id", static_cast<int>(cv::aruco::DICT_4X4_250));

    marker_size_ = this->get_parameter("marker_size").as_double();
    dictionary_id_ = this->get_parameter("dictionary_id").as_int();

    // Initialize Publishers and Subscribers
    aruco_marker_pub_ = this->create_publisher<drone_follow::msg::ArucoMarker>("/aruco_marker", 10);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", 10,
        std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10,
        std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1));

    // Initialize dynamic TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "ArUco Detector Node initialized successfully.");
}

void ArucoDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    // Read camera calibration matrix (K) and distortion coefficients (D) only once
    if (camera_info_received_) return;

    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    dist_coeffs_ = cv::Mat(1, msg->d.size(), CV_64F, const_cast<double*>(msg->d.data())).clone();
    camera_info_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Camera calibration parameters successfully loaded.");
}

void ArucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    // Throttle warning if image callback fires before calibration data arrives
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for camera calibration data...");
        return;
    }

    // Convert ROS Image message to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Predefined ArUco dictionary lookup
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    // Detect markers in the image frame
    cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);

    if (!ids.empty()) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); ++i) {
            // Calculate Euclidean distance from camera optical frame
            double distance = cv::norm(tvecs[i]);

            // Convert Rodrigues rotation vector (rvec) to 3x3 rotation matrix
            cv::Mat rot_matrix;
            cv::Rodrigues(rvecs[i], rot_matrix);

            // Convert 3x3 rotation matrix to TF2 matrix format
            tf2::Matrix3x3 tf2_rot(
                rot_matrix.at<double>(0, 0), rot_matrix.at<double>(0, 1), rot_matrix.at<double>(0, 2),
                rot_matrix.at<double>(1, 0), rot_matrix.at<double>(1, 1), rot_matrix.at<double>(1, 2),
                rot_matrix.at<double>(2, 0), rot_matrix.at<double>(2, 1), rot_matrix.at<double>(2, 2)
            );
            
            // Extract Quaternion from rotation matrix
            tf2::Quaternion q;
            tf2_rot.getRotation(q);

            // 1. Populate custom ArucoMarker message
            drone_follow::msg::ArucoMarker aruco_marker_msg;
            aruco_marker_msg.header = msg->header;
            aruco_marker_msg.id = ids[i];
            aruco_marker_msg.distance = distance;
            aruco_marker_msg.pose.position.x = tvecs[i][0];
            aruco_marker_msg.pose.position.y = tvecs[i][1];
            aruco_marker_msg.pose.position.z = tvecs[i][2];
            aruco_marker_msg.pose.orientation.x = q.x();
            aruco_marker_msg.pose.orientation.y = q.y();
            aruco_marker_msg.pose.orientation.z = q.z();
            aruco_marker_msg.pose.orientation.w = q.w();
            aruco_marker_pub_->publish(aruco_marker_msg);

            // 2. Broadcast dynamic TF transform (Camera -> Marker)
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header = msg->header;
            tf_msg.child_frame_id = "aruco_marker_" + std::to_string(ids[i]);
            tf_msg.transform.translation.x = tvecs[i][0];
            tf_msg.transform.translation.y = tvecs[i][1];
            tf_msg.transform.translation.z = tvecs[i][2];
            tf_msg.transform.rotation.x = q.x();
            tf_msg.transform.rotation.y = q.y();
            tf_msg.transform.rotation.z = q.z();
            tf_msg.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(tf_msg);

            RCLCPP_INFO(this->get_logger(), "Detected Marker ID [%d] | Distance: %.3f m", ids[i], distance);
        }
    }
}

}  // namespace drone_follow

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<drone_follow::ArucoDetectorNode>());
    rclcpp::shutdown();
    return 0;
}