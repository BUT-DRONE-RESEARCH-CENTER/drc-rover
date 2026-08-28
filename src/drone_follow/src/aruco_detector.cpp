#include "drone_follow/aruco_detector.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace drone_follow {

ArucoDetectorNode::ArucoDetectorNode(const rclcpp::NodeOptions & options)
: Node("aruco_detector_node", options) {
    this->declare_parameter("marker_size", 0.);
    this->declare_parameter("dictionary_id", static_cast<int>(cv::aruco::DICT_4X4_250));

    marker_size_ = this->get_parameter("marker_size").as_double();
    dictionary_id_ = this->get_parameter("dictionary_id").as_int();

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/aruco_pose", 10);

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", 10,
        std::bind(&ArucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10,
        std::bind(&ArucoDetectorNode::imageCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Aruco Detector Node spuštěn.");
}

void ArucoDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (camera_info_received_) return;

    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
    dist_coeffs_ = cv::Mat(1, msg->d.size(), CV_64F, const_cast<double*>(msg->d.data())).clone();
    camera_info_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Kalibrace kamery úspěšně načtena.");
}

void ArucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Čekám na kalibraci kamery...");
        return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge chyba: %s", e.what());
        return;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_id_);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;

    cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);

    if (!ids.empty()) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

        for (size_t i = 0; i < ids.size(); ++i) {
            double distance = cv::norm(tvecs[i]);

            cv::Mat rot_matrix;
            cv::Rodrigues(rvecs[i], rot_matrix);

            tf2::Matrix3x3 tf2_rot(
                rot_matrix.at<double>(0, 0), rot_matrix.at<double>(0, 1), rot_matrix.at<double>(0, 2),
                rot_matrix.at<double>(1, 0), rot_matrix.at<double>(1, 1), rot_matrix.at<double>(1, 2),
                rot_matrix.at<double>(2, 0), rot_matrix.at<double>(2, 1), rot_matrix.at<double>(2, 2)
            );
            
            tf2::Quaternion q;
            tf2_rot.getRotation(q);

            // Publikování PoseStamped
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = msg->header;
            pose_msg.pose.position.x = tvecs[i][0];
            pose_msg.pose.position.y = tvecs[i][1];
            pose_msg.pose.position.z = tvecs[i][2];
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();
            pose_pub_->publish(pose_msg);

            // Publikování TF transformace
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

            RCLCPP_INFO(this->get_logger(), "Marker ID [%d] | Vzdálenost: %.3f m", ids[i], distance);
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