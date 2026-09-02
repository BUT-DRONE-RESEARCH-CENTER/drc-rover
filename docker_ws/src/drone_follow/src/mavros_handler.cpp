#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/landing_target.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "drone_follow/msg/aruco_marker.hpp"

namespace drone_follow {

class MavrosHandlerNode : public rclcpp::Node {
public:
    MavrosHandlerNode() : Node("mavros_handler_node") {
        this->declare_parameter("target_id", 2);
        target_id_ = this->get_parameter("target_id").as_int();

        sub_aruco_ = this->create_subscription<drone_follow::msg::ArucoMarker>(
            "/aruco_marker", 10,
            std::bind(&MavrosHandlerNode::arucoCallback, this, std::placeholders::_1));

        // Oficiální MAVROS topik pro LandingTarget
        pub_landing_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/landing_target/pose", 10);

        RCLCPP_INFO(this->get_logger(), "MAVROS Handler running. Tracking ID: %d", target_id_);
    }

private:
    geometry_msgs::msg::Point convertCameraToFRD(const geometry_msgs::msg::Point &cam) {
        geometry_msgs::msg::Point frd;
        frd.x = -cam.y;  // Forward (X_frd)
        frd.y =  cam.x;  // Right (Y_frd)
        frd.z =  -cam.z;  // Down (Z_frd)
        return frd;
    }

    void arucoCallback(const drone_follow::msg::ArucoMarker::SharedPtr msg) {
        if (msg->id != target_id_) return;

        geometry_msgs::msg::Point frd_point = convertCameraToFRD(msg->pose.position);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "landing_target_0"; // Poslední znak '0' určí target_num = 0!

        pose_msg.pose.position.x = frd_point.x;
        pose_msg.pose.position.y = frd_point.y;
        pose_msg.pose.position.z = frd_point.z;

        pose_msg.pose.orientation = msg->pose.orientation; // Ponecháme orientaci z ArUco detekce

        pub_landing_pose_->publish(pose_msg);

        RCLCPP_INFO(this->get_logger(), "Published landing target pose for ID: %d [%.2f, %.2f, %.2f]", msg->id, pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
    }

    int target_id_;
    rclcpp::Subscription<drone_follow::msg::ArucoMarker>::SharedPtr sub_aruco_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_landing_pose_;
};

}  // namespace drone_follow

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<drone_follow::MavrosHandlerNode>());
    rclcpp::shutdown();
    return 0;
}