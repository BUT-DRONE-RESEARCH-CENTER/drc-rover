#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/landing_target.hpp>
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

        pub_landing_target_ = this->create_publisher<mavros_msgs::msg::LandingTarget>(
            "/mavros/landing_target/raw_data", 10);

        RCLCPP_INFO(this->get_logger(), "MAVROS Handler started. Tracking ID: %d", target_id_);
    }

private:
    // 3-line transformation: Camera optical frame -> Drone FRD frame
    geometry_msgs::msg::Point convertCameraToFRD(const geometry_msgs::msg::Point &cam) {
        geometry_msgs::msg::Point frd;
        frd.x = -cam.y;  // Forward (X_frd)
        frd.y =  cam.x;  // Right (Y_frd)
        frd.z =  cam.z;  // Down (Z_frd)
        return frd;
    }

    void arucoCallback(const drone_follow::msg::ArucoMarker::SharedPtr msg) {
        if (msg->id != target_id_) return;

        mavros_msgs::msg::LandingTarget landing_msg;
        landing_msg.header = msg->header;
        landing_msg.target_num = static_cast<uint8_t>(msg->id);
        landing_msg.type = 1; // Vision target

        // Apply 3-line position remapping
        landing_msg.pose.position = convertCameraToFRD(msg->pose.position);
        landing_msg.pose.orientation = msg->pose.orientation;

        pub_landing_target_->publish(landing_msg);
        RCLCPP_INFO(this->get_logger(), "LandingTarget sent | FRD: X=%.2f Y=%.2f Z=%.2f", 
                    landing_msg.pose.position.x, landing_msg.pose.position.y, landing_msg.pose.position.z);
    }

    int target_id_;
    rclcpp::Subscription<drone_follow::msg::ArucoMarker>::SharedPtr sub_aruco_;
    rclcpp::Publisher<mavros_msgs::msg::LandingTarget>::SharedPtr pub_landing_target_;
};

}  // namespace drone_follow

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<drone_follow::MavrosHandlerNode>());
    rclcpp::shutdown();
    return 0;
}