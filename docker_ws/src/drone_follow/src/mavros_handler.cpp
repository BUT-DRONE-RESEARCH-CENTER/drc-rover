#include <memory>
#include <cmath>
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

        // Oficiální MAVROS topik pro LandingTarget
        pub_landing_target_ = this->create_publisher<mavros_msgs::msg::LandingTarget>(
            "/mavros/landing_target/raw", 10);

        RCLCPP_INFO(this->get_logger(), "MAVROS Handler running. Tracking ID: %d", target_id_);
    }

private:
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

        // 1. Nastavení rámce MAV_FRAME_BODY_FRD (12) přímo pro ArduPilot
        landing_msg.frame = 12; // 12 = MAV_FRAME_BODY_FRD
        landing_msg.type = mavros_msgs::msg::LandingTarget::VISION_FIDUCIAL; // 2 = VISION_FIDUCIAL

        // 2. Převod pozice do tělesného rámce dronu (Forward-Right-Down)
        geometry_msgs::msg::Point frd = convertCameraToFRD(msg->pose.position);
        landing_msg.pose.position = frd;
        landing_msg.pose.orientation = msg->pose.orientation;

        // 3. Výpočet vzdálenosti (POVINNÉ pro ArduPilot 3D mode)
        float dist = static_cast<float>(
            std::sqrt(frd.x * frd.x + frd.y * frd.y + frd.z * frd.z));
        landing_msg.distance = dist;

        // Úhly vynulujeme (ArduPilot je díky position_valid=1 v MAVROS ignoruje)
        landing_msg.angle[0] = 0.0f;
        landing_msg.angle[1] = 0.0f;

        pub_landing_target_->publish(landing_msg);
        RCLCPP_INFO(this->get_logger(), "Target 3D sent | Frame: 12 | Dist: %.2f m", dist);
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