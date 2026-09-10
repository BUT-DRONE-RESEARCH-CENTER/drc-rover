#ifndef ROVER_MAVLINK_COMMUNICATION__ROVER_MAVLINK_NODE_HPP_
#define ROVER_MAVLINK_COMMUNICATION__ROVER_MAVLINK_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
// #include <mavros_msgs/msg/estimator_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "ardupilot_interface.hpp"
#include "external_pose_publisher.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>


class RoverMavlinkNode : public rclcpp::Node
{
public:
    RoverMavlinkNode();

private:
    enum class AutonomousState
    {
        IDLE,
        REQUEST_GUIDED,
        WAIT_FOR_GUIDED,
        // WAIT_FOR_ESTIMATOR,
        CHECK_EKF,
        WAIT_FOR_RF2O,
        ENABLE_EXTERNAL_NAV,
        WAIT_FOR_EKF,
        CHECK_ARMED,
        REQUEST_ARM,
        WAIT_FOR_ARM,
        ACTIVE,
        REQUEST_MANUAL,
        FAILED
    };

    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void autonomousCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void rf2oCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void heartbeatCallback();
    void autonomousStateCallback();

    bool isAutonomousRequested() const;
    bool isRf2oHealthy() const;
    bool isRf2oMessageFinite(const nav_msgs::msg::Odometry & msg) const;
    
    void failAutonomous(const std::string & reason);

    bool isPositionEstimateValid() const;
    void localPositionCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomous_subscriber_;
    // rclcpp::Subscription<mavros_msgs::msg::EstimatorStatus>::SharedPtr estimator_status_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr rf2o_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mavros_odometry_publisher_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_subscriber_;


    rclcpp::Time last_local_position_time_{0, 0, RCL_ROS_TIME};
    
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr autonomous_timer_;

    std::shared_ptr<ArduPilotInterface> ardupilot_;
    std::shared_ptr<ExternalPosePublisher> external_pose_publisher_;

    rclcpp::Time last_command_time_;
    rclcpp::Time last_rf2o_time_;
    rclcpp::Time state_deadline_;

    

    double max_linear_velocity_;
    double max_angular_velocity_;
    double heartbeat_timeout_;
    double rf2o_timeout_;
    double guided_timeout_;
    double ekf_timeout_;
    double arm_timeout_;
    double local_position_timeout_{1.0};

    int max_arm_attempts_;
    int arm_attempts_;

    bool auto_arm_;
    bool autonomous_enabled_;
    bool command_received_;
    bool heartbeat_triggered_;
    // bool estimator_status_received_;
    // bool ekf_valid_;
    bool rf2o_received_;
    bool forward_rf2o_to_mavros_;
    bool external_nav_selected_;
    bool local_position_received_{false};

    std::string rf2o_topic_;
    std::string mavros_odometry_topic_;

    AutonomousState autonomous_state_;
};

#endif  // ROVER_MAVLINK_COMMUNICATION__ROVER_MAVLINK_NODE_HPP_
