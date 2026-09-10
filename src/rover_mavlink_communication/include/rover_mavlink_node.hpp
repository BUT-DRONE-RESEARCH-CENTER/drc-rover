#ifndef ROVER_MAVLINK_COMMUNICATION__ROVER_MAVLINK_NODE_HPP_
#define ROVER_MAVLINK_COMMUNICATION__ROVER_MAVLINK_NODE_HPP_

#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/state.hpp>

#include "ardupilot_interface.hpp"
#include "external_pose_publisher.hpp"


class RoverMavlinkNode : public rclcpp::Node
{
public:
    RoverMavlinkNode();

private:
    void velocityCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg);

    void heartbeatCallback();

    void localPositionCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    bool isPositionEstimateValid() const;

    // ROS subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        velocity_subscriber_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        local_position_subscriber_;

    // heartbeat timer
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;


    // ArduPilot interface
    std::shared_ptr<ArduPilotInterface> ardupilot_;

    std::shared_ptr<ExternalPosePublisher> external_pose_publisher_;


    // Time of last navigation command
    rclcpp::Time last_command_time_;
    rclcpp::Time last_local_position_time_{0, 0, RCL_ROS_TIME};


    // Parameters
    double max_linear_velocity_;
    double max_angular_velocity_;
    double heartbeat_timeout_;
    double local_position_timeout_{1.0};


    // State
    bool command_received_;
    bool heartbeat_triggered_;
    bool local_position_received_{false};
};

#endif