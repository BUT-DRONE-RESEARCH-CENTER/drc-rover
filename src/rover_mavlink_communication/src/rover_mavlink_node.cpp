#include "../include/rover_mavlink_node.hpp"

#include <algorithm>
#include <functional>

RoverMavlinkNode::RoverMavlinkNode()
    : Node("rover_mavlink_node"),      
    command_received_(false),
    heartbeat_triggered_(false)
{
    // Parameters
    this->declare_parameter<double>("max_linear_velocity", 1.0);
    this->declare_parameter<double>("max_angular_velocity", 1.0);
    this->declare_parameter<double>("heartbeat_timeout", 5.0);

    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    heartbeat_timeout_ = this->get_parameter("heartbeat_timeout").as_double();


    // ========================================================
    // ArduPilot interface
    // ========================================================

    ardupilot_ = std::make_shared<ArduPilotInterface>(this);
    external_pose_publisher_ = std::make_shared<ExternalPosePublisher>(this);

    // Subscription to velocity commands from navigation

    velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(
                &RoverMavlinkNode::velocityCallback,
                this,
                std::placeholders::_1));

    // heartbeat timer - if there is no command received within the specified timeout, stop the rover
    heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
        std::bind(
            &RoverMavlinkNode::heartbeatCallback,
            this));

    RCLCPP_INFO(
        this->get_logger(),
        "Rover MAVLink communication node started");
}

void RoverMavlinkNode::velocityCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    geometry_msgs::msg::TwistStamped command;

    command.header.stamp = this->now();
    command.header.frame_id = "base_link";

    // Update heartbeat
    last_command_time_ = this->now();
    command_received_ = true;

    // If heartbeat was previously triggered,
    // mark communication as restored
    if (heartbeat_triggered_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Navigation command stream restored");

        heartbeat_triggered_ = false;
    }

    // ========================================================
    // Velocity limits
    // ========================================================

    // Normalize linear velocity to be within the specified limits (based on max_linear_velocity_)
    command.twist.linear.x =
        std::clamp(
            msg->linear.x,
            -max_linear_velocity_,
            max_linear_velocity_);

    command.twist.linear.y = 0.0;
    command.twist.linear.z = 0.0;


    // Limit steering / yaw rate
    command.twist.angular.x = 0.0;
    command.twist.angular.y = 0.0;

    // Normalize the angular velocity to be within the specified limits (based on max_angular_velocity_)
    command.twist.angular.z =
        std::clamp(
            msg->angular.z,
            -max_angular_velocity_,
            max_angular_velocity_);

    
    // ========================================================
    // Decide according to ArduPilot mode
    // ========================================================

    switch (ardupilot_->getControlMode())
    {
        case RoverControlMode::GUIDED:
            ardupilot_->setVelocity(
                command.twist.linear.x,
                command.twist.angular.z);

            // RCLCPP_DEBUG(
            //     this->get_logger(),
            //     "Command: velocity %.2f m/s | yaw rate %.2f rad/s",
            //     command.twist.linear.x,
            //     command.twist.angular.z);

            RCLCPP_INFO(
                this->get_logger(),
                "Received command: velocity %.2f m/s | yaw rate %.2f rad/s",
                command.twist.linear.x,
                command.twist.angular.z);
            break;

        case RoverControlMode::MANUAL:
            RCLCPP_DEBUG(
                this->get_logger(),
                "MANUAL mode - autonomous command ignored");
            break;

        case RoverControlMode::HOLD:
            RCLCPP_DEBUG(
                this->get_logger(),
                "HOLD mode - autonomous command ignored");
            break;

        case RoverControlMode::AUTO:
            RCLCPP_DEBUG(
                this->get_logger(),
                "AUTO mode - autonomous ROS command ignored");
            break;

        case RoverControlMode::UNKNOWN:
        default:
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Unknown ArduPilot mode - command ignored");
            break;
    }

}

void RoverMavlinkNode::heartbeatCallback()
{
    // Check if a command has been received
    if (!command_received_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "No navigation command received yet");
        return;
    }

    // Check if the time since the last command exceeds the heartbeat timeout
    rclcpp::Duration time_since_last_command = this->now() - last_command_time_;
    if (time_since_last_command.seconds() > heartbeat_timeout_)
    {
        if (!heartbeat_triggered_)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "heartbeat triggered! No navigation command received for %.2f seconds. Stopping the rover.",
                time_since_last_command.seconds());

            // Stop the rover
            ardupilot_->stop();

            heartbeat_triggered_ = true;
        }
    }
}
