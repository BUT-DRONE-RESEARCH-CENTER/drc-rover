#include "../include/ardupilot_interface.hpp"

ArduPilotInterface::ArduPilotInterface(rclcpp::Node * node)
    : node_(node),
      connected_(false),
      armed_(false),
      current_mode_("UNKNOWN")
{
    velocity_publisher_ =
        node_->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel",
            10);

    state_subscriber_ =
        node_->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state",
            10,
            std::bind(
                &ArduPilotInterface::stateCallback,
                this,
                std::placeholders::_1));

    gps_subscriber_ =
        node_->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/mavros/global_position/global",
            10,
            std::bind(
                &ArduPilotInterface::gpsCallback,
                this,
                std::placeholders::_1));

    RCLCPP_INFO(
        node_->get_logger(),
        "ArduPilot interface initialized");
}



void ArduPilotInterface::setVelocity(
    double linear_velocity,
    double angular_velocity)
{
    if (!connected_)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "Cannot send velocity command: MAVROS is not connected");

        return;
    }

    if (!armed_)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "Cannot send velocity command: rover is not armed");

        return;
    }

    if (!isGuided())
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "Velocity command ignored: current mode is %s, GUIDED required",
            current_mode_.c_str());

        return;
    }


    geometry_msgs::msg::TwistStamped command;

    command.header.stamp = node_->now();
    command.header.frame_id = "base_link";

    command.twist.linear.x = linear_velocity;
    command.twist.linear.y = 0.0;
    command.twist.linear.z = 0.0;

    command.twist.angular.x = 0.0;
    command.twist.angular.y = 0.0;
    command.twist.angular.z = angular_velocity;

    publish(command);
}

void ArduPilotInterface::stop()
{

    if (!connected_)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "Cannot stop rover: MAVROS is not connected");

        return;
    }

    geometry_msgs::msg::TwistStamped command;

    command.header.stamp = node_->now();
    command.header.frame_id = "base_link";

    command.twist.linear.x = 0.0;
    command.twist.linear.y = 0.0;
    command.twist.linear.z = 0.0;

    command.twist.angular.x = 0.0;
    command.twist.angular.y = 0.0;
    command.twist.angular.z = 0.0;

    velocity_publisher_->publish(command);

    RCLCPP_WARN(
        node_->get_logger(),
        "STOP command sent to ArduPilot");
}

void ArduPilotInterface::stateCallback(
    const mavros_msgs::msg::State::SharedPtr msg)
{
    const std::string previous_mode = current_mode_;

    connected_ = msg->connected;
    armed_ = msg->armed;
    current_mode_ = msg->mode;

    if (current_mode_ != previous_mode)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "ArduPilot mode changed: %s -> %s",
            previous_mode.c_str(),
            current_mode_.c_str());
    }
}

RoverControlMode ArduPilotInterface::getControlMode() const
{
    if (current_mode_ == "GUIDED")
    {
        return RoverControlMode::GUIDED;
    }

    if (current_mode_ == "MANUAL")
    {
        return RoverControlMode::MANUAL;
    }

    if (current_mode_ == "HOLD")
    {
        return RoverControlMode::HOLD;
    }

    if (current_mode_ == "AUTO")
    {
        return RoverControlMode::AUTO;
    }

    return RoverControlMode::UNKNOWN;
}

bool ArduPilotInterface::isConnected() const
{
    return connected_;
}

bool ArduPilotInterface::isAutonomous() const
{
    isAutonomous_ == true;
    return current_mode_ == "AUTO";
}

bool ArduPilotInterface::isArmed() const
{
    return armed_;
}


bool ArduPilotInterface::isGuided() const
{
    return current_mode_ == "GUIDED";
}


std::string ArduPilotInterface::getMode() const
{
    return current_mode_;
}

void ArduPilotInterface::publish(const geometry_msgs::msg::TwistStamped &command)
{
    velocity_publisher_->publish(command);
}