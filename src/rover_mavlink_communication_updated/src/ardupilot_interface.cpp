#include "ardupilot_interface.hpp"

#include <functional>
#include <utility>

ArduPilotInterface::ArduPilotInterface(rclcpp::Node * node)
    : node_(node),
      connected_(false),
      armed_(false),
      current_mode_("UNKNOWN")
{
    velocity_publisher_ =
        node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10);

    state_subscriber_ =
        node_->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state",
        10,
        std::bind(
            &ArduPilotInterface::stateCallback,
            this,
            std::placeholders::_1));

    set_mode_client_ =
        node_->create_client<mavros_msgs::srv::SetMode>(
        "/mavros/set_mode");

    arming_client_ =
        node_->create_client<mavros_msgs::srv::CommandBool>(
        "/mavros/cmd/arming");

    command_client_ =
        node_->create_client<mavros_msgs::srv::CommandLong>(
        "/mavros/cmd/command");

    RCLCPP_INFO(node_->get_logger(), "ArduPilot interface initialized");
}

bool ArduPilotInterface::setMode(const std::string & mode)
{
    if (!connected_)
    {
        RCLCPP_WARN(node_->get_logger(), "Cannot request %s: FCU is disconnected", mode.c_str());
        return false;
    }

    if (!set_mode_client_->service_is_ready())
    {
        RCLCPP_WARN(node_->get_logger(), "MAVROS set_mode service is unavailable");
        return false;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = mode;

    set_mode_client_->async_send_request(
        request,
        [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future)
        {
            const auto response = future.get();
            if (response->mode_sent)
            {
                RCLCPP_INFO(node_->get_logger(), "Mode request sent: %s", mode.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Mode request rejected: %s", mode.c_str());
            }
        });

    return true;
}

bool ArduPilotInterface::arm(bool arm_vehicle)
{
    if (!connected_)
    {
        RCLCPP_WARN(node_->get_logger(), "Cannot change arming state: FCU is disconnected");
        return false;
    }

    if (!arming_client_->service_is_ready())
    {
        RCLCPP_WARN(node_->get_logger(), "MAVROS arming service is unavailable");
        return false;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm_vehicle;

    arming_client_->async_send_request(
        request,
        [this, arm_vehicle](
            rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future)
        {
            const auto response = future.get();
            if (!response->success)
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "%s command rejected, MAV_RESULT=%u",
                    arm_vehicle ? "ARM" : "DISARM",
                    response->result);
            }
        });

    return true;
}

bool ArduPilotInterface::setEkfSourceSet(std::uint8_t source_set)
{
    if (source_set < 1U || source_set > 3U)
    {
        RCLCPP_ERROR(node_->get_logger(), "Invalid EKF source set: %u", source_set);
        return false;
    }

    if (!connected_)
    {
        RCLCPP_WARN(node_->get_logger(), "Cannot switch EKF source: FCU is disconnected");
        return false;
    }

    if (!command_client_->service_is_ready())
    {
        RCLCPP_WARN(node_->get_logger(), "MAVROS CommandLong service is unavailable");
        return false;
    }

    auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    request->broadcast = false;
    request->command = 42007U;  // MAV_CMD_SET_EKF_SOURCE_SET
    request->confirmation = 0U;
    request->param1 = static_cast<float>(source_set);
    request->param2 = 0.0F;
    request->param3 = 0.0F;
    request->param4 = 0.0F;
    request->param5 = 0.0F;
    request->param6 = 0.0F;
    request->param7 = 0.0F;

    command_client_->async_send_request(
        request,
        [this, source_set](
            rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture future)
        {
            const auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(
                    node_->get_logger(),
                    "EKF source switched to SRC%u",
                    source_set);
            }
            else
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "EKF source switch rejected, MAV_RESULT=%u",
                    response->result);
            }
        });

    return true;
}

void ArduPilotInterface::setVelocity(
    double linear_velocity,
    double angular_velocity)
{
    if (!connected_)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Cannot send velocity: MAVROS is disconnected");
        return;
    }

    if (!armed_)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Cannot send velocity: rover is not armed");
        return;
    }

    if (!isGuided())
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Cannot send velocity: current mode is %s, GUIDED required",
            current_mode_.c_str());
        return;
    }

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = node_->now();
    command.header.frame_id = "base_link";
    command.twist.linear.x = linear_velocity;
    command.twist.angular.z = angular_velocity;
    publish(command);
}

void ArduPilotInterface::stop()
{
    if (!connected_)
    {
        return;
    }

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = node_->now();
    command.header.frame_id = "base_link";
    publish(command);

    RCLCPP_WARN(node_->get_logger(), "STOP command sent to ArduPilot");
}

void ArduPilotInterface::stateCallback(
    const mavros_msgs::msg::State::SharedPtr msg)
{
    const bool was_connected = connected_;
    const bool was_armed = armed_;
    const std::string previous_mode = current_mode_;

    connected_ = msg->connected;
    armed_ = msg->armed;
    current_mode_ = msg->mode;

    if (connected_ != was_connected)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "FCU connection: %s",
            connected_ ? "connected" : "disconnected");
    }

    if (armed_ != was_armed)
    {
        RCLCPP_INFO(node_->get_logger(), "Rover is now %s", armed_ ? "armed" : "disarmed");
    }

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
    if (current_mode_ == "GUIDED") return RoverControlMode::GUIDED;
    if (current_mode_ == "MANUAL") return RoverControlMode::MANUAL;
    if (current_mode_ == "HOLD") return RoverControlMode::HOLD;
    if (current_mode_ == "AUTO") return RoverControlMode::AUTO;
    return RoverControlMode::UNKNOWN;
}

bool ArduPilotInterface::isConnected() const
{
    return connected_;
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

void ArduPilotInterface::publish(
    const geometry_msgs::msg::TwistStamped & command)
{
    velocity_publisher_->publish(command);
}
