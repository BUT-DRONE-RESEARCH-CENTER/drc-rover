#ifndef ROVER_MAVLINK_COMMUNICATION__ARDUPILOT_INTERFACE_HPP_
#define ROVER_MAVLINK_COMMUNICATION__ARDUPILOT_INTERFACE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include "mode_control.hpp"

class ArduPilotInterface
{
public:
    explicit ArduPilotInterface(rclcpp::Node * node);
    RoverControlMode getControlMode() const;

    void setVelocity(
        double linear_velocity,
        double angular_velocity);

    void stop();

    bool isConnected() const;
    bool isArmed() const;
    bool isGuided() const;

    std::string getMode() const;

private:
    void stateCallback(
        const mavros_msgs::msg::State::SharedPtr msg);

    void publish(
        const geometry_msgs::msg::TwistStamped & command);

    rclcpp::Node * node_;

    rclcpp::Publisher<
        geometry_msgs::msg::TwistStamped>::SharedPtr
        velocity_publisher_;

    rclcpp::Subscription<
        mavros_msgs::msg::State>::SharedPtr
        state_subscriber_;

    bool connected_;
    bool armed_;

    std::string current_mode_;
};

#endif