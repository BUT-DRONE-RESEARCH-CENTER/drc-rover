#ifndef ROVER_MAVLINK_COMMUNICATION__ARDUPILOT_INTERFACE_HPP_
#define ROVER_MAVLINK_COMMUNICATION__ARDUPILOT_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mode_control.hpp"

class ArduPilotInterface
{
public:
    explicit ArduPilotInterface(rclcpp::Node * node);

    bool setMode(const std::string & mode);
    bool arm(bool arm_vehicle);
    bool setEkfSourceSet(std::uint8_t source_set);

    void setVelocity(double linear_velocity, double angular_velocity);
    void stop();

    bool isConnected() const;
    bool isArmed() const;
    bool isGuided() const;
    RoverControlMode getControlMode() const;
    std::string getMode() const;

private:
    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
    void publish(const geometry_msgs::msg::TwistStamped & command);

    rclcpp::Node * node_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        velocity_publisher_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr
        state_subscriber_;

    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr
        set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr
        arming_client_;
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr
        command_client_;

    bool connected_;
    bool armed_;
    std::string current_mode_;
};

#endif  // ROVER_MAVLINK_COMMUNICATION__ARDUPILOT_INTERFACE_HPP_
