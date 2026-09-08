#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Rf2oMavrosBridge : public rclcpp::Node
{
public:
  Rf2oMavrosBridge()
  : Node("rf2o_mavros_bridge"), last_velocity_time_(now())
  {
    const auto rf2o_topic = declare_parameter<std::string>("rf2o_topic", "/odom_rf2o");
    const auto mavros_odom_topic =
      declare_parameter<std::string>("mavros_odom_topic", "/mavros/odometry/in");
    const auto velocity_input_topic =
      declare_parameter<std::string>("velocity_input_topic", "/cmd_vel");
    const auto mavros_velocity_topic = declare_parameter<std::string>(
      "mavros_velocity_topic", "/mavros/setpoint_velocity/cmd_vel");

    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    velocity_frame_id_ = declare_parameter<std::string>("velocity_frame_id", "base_link");
    max_linear_velocity_ =
      declare_parameter<double>("max_linear_velocity", 1.0);
    max_angular_velocity_ =
      declare_parameter<double>("max_angular_velocity", 1.0);
    watchdog_timeout_ = declare_parameter<double>("watchdog_timeout", 0.5);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(mavros_odom_topic, 10);
    velocity_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(mavros_velocity_topic, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      rf2o_topic, rclcpp::SensorDataQoS(),
      std::bind(&Rf2oMavrosBridge::odometryCallback, this, std::placeholders::_1));

    velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      velocity_input_topic, 10,
      std::bind(&Rf2oMavrosBridge::velocityCallback, this, std::placeholders::_1));

    watchdog_timer_ = create_wall_timer(50ms, std::bind(&Rf2oMavrosBridge::watchdog, this));

    RCLCPP_INFO(
      get_logger(), "RF2O %s -> %s; velocity %s -> %s",
      rf2o_topic.c_str(), mavros_odom_topic.c_str(), velocity_input_topic.c_str(),
      mavros_velocity_topic.c_str());
  }

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr input)
  {
    auto output = *input;
    output.header.frame_id = odom_frame_id_;
    output.child_frame_id = child_frame_id_;
    odom_pub_->publish(output);
  }

  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr input)
  {
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = now();
    output.header.frame_id = velocity_frame_id_;

    // Differential rover command: v = linear.x, w = angular.z.
    output.twist.linear.x = std::clamp(
      input->linear.x, -max_linear_velocity_, max_linear_velocity_);
    output.twist.angular.z = std::clamp(
      input->angular.z, -max_angular_velocity_, max_angular_velocity_);

    velocity_pub_->publish(output);
    last_velocity_time_ = now();
    velocity_received_ = true;
    stop_sent_ = false;
  }

  void watchdog()
  {
    if (!velocity_received_ || stop_sent_ || watchdog_timeout_ <= 0.0) {
      return;
    }

    if ((now() - last_velocity_time_).seconds() <= watchdog_timeout_) {
      return;
    }

    geometry_msgs::msg::TwistStamped stop;
    stop.header.stamp = now();
    stop.header.frame_id = velocity_frame_id_;
    velocity_pub_->publish(stop);
    stop_sent_ = true;
    RCLCPP_WARN(get_logger(), "Velocity watchdog timed out; zero command published");
  }

  std::string odom_frame_id_;
  std::string child_frame_id_;
  std::string velocity_frame_id_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double watchdog_timeout_;
  bool velocity_received_{false};
  bool stop_sent_{false};
  rclcpp::Time last_velocity_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rf2oMavrosBridge>());
  rclcpp::shutdown();
  return 0;
}
