#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Rf2oMavrosBridge : public rclcpp::Node
{
public:
  Rf2oMavrosBridge()
  : Node("rf2o_mavros_bridge"),
    last_velocity_time_(now()),
    last_odom_publish_time_(now())
  {
    const auto rf2o_topic = declare_parameter<std::string>("rf2o_topic", "/odom_rf2o");
    const auto mavros_odom_topic =
      declare_parameter<std::string>("mavros_odom_topic", "/mavros/odometry/out");
    const auto origin_topic =
      declare_parameter<std::string>("origin_topic", "/mavros/global_position/set_gp_origin");
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

    // Omezení frekvence pro odlehčení ELRS linky
    target_odom_rate_hz_ = declare_parameter<double>("target_odom_rate_hz", 6.0);

    // Souřadnice pro EKF Origin (výchozí: Brno)
    origin_lat_ = declare_parameter<double>("origin_lat", 49.22836);
    origin_lon_ = declare_parameter<double>("origin_lon", 16.57265);
    origin_alt_ = declare_parameter<double>("origin_alt", 250.0);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(mavros_odom_topic, 10);
    velocity_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(mavros_velocity_topic, 10);
    origin_pub_ =
      create_publisher<geographic_msgs::msg::GeoPointStamped>(origin_topic, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      rf2o_topic, rclcpp::SensorDataQoS(),
      std::bind(&Rf2oMavrosBridge::odometryCallback, this, std::placeholders::_1));

    velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      velocity_input_topic, 10,
      std::bind(&Rf2oMavrosBridge::velocityCallback, this, std::placeholders::_1));

    watchdog_timer_ = create_wall_timer(50ms, std::bind(&Rf2oMavrosBridge::watchdog, this));
    
    // Periodické odesílání Originu (1 Hz)
    origin_timer_ = create_wall_timer(1s, std::bind(&Rf2oMavrosBridge::publishOrigin, this));

    RCLCPP_INFO(
      get_logger(),
      "RF2O %s -> %s (max %.1f Hz); velocity %s -> %s; Origin auto-pub active (1 Hz)",
      rf2o_topic.c_str(), mavros_odom_topic.c_str(), target_odom_rate_hz_,
      velocity_input_topic.c_str(), mavros_velocity_topic.c_str());
  }

private:
  void publishOrigin()
  {
    geographic_msgs::msg::GeoPointStamped origin;
    // Nulové razítko zabrání odmítnutí zprávy kvůli rozdílu hodin
    origin.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    origin.header.frame_id = "map";
    origin.position.latitude = origin_lat_;
    origin.position.longitude = origin_lon_;
    origin.position.altitude = origin_alt_;
    origin_pub_->publish(origin);
  }

  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr input)
  {
    const auto current_time = now();
    const double min_interval =
      (target_odom_rate_hz_ > 0.0) ? (1.0 / target_odom_rate_hz_) : 0.0;

    // Rate limiter pro ochranu ELRS linky
    if (min_interval > 0.0 && (current_time - last_odom_publish_time_).seconds() < min_interval) {
      return;
    }
    last_odom_publish_time_ = current_time;

    auto output = *input;

    // KLÍČOVÉ: Vynulování časové značky. MAVROS pošle time_usec = 0,
    // což Cube přinutí orazítkovat zprávu vlastním palubním časem
    output.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    output.header.frame_id = odom_frame_id_;
    output.child_frame_id = child_frame_id_;

    // Vyplnění nenulové diagonály kovariance 6x6
    std::array<double, 36> pose_cov = {0.0};
    pose_cov[0]  = 0.01;  // X var [m^2]
    pose_cov[7]  = 0.01;  // Y var [m^2]
    pose_cov[14] = 0.10;  // Z var [m^2]
    pose_cov[21] = 0.05;  // Roll var [rad^2]
    pose_cov[28] = 0.05;  // Pitch var [rad^2]
    pose_cov[35] = 0.02;  // Yaw var [rad^2]
    output.pose.covariance = pose_cov;

    std::array<double, 36> twist_cov = {0.0};
    twist_cov[0]  = 0.02; // Vx
    twist_cov[7]  = 0.02; // Vy
    twist_cov[14] = 0.10; // Vz
    twist_cov[21] = 0.05; // dRoll
    twist_cov[28] = 0.05; // dPitch
    twist_cov[35] = 0.05; // dYaw
    output.twist.covariance = twist_cov;

    odom_pub_->publish(output);
  }

  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr input)
  {
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = now();
    output.header.frame_id = velocity_frame_id_;

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
  double target_odom_rate_hz_{6.0};

  double origin_lat_{49.22836};
  double origin_lon_{16.57265};
  double origin_alt_{250.0};

  bool velocity_received_{false};
  bool stop_sent_{false};
  rclcpp::Time last_velocity_time_;
  rclcpp::Time last_odom_publish_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr origin_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr origin_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rf2oMavrosBridge>());
  rclcpp::shutdown();
  return 0;
}
