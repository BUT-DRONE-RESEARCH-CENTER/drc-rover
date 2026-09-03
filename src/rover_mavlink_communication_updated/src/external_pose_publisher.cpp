#include "external_pose_publisher.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

ExternalPosePublisher::ExternalPosePublisher(rclcpp::Node * node)
    : node_(node)
{
    global_frame_ = node_->declare_parameter<std::string>(
        "external_pose.global_frame", "map");
    base_frame_ = node_->declare_parameter<std::string>(
        "external_pose.base_frame", "base_link");
    output_topic_ = node_->declare_parameter<std::string>(
        "external_pose.output_topic", "/mavros/vision_pose/pose");
    publish_rate_ = node_->declare_parameter<double>(
        "external_pose.publish_rate", 20.0);

    if (publish_rate_ <= 0.0)
    {
        throw std::invalid_argument("external_pose.publish_rate must be greater than zero");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pose_publisher_ =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = node_->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ExternalPosePublisher::timerCallback, this));

    RCLCPP_INFO(
        node_->get_logger(),
        "External pose publisher: %s -> %s at %.1f Hz",
        global_frame_.c_str(),
        base_frame_.c_str(),
        publish_rate_);
}

void ExternalPosePublisher::timerCallback()
{
    geometry_msgs::msg::TransformStamped transform;

    try
    {
        transform = tf_buffer_->lookupTransform(
            global_frame_, base_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "Could not get TF %s -> %s: %s",
            global_frame_.c_str(),
            base_frame_.c_str(),
            ex.what());
        return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    pose_publisher_->publish(pose);
}
