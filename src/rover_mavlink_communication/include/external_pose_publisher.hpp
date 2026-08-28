#ifndef ROVER_MAVLINK_COMMUNICATION__EXTERNAL_POSE_PUBLISHER_HPP_
#define ROVER_MAVLINK_COMMUNICATION__EXTERNAL_POSE_PUBLISHER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>


class ExternalPosePublisher
{
public:
    explicit ExternalPosePublisher(rclcpp::Node * node);

private:
    void timerCallback();

    rclcpp::Node * node_;

    std::shared_ptr<tf2_ros::Buffer>
        tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener>
        tf_listener_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr
        pose_publisher_;

    rclcpp::TimerBase::SharedPtr
        timer_;

    std::string global_frame_;
    std::string base_frame_;

    double publish_rate_;
};

#endif