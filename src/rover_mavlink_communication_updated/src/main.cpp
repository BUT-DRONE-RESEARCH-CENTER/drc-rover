#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rover_mavlink_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverMavlinkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
