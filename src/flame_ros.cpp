#include <flame_ros/flame_ros.hpp>

namespace flame_ros {

FlameRos::FlameRos(const rclcpp::NodeOptions & options) : Node("flame", options) {
    RCLCPP_INFO(get_logger(), "flame_ros constructed");
}

} // namespace flame_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(flame_ros::FlameRos)