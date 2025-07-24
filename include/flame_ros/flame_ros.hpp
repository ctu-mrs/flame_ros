#include <rclcpp/rclcpp.hpp>

namespace flame_ros {

class FlameRos : public rclcpp::Node
{
  public:
    FlameRos(const rclcpp::NodeOptions & options);
};

} // namespace flame_ros