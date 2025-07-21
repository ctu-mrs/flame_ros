#include <flame_ros/flame_ros.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<flame_ros::Flame>());
  rclcpp::shutdown();
  return 0;
}