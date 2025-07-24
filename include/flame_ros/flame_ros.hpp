#ifndef FLAME_ROS_FLAME_HPP
#define FLAME_ROS_FLAME_HPP

#include <ros_sensor_streams/tracked_image_stream.h>

//#include <ros_sensor_streams/header2.hpp>

namespace flame_ros {

class FlameRos : public rclcpp::Node
{
  public:
    FlameRos(const rclcpp::NodeOptions & options);
};

} // namespace flame_ros

#endif