#include <rclcpp/rclcpp.hpp>

namespace flame_ros {

class Flame : public rclcpp::Node
{
public:
  Flame();
//   : Node("flame node") //, count_(0)
//   {
//     publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//     auto timer_callback =
//       [this]() -> void {
//         auto message = std_msgs::msg::String();
//         message.data = "Hello, world! " + std::to_string(this->count_++);
//         RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//         this->publisher_->publish(message);
//       };
//     timer_ = this->create_wall_timer(500ms, timer_callback);
//   }

//private:
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//   size_t count_;
};

} // namespace flame_ros