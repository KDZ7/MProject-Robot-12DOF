#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "space_interfaces/msg/position.hpp"

#define __DEBUG 1
#if __DEBUG
#define __debug_print(...) RCLCPP_INFO(__VA_ARGS__)
#else
#define __debug_print(...)
#endif

class manette_node : public rclcpp::Node
{
public:
  manette_node() : Node("manette_node")
  {
    subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, std::bind(&manette_node::joy_callback, this, std::placeholders::_1));

    publisher = this->create_publisher<space_interfaces::msg::Position>("position", 1);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription;
  rclcpp::Publisher<space_interfaces::msg::Position>::SharedPtr publisher;
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    __debug_print(this->get_logger(), "Joystick 1: - Axe X: %f - Axe Y: %f", msg->axes[0], msg->axes[1]);
    __debug_print(this->get_logger(), "Joystick 2: - Axe X: %f - Axe Y: %f", msg->axes[2], msg->axes[3]);
    __debug_print(this->get_logger(), "Btn A: %d - Btn B: %d - Btn X: %d - Btn Y: %d", msg->buttons[0], msg->buttons[1], msg->buttons[3], msg->buttons[4]);

    auto position_msg = space_interfaces::msg::Position();

    position_msg.x = msg->axes[2];
    position_msg.y = msg->axes[1];
    position_msg.z = 0.0;
    position_msg.roll = 0.0;
    position_msg.pitch = 0.0;
    position_msg.yaw = atan2(position_msg.y, position_msg.x);

    publisher->publish(position_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<manette_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
