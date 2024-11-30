#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "space_interfaces/msg/position.hpp"

class dgz_keyboard_node : public rclcpp::Node
{
public:
    dgz_keyboard_node() : Node("dgz_keyboard_node"), timer_interval_ms(500)
    {
        subscription = this->create_subscription<geometry_msgs::msg::Twist>(
            "dgz/cmd_vel", 1, std::bind(&dgz_keyboard_node::cmd_vel_callback, this, std::placeholders::_1));
        publisher = this->create_publisher<space_interfaces::msg::Position>("position", 1);
        timer = this->create_wall_timer(std::chrono::milliseconds(timer_interval_ms), std::bind(&dgz_keyboard_node::timer_callback, this));
        last_time_msg = this->now();
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
    rclcpp::Publisher<space_interfaces::msg::Position>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Time last_time_msg;
    const int timer_interval_ms;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_time_msg = this->now();
        auto position_msg = space_interfaces::msg::Position();
        // Transformation from TURTLESIM frame to DGZ frame
        // TURTLESIM -> DGZ
        position_msg.x = msg->angular.z;
        position_msg.y = msg->linear.x;
        position_msg.z = 0.0;
        position_msg.roll = 0.0;
        position_msg.pitch = 0.0;
        position_msg.yaw = position_msg.x != 0 ? position_msg.y / position_msg.x : 0;
        publisher->publish(position_msg);
    }
    void timer_callback()
    {
        if (this->now() - last_time_msg > std::chrono::milliseconds(timer_interval_ms))
        {
            auto position_msg = space_interfaces::msg::Position();
            position_msg.x = 0.0;
            position_msg.y = 0.0;
            position_msg.z = 0.0;
            position_msg.roll = 0.0;
            position_msg.pitch = 0.0;
            position_msg.yaw = 0.0;
            publisher->publish(position_msg);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dgz_keyboard_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}