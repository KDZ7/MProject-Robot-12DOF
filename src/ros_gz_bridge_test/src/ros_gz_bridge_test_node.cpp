#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class ros_gz_bridge_test_node : public rclcpp::Node
{
public:
    ros_gz_bridge_test_node() : Node("ros_gz_bridge_test_node")
    {
        publisher_1 = this->create_publisher<std_msgs::msg::Int32>("/ros_gz_bridge_test_node/value", 1);
        publisher_2 = this->create_publisher<std_msgs::msg::String>("/ros_gz_bridge_test_node/string", 1);
        subscriber_1 = this->create_subscription<std_msgs::msg::Int32>("/ros_gz_bridge_test_node/value", 1, [](const std_msgs::msg::Int32::SharedPtr msg)
                                                                       { RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%d'", msg->data); });
        subscriber_2 = this->create_subscription<std_msgs::msg::String>("/ros_gz_bridge_test_node/string", 1, [](const std_msgs::msg::String::SharedPtr msg)
                                                                        { RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: '%s'", msg->data.c_str()); });
        timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ros_gz_bridge_test_node::timer_callback, this));
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_1;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_1;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_2;
    rclcpp::TimerBase::SharedPtr timer;
    void timer_callback()
    {
        auto message_1 = std_msgs::msg::Int32();
        message_1.data = 1;
        auto message_2 = std_msgs::msg::String();
        message_2.data = "string ros_gz_bridge_test_node !!!";
        publisher_1->publish(message_1);
        publisher_2->publish(message_2);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ros_gz_bridge_test_node>());
    rclcpp::shutdown();
    return 0;
}