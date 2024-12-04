#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class quadro_move_ctrl_node : public rclcpp::Node
{
public:
    quadro_move_ctrl_node() : Node("quadro_move_ctrl_node")
    {
        publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_position_controller/commands", 1);
        timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&quadro_move_ctrl_node::timer_callback, this));
        subscriber = this->create_subscription<std_msgs::msg::String>(
            "/quadro_move_ctrl_node/cmd", 1, std::bind(&quadro_move_ctrl_node::cmd_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber;
    rclcpp::TimerBase::SharedPtr timer;
    std::string cmd;
    size_t current_phase = 0;
    void timer_callback()
    {
        std::vector<std::vector<double>> seq = get_seq(cmd);
        std_msgs::msg::Float64MultiArray msg;
        msg.data = seq[current_phase];
        publisher->publish(msg);
        current_phase = (current_phase + 1) % seq.size();
    }
    std::vector<std::vector<double>> get_seq(std::string cmd)
    {
        std::vector<std::vector<double>> seq;
        if (cmd == "F")
            return {
                {0.1, 0.6, -0.3, 0.0, 0.5, -0.5, 0.1, 0.6, -0.3, 0.0, 0.5, -0.5},
                {0.0, 0.5, -0.5, 0.1, 0.6, -0.3, 0.0, 0.5, -0.5, 0.1, 0.6, -0.3}};
        else if (cmd == "B")
            return {
                {-0.1, 0.4, -0.7, -0.1, 0.4, -0.7, -0.1, 0.4, -0.7, -0.1, 0.4, -0.7},
                {0.0, 0.5, -0.5, -0.1, 0.4, -0.7, 0.0, 0.5, -0.5, -0.1, 0.4, -0.7}};

        else if (cmd == "L")
            return {
                {0.1, 0.6, -0.3, -0.1, 0.4, -0.7, -0.1, 0.4, -0.7, 0.1, 0.6, -0.3},
                {0.0, 0.5, -0.5, 0.1, 0.6, -0.3, 0.0, 0.5, -0.5, -0.1, 0.4, -0.7}};
        else if (cmd == "R")
        {
            return {
                {-0.1, 0.4, -0.7, 0.1, 0.6, -0.3, 0.1, 0.6, -0.3, -0.1, 0.4, -0.7},
                {0.0, 0.5, -0.5, -0.1, 0.4, -0.7, 0.0, 0.5, -0.5, 0.1, 0.6, -0.3}};
        }

        return {{0.0, 0.5, -0.5, 0.0, 0.5, -0.5, 0.0, 0.5, -0.5, 0.0, 0.5, -0.5}};
    }
    void cmd_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        cmd = msg->data;
        current_phase = 0;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<quadro_move_ctrl_node>());
    rclcpp::shutdown();
    return 0;
}