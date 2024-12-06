#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#define __seq_calibration {0.0, -0.8, 1.6, 0.0, 0.8, 1.6, 0.0, -0.8, 1.6, 0.0, 0.8, 1.6}

class autorun_node : public rclcpp::Node
{
public:
    autorun_node() : Node("autorun_node")
    {
        publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_position_controller/commands", 10);
        subscriber = this->create_subscription<std_msgs::msg::String>("/autorun_node/cmd", 1, std::bind(&autorun_node::cmd_callback, this, std::placeholders::_1));
        timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&autorun_node::timer_callback, this));
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

        RCLCPP_INFO(this->get_logger(), "autorun_node sequence (%s): %f %f %f %f %f %f %f %f %f %f %f %f",
                    cmd.c_str(), msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4], msg.data[5],
                    msg.data[6], msg.data[7], msg.data[8], msg.data[9], msg.data[10], msg.data[11]);
    }

    std::vector<std::vector<double>> get_seq(std::string cmd)
    {
        std::vector<std::vector<double>> seq;
        if (cmd == "F")
            return {
                {__seq_calibration},
                {0.0, -0.4, 1.6, 0.0, 0.8, 1.6, 0.0, -0.4, 1.6, 0.0, 0.8, 1.6},
                {0.2, -0.4, 1.6, 0.0, 0.8, 1.6, -0.2, -0.4, 1.6, 0.0, 0.8, 1.6},
                {0.2, -0.8, 1.6, 0.0, 0.8, 1.6, -0.2, -0.8, 1.6, 0.0, 0.8, 1.6},
                {0.0, -0.8, 1.6, 0.2, -0.4, 1.6, 0.0, -0.8, 1.6, -0.2, -0.4, 1.6},
                {0.0, -0.8, 1.6, 0.2, -0.4, 1.6, 0.0, -0.8, 1.6, -0.2, -0.4, 1.6},
                {0.0, -0.8, 1.6, 0.2, -0.8, 1.6, 0.0, -0.8, 1.6, -0.2, -0.8, 1.6},
                {__seq_calibration}};

        else if (cmd == "B")
            return {
                {__seq_calibration},
                {0.0, -0.4, 1.6, 0.0, 0.8, 1.6, 0.0, -0.4, 1.6, 0.0, 0.8, 1.6},
                {-0.2, -0.4, 1.6, 0.0, 0.8, 1.6, 0.2, -0.4, 1.6, 0.0, 0.8, 1.6},
                {-0.2, -0.8, 1.6, 0.0, 0.8, 1.6, 0.2, -0.8, 1.6, 0.0, 0.8, 1.6},
                {0.0, -0.8, 1.6, -0.2, -0.4, 1.6, 0.0, -0.8, 1.6, 0.2, -0.4, 1.6},
                {0.0, -0.8, 1.6, -0.2, -0.4, 1.6, 0.0, -0.8, 1.6, 0.2, -0.4, 1.6},
                {0.0, -0.8, 1.6, -0.2, -0.8, 1.6, 0.0, -0.8, 1.6, 0.2, -0.8, 1.6},
                {__seq_calibration}};
        else if (cmd == "L")
            return {
                {__seq_calibration},
                {0.0, -0.4, 1.6, 0.0, 0.8, 1.6, 0.0, -0.4, 1.6, 0.0, 0.8, 1.6},
                {0.2, -0.4, 1.6, 0.0, 0.8, 1.6, 0.2, -0.4, 1.6, 0.0, 0.8, 1.6},
                {0.2, -0.8, 1.6, 0.0, 0.8, 1.6, 0.2, -0.8, 1.6, 0.0, 0.8, 1.6},
                {-0.2, -0.8, 1.6, 0.0, -0.4, 1.6, -0.2, -0.8, 1.6, 0.0, -0.4, 1.6},
                {-0.2, -0.8, 1.6, 0.2, -0.4, 1.6, -0.2, -0.8, 1.6, 0.2, -0.4, 1.6},
                {__seq_calibration}};
        else if (cmd == "R")
            return {
                {__seq_calibration},
                {0.0, -0.4, 1.6, 0.0, 0.8, 1.6, 0.0, -0.4, 1.6, 0.0, 0.8, 1.6},
                {0.0, -0.4, 1.6, 0.2, 0.8, 1.6, 0.0, -0.4, 1.6, -0.2, 0.8, 1.6},
                {0.0, -0.8, 1.6, 0.2, 0.8, 1.6, 0.0, -0.8, 1.6, -0.2, 0.8, 1.6},
                {0.2, -0.8, 1.6, 0.0, -0.4, 1.6, 0.2, -0.8, 1.6, 0.0, -0.4, 1.6},
                {__seq_calibration}};
        return {__seq_calibration};
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
    rclcpp::spin(std::make_shared<autorun_node>());
    rclcpp::shutdown();
    return 0;
}