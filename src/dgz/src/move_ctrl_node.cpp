#include "rclcpp/rclcpp.hpp"
#include "space_interfaces/msg/position.hpp"

#if __DEBUG
#define __debug_print(...) RCLCPP_INFO(__VA_ARGS__)
#define __debug_print_error(...) RCLCPP_ERROR(__VA_ARGS__)
#else
#define __debug_print(...)
#define __debug_print_error(...)
#endif

class move_ctrl_node : public rclcpp::Node
{
public:
    move_ctrl_node() : Node("move_ctrl_node")
    {
        subscription = this->create_subscription<space_interfaces::msg::Position>("position", 1, std::bind(&move_ctrl_node::position_callback, this, std::placeholders::_1));
        publisher = this->create_publisher<space_interfaces::msg::Position>("move_ctrl_cmd", 1);

        // pid_ctrl_x = {0.2, 0.01, 0.05};
        // pid_ctrl_y = {0.2, 0.01, 0.05};
        // pid_ctrl_z = {0.0, 0.0, 0.0};
        // pid_ctrl_roll = {0.0, 0.0, 0.0};
        // pid_ctrl_pitch = {0.0, 0.0, 0.0};
        // pid_ctrl_yaw = {0.3, 0.02, 0.01};
    }

private:
    rclcpp::Subscription<space_interfaces::msg::Position>::SharedPtr subscription;
    rclcpp::Publisher<space_interfaces::msg::Position>::SharedPtr publisher;
    space_interfaces::msg::Position current_position;
    struct pid_ctrl_t
    {
        double kp, ki, kd;
        double prev_error = 0.0, error = 0.0, integral = 0.0, derivative = 0.0;
    };
    pid_ctrl_t pid_ctrl_x, pid_ctrl_y, pid_ctrl_z, pid_ctrl_roll, pid_ctrl_pitch, pid_ctrl_yaw;

    double pid_apply(double current_value, double desired_value, pid_ctrl_t &pid_ctrl)
    {
        pid_ctrl.error = desired_value - current_value;
        pid_ctrl.integral += pid_ctrl.error;
        pid_ctrl.derivative = pid_ctrl.error - pid_ctrl.prev_error;
        pid_ctrl.prev_error = pid_ctrl.error;
        return pid_ctrl.kp * pid_ctrl.error + pid_ctrl.ki * pid_ctrl.integral + pid_ctrl.kd * pid_ctrl.derivative;
    }
    void position_callback(const space_interfaces::msg::Position::SharedPtr msg)
    {
        __debug_print(this->get_logger(), "Received from topic position: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw);

        auto move_ctrl_cmd = space_interfaces::msg::Position();

        // move_ctrl_cmd.x = pid_apply(current_position.x, msg->x, pid_ctrl_x);
        // move_ctrl_cmd.y = pid_apply(current_position.y, msg->y, pid_ctrl_y);
        // move_ctrl_cmd.z = pid_apply(current_position.z, msg->z, pid_ctrl_z);
        // move_ctrl_cmd.roll = pid_apply(current_position.roll, msg->roll, pid_ctrl_roll);
        // move_ctrl_cmd.pitch = pid_apply(current_position.pitch, msg->pitch, pid_ctrl_pitch);
        // move_ctrl_cmd.yaw = pid_apply(current_position.yaw, msg->yaw, pid_ctrl_yaw);

        move_ctrl_cmd.x = msg->x;
        move_ctrl_cmd.y = msg->y;
        move_ctrl_cmd.z = msg->z;
        move_ctrl_cmd.roll = msg->roll;
        move_ctrl_cmd.pitch = msg->pitch;
        move_ctrl_cmd.yaw = msg->yaw;

        current_position.x = move_ctrl_cmd.x;
        current_position.y = move_ctrl_cmd.y;
        current_position.z = move_ctrl_cmd.z;
        current_position.roll = move_ctrl_cmd.roll;
        current_position.pitch = move_ctrl_cmd.pitch;
        current_position.yaw = move_ctrl_cmd.yaw;

        publisher->publish(move_ctrl_cmd);

        __debug_print(this->get_logger(), "Published move_ctrl_cmd: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", move_ctrl_cmd.x, move_ctrl_cmd.y, move_ctrl_cmd.z, move_ctrl_cmd.roll, move_ctrl_cmd.pitch, move_ctrl_cmd.yaw);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<move_ctrl_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}