#include "rclcpp/rclcpp.hpp"
#include "serial/serial.hpp"
#include "dgz/dgz_frame.hpp"
#include "space_interfaces/msg/position.hpp"

class dgz_ctrl_node : public rclcpp::Node
{
public:
    dgz_ctrl_node() : Node("dgz_ctrl_node")
    {
        subscription = this->create_subscription<space_interfaces::msg::Position>(
            "move_ctrl_cmd", 1, std::bind(&dgz_ctrl_node::move_ctrl_cmd_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<space_interfaces::msg::Position>::SharedPtr subscription;
    serial serial_port = serial("/dev/ttyUSB0", 115200, 8, 1, 0, 0, 10, 1000);

    void move_ctrl_cmd_callback(const space_interfaces::msg::Position::SharedPtr msg)
    {
        __debug_print(this->get_logger(), "Received move control command: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw);

        uint8_t data[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
        dgz_frame frame = dgz_frame(0x01, 0x01, data, 6);
        uint8_t buf[8 + 6];
        frame.toBufs(buf);
        serial_port.write(buf, 8 + 6);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dgz_ctrl_node>());
    rclcpp::shutdown();
    return 0;
}