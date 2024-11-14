#include "dgz/dgz_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.hpp"
#include "space_interfaces/msg/position.hpp"

#define CONVERT_RAD_BUF(x) (uint8_t)(x * 0xFF)

#define __DEBUG 1
#if __DEBUG
#define __debug_print(...) RCLCPP_INFO(__VA_ARGS__)
#else
#define __debug_print(...)
#endif

class dgz_ctrl_node : public rclcpp::Node
{
public:
    dgz_ctrl_node() : Node("dgz_ctrl_node")
    {
        serial_.init();
        subscription = this->create_subscription<space_interfaces::msg::Position>(
            "move_ctrl_cmd", 1, std::bind(&dgz_ctrl_node::move_ctrl_cmd_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<space_interfaces::msg::Position>::SharedPtr subscription;
    serial serial_ = serial("/dev/ttyAMA0", 115200, 8, 1, 0, 0, 10, 1000);

    void move_ctrl_cmd_callback(const space_interfaces::msg::Position::SharedPtr msg)
    {
        __debug_print(this->get_logger(), "Received move control command: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw);
        uint8_t x = CONVERT_RAD_BUF(msg->x);
        uint8_t y = CONVERT_RAD_BUF(msg->y);
        uint8_t yaw = CONVERT_RAD_BUF(msg->yaw);

        dgz_frame frame_x = dgz_frame(0x01, 0x30, &y, 1);
        dgz_frame frame_y = dgz_frame(0x01, 0x31, &x, 1);

        uint8_t buf[256];
        frame_x.toBufs(buf);
        serial_.write(buf, frame_x.Length);

        size_t bytes_read = serial_.read(buf, 256);

        RCLCPP_INFO(this->get_logger(), "Received frame:");
        for (size_t i = 0; i < bytes_read; ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Byte %zu: %d", i, buf[i]);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dgz_ctrl_node>());
    rclcpp::shutdown();
    return 0;
}