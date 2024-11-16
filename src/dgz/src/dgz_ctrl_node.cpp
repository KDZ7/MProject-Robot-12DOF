#include "dgz/dgz_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.hpp"
#include "space_interfaces/msg/position.hpp"

#if __DEBUG
#define __debug_print(...) RCLCPP_INFO(__VA_ARGS__)
#define __debug_print_error(...) RCLCPP_ERROR(__VA_ARGS__)
#else
#define __debug_print(...)
#define __debug_print_error(...)
#endif

class dgz_ctrl_node : public rclcpp::Node
{
public:
    dgz_ctrl_node() : Node("dgz_ctrl_node")
    {
        serial_ = std::make_unique<serial>(
            "/dev/ttyAMA0",
            BAUDRATE_115200,
            DATABIT_8,
            STOPBIT_1,
            PARITY_NONE,
            false,
            10,
            0,
            false);
        if (!serial_->init())
        {
            __debug_print_error(this->get_logger(), "Failed to initialize serial communication");
            return;
        }
        subscription = this->create_subscription<space_interfaces::msg::Position>("move_ctrl_cmd", 1, std::bind(&dgz_ctrl_node::move_ctrl_cmd_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<space_interfaces::msg::Position>::SharedPtr subscription;
    std::unique_ptr<serial> serial_;

    uint8_t discretize(double x)
    {
        if (x < -0.5)
            return 0x40;
        else if (x > 0.5)
            return 0xBF;
        else
            return 0x80;
    }

    void move_ctrl_cmd_callback(const space_interfaces::msg::Position::SharedPtr msg)
    {
        __debug_print(this->get_logger(), "Received from topic move_ctrl_cmd: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw);

        uint8_t vx = discretize(msg->x);
        uint8_t vy = discretize(msg->y);
        uint8_t vyaw = discretize(msg->yaw);

        uint8_t dvx[] = {vx};
        uint8_t dvy[] = {vy};
        uint8_t dvyaw[] = {vyaw};

        __debug_print(this->get_logger(), "Discretized values: vx=0x%02X, vy=0x%02X, vyaw=0x%02X", vx, vy, vyaw);

        dgz_frame frame_x = dgz_frame(0x01, 0x31, dvx, 1);
        dgz_frame frame_y = dgz_frame(0x01, 0x30, dvy, 1);
        dgz_frame frame_z = dgz_frame(0x01, 0x32, dvyaw, 1);

        uint8_t bufs[32];

        frame_x.toBufs(bufs);
        if (serial_->write(bufs, frame_x.Length()) < 0)
            __debug_print_error(this->get_logger(), "Failed to write X data to serial device");
        frame_y.toBufs(bufs);
        if (serial_->write(bufs, frame_y.Length()) < 0)
            __debug_print_error(this->get_logger(), "Failed to write Y data to serial device");
        frame_z.toBufs(bufs);
        if (serial_->write(bufs, frame_z.Length()) < 0)
            __debug_print_error(this->get_logger(), "Failed to write Z data to serial device");

        __debug_print(this->get_logger(), "-----------------------");
        for (int i = 0; i < frame_x.Length(); i++)
            __debug_print(this->get_logger(), "dvx[%d] = 0x%02X", i, bufs[i]);
        __debug_print(this->get_logger(), "-----------------------");
        for (int i = 0; i < frame_y.Length(); i++)
            __debug_print(this->get_logger(), "dvy[%d] = 0x%02X", i, bufs[i]);
        __debug_print(this->get_logger(), "-----------------------");
        for (int i = 0; i < frame_z.Length(); i++)
            __debug_print(this->get_logger(), "dvyaw[%d] = 0x%02X", i, bufs[i]);
        __debug_print(this->get_logger(), "-----------------------");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dgz_ctrl_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}