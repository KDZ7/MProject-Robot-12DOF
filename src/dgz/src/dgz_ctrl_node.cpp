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
        serial_ = new serial(
            "/dev/ttyAMA0",
            BAUDRATE_115200,
            DATABIT_8,
            STOPBIT_1,
            PARITY_NONE,
            false,
            10,
            0,
            false);
        serial_->init();
        subscription = this->create_subscription<space_interfaces::msg::Position>(
            "move_ctrl_cmd", 10, std::bind(&dgz_ctrl_node::move_ctrl_cmd_callback, this, std::placeholders::_1));
    }

    ~dgz_ctrl_node()
    {
        delete serial_;
        serial_ = nullptr;
    }

private:
    rclcpp::Subscription<space_interfaces::msg::Position>::SharedPtr subscription;
    serial *serial_;

    void move_ctrl_cmd_callback(const space_interfaces::msg::Position::SharedPtr msg)
    {
        __debug_print(this->get_logger(), "Received move control command: x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f", msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw);

        uint8_t vx = 0x80;

        switch ((int)msg->x)
        {
        case -1:
            vx = 0x40;
            break;
        case 0:
            vx = 0x80;
            break;
        case 1:
            vx = 0xBF;
            break;
        }

        uint8_t vy = 0x80;
        switch ((int)msg->y)
        {
        case -1:
            vy = 0x40;
            break;
        case 0:
            vy = 0x80;
            break;
        case 1:
            vy = 0xBF;
            break;
        }

        uint8_t vz = 0x80;
        switch ((int)msg->z)
        {
        case -1:
            vz = 0x40;
            break;
        case 0:
            vz = 0x80;
            break;
        case 1:
            vz = 0xBF;
            break;
        }

        uint8_t dvx[] = {vx};
        uint8_t dvy[] = {vy};
        uint8_t dvz[] = {vz};

        dgz_frame frame_x = dgz_frame(0x01, 0x31, dvx, 1);
        dgz_frame frame_y = dgz_frame(0x01, 0x32, dvy, 1);
        dgz_frame frame_z = dgz_frame(0x01, 0x33, dvz, 1);

        uint8_t bufs[32];

        frame_x.toBufs(bufs);
        if (!serial_->write(bufs, frame_x.Length()))
            __debug_print(this->get_logger(), "Failed to write X data to serial device");
        frame_y.toBufs(bufs);
        if (!serial_->write(bufs, frame_y.Length()))
            __debug_print(this->get_logger(), "Failed to write Y data to serial device");
        frame_z.toBufs(bufs);
        if (!serial_->write(bufs, frame_z.Length()))
            __debug_print(this->get_logger(), "Failed to write Z data to serial device");
        for (int i = 0; i < frame_x.Length(); i++)
            __debug_print(this->get_logger(), "dvx[%d] = 0x%02X", i, bufs[i]);
        for (int i = 0; i < frame_y.Length(); i++)
            __debug_print(this->get_logger(), "dvy[%d] = 0x%02X", i, bufs[i]);
        for (int i = 0; i < frame_z.Length(); i++)
            __debug_print(this->get_logger(), "dvz[%d] = 0x%02X", i, bufs[i]);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dgz_ctrl_node>());
    rclcpp::shutdown();
    return 0;
}