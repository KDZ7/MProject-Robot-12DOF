#include <string>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.hpp"

class testlib_node : public rclcpp::Node
{
public:
    testlib_node() : Node("testlib_node")
    {
        std::string device;
        int baudrate;
        int databits;
        int stopbits;
        int parity;
        int min_buf_read;
        int max_timeout_read_ds;
        int attempts_read;
        std::string str_data;

        this->declare_parameter("device", "/dev/ttyAMA0");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("databits", 8);
        this->declare_parameter("stopbits", 1);
        this->declare_parameter("parity", 0);
        this->declare_parameter("min_buf_read", 0);
        this->declare_parameter("max_timeout_read_ds", 10);
        this->declare_parameter("attempts_read", 1000);
        this->declare_parameter("str_data", "This is a test string for serial communication");

        this->get_parameter("device", device);
        this->get_parameter("baudrate", baudrate);
        this->get_parameter("databits", databits);
        this->get_parameter("stopbits", stopbits);
        this->get_parameter("parity", parity);
        this->get_parameter("min_buf_read", min_buf_read);
        this->get_parameter("max_timeout_read_ds", max_timeout_read_ds);
        this->get_parameter("attempts_read", attempts_read);
        this->get_parameter("str_data", str_data);

        RCLCPP_INFO(this->get_logger(), "Starting serial communication with device: %s", device.c_str());
        serial serial_(device.c_str(), baudrate, databits, stopbits, parity, min_buf_read, max_timeout_read_ds, attempts_read);
        if (!serial_.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial communication");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Serial communication initialized");

        rclcpp::Rate rate(1.0); // 1 Hz

        while (rclcpp::ok())
        {
            if (!serial_.write(reinterpret_cast<const uint8_t *>(str_data.c_str()), str_data.size()))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write data to serial device");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Data written successfully: %s", str_data.c_str());

            uint8_t data[256];
            if (!serial_.read(data, str_data.size()))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to read data from serial device");
                return;
            }
            data[str_data.size()] = '\0';
            RCLCPP_INFO(this->get_logger(), "Data read successfully: %s", data);

            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<testlib_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}