#include <string>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.hpp"

class testLibNode : public rclcpp::Node
{
private:
    std::string device;
    int baudrate;
    int databit;
    int stopbit;
    int parity;
    bool enable_RTS_CTS;
    int max_timeout_read_ds;
    int min_buf_read;
    bool wait_buf_read;
    std::string str_data;
    std::unique_ptr<serial> serial_;

public:
    testLibNode() : Node("test_lib_node")
    {
        this->declare_parameter("device", "/dev/ttyAMA0");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("databit", 8);
        this->declare_parameter("stopbit", 1);
        this->declare_parameter("parity", 0);
        this->declare_parameter("enable_RTS_CTS", false);
        this->declare_parameter("max_timeout_read_ds", 1);
        this->declare_parameter("min_buf_read", 10);
        this->declare_parameter("wait_buf_read", true);
        this->declare_parameter("str_data", "This is a test string for serial communication");

        this->get_parameter("device", device);
        this->get_parameter("baudrate", baudrate);
        this->get_parameter("databit", databit);
        this->get_parameter("stopbit", stopbit);
        this->get_parameter("parity", parity);
        this->get_parameter("enable_RTS_CTS", enable_RTS_CTS);
        this->get_parameter("max_timeout_read_ds", max_timeout_read_ds);
        this->get_parameter("min_buf_read", min_buf_read);
        this->get_parameter("wait_buf_read", wait_buf_read);
        this->get_parameter("str_data", str_data);

        RCLCPP_INFO(this->get_logger(), "Starting serial communication with device: %s", device.c_str());
        RCLCPP_INFO(this->get_logger(), "wait_buf_read: %d", wait_buf_read);
        serial_ = std::make_unique<serial>(device.c_str(),
                                           static_cast<baudrate_t>(baudrate),
                                           static_cast<databit_t>(databit),
                                           static_cast<stopbit_t>(stopbit),
                                           static_cast<parity_t>(parity),
                                           enable_RTS_CTS,
                                           max_timeout_read_ds,
                                           min_buf_read,
                                           wait_buf_read);

        if (!serial_->init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial communication");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Serial communication initialized");
    }

    void run()
    {
        rclcpp::Rate rate(10); // 1 Hz
        uint8_t bufs[256];
        memset(bufs, 0, sizeof(bufs));
        while (rclcpp::ok())
        {

            // rclcpp::spin_some(this->get_node_base_interface());

            ssize_t bytes_written = serial_->write(reinterpret_cast<const uint8_t *>(str_data.c_str()), str_data.size());
            if (bytes_written < 0)
                RCLCPP_ERROR(this->get_logger(), "Failed to write data to serial device");

            RCLCPP_INFO(this->get_logger(), "Data written successfully: %s", str_data.c_str());

            ssize_t bytes_read = serial_->read(bufs, sizeof(bufs));
            if (bytes_read < 0)
                RCLCPP_ERROR(this->get_logger(), "Failed to read data from serial device");

            bufs[bytes_read] = '\0';
            RCLCPP_INFO(this->get_logger(), "Data read successfully: %s", bufs);

            rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "Closing serial communication ...");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<testLibNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
