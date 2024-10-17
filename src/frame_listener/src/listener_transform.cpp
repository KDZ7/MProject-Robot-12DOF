#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class listener_transform : public rclcpp::Node
{
public:
    listener_transform() : Node("listener_transform")
    {
        source = this->declare_parameter<std::string>("source", "");
        target = this->declare_parameter<std::string>("target", "");

        if (source.empty() || target.empty())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Missing parameters: source: '%s', target: '%s'",
                         source.empty() ? "not specified" : source.c_str(),
                         target.empty() ? "not specified" : target.c_str());

            rclcpp::shutdown();
            return;
        }
        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, this);
        timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this](void)
            { this->hdle_timer(); });
    }

private:
    std::string source;
    std::string target;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    rclcpp::TimerBase::SharedPtr timer;
    void hdle_timer()
    {
        try
        {
            geometry_msgs::msg::TransformStamped tf_s;
            tf_s = tf_buffer->lookupTransform(target, source, tf2::TimePointZero);
            double roll, pitch, yaw;
            std::tie(roll, pitch, yaw) = quaternion_euler(tf_s.transform.rotation);
            RCLCPP_INFO(
                this->get_logger(), "Received transform '%s' <- '%s': x=%f, y=%f, z=%f | roll=%f, pitch=%f, yaw=%f",
                source.c_str(), target.c_str(),
                tf_s.transform.translation.x, tf_s.transform.translation.y, tf_s.transform.translation.z,
                roll, pitch, yaw);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }
    }
    std::tuple<double, double, double> quaternion_euler(const geometry_msgs::msg::Quaternion &q)
    {
        double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        double pitch = std::asin(2.0 * (q.w * q.y - q.z * q.x));
        double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        return std::make_tuple(roll, pitch, yaw);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<listener_transform>());
    rclcpp::shutdown();
    return 0;
}