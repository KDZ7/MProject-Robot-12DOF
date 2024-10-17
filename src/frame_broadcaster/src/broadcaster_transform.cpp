#include <memory>
#include <string>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "space_interfaces/msg/position.hpp"

class broadcaster_transform : public rclcpp::Node
{
public:
    broadcaster_transform() : Node("broadcaster_transform")
    {
        source = this->declare_parameter<std::string>("source", "");
        target = this->declare_parameter<std::string>("target", "");
        topic_position = this->declare_parameter<std::string>("topic_position", "");

        if (source.empty() || target.empty() || topic_position.empty())
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Missing parameters: source: '%s', target: '%s', topic_position: '%s'",
                         source.empty() ? "not specified" : source.c_str(),
                         target.empty() ? "not specified" : target.c_str(),
                         topic_position.empty() ? "not specified" : topic_position.c_str());

            rclcpp::shutdown();
            return;
        }
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        subscription = this->create_subscription<space_interfaces::msg::Position>(
            topic_position,
            10,
            [this](const space_interfaces::msg::Position::SharedPtr msg)
            { this->hdle_receive_position(msg); });
    }

private:
    std::string source;
    std::string target;
    std::string topic_position;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Subscription<space_interfaces::msg::Position>::SharedPtr subscription;

    void hdle_receive_position(const space_interfaces::msg::Position::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(), "Received position '%s' <- '%s': x=%f, y=%f, z=%f | roll=%f, pitch=%f, yaw=%f",
            source.c_str(), target.c_str(),
            msg->x, msg->y, msg->z,
            msg->roll, msg->pitch, msg->yaw);
        feedback(msg);
    }
    void feedback(const space_interfaces::msg::Position::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf_s;
        tf_s.header.stamp = this->get_clock()->now();
        tf_s.header.frame_id = source;
        tf_s.child_frame_id = target;
        tf_s.transform.translation.x = msg->x;
        tf_s.transform.translation.y = msg->y;
        tf_s.transform.translation.z = msg->z;
        tf2::Quaternion q;
        q.setRPY(msg->roll, msg->pitch, msg->yaw);
        tf_s.transform.rotation.x = q.x();
        tf_s.transform.rotation.y = q.y();
        tf_s.transform.rotation.z = q.z();
        tf_s.transform.rotation.w = q.w();
        tf_broadcaster->sendTransform(tf_s);
        RCLCPP_INFO(this->get_logger(), "Broadcasted transform '%s' -> '%s'", source.c_str(), target.c_str());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<broadcaster_transform>());
    rclcpp::shutdown();
    return 0;
}