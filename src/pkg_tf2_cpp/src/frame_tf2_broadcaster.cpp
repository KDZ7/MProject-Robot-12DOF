#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

class FrameTF2Broadcaster : public rclcpp::Node
{
public:
  explicit FrameTF2Broadcaster()
      : Node("frame_tf2_broadcaster")
  {
    this->frame_id = this->declare_parameter<std::string>("--frame-id", "world");
    this->child_frame_id = this->declare_parameter<std::string>("--child-frame-id", "turtle1");
    this->topic_position = this->declare_parameter<std::string>("--topic-position", "");
    if (this->topic_position.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No topic position provided");
      throw std::runtime_error("No topic position provided");
    }
    this->tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    auto hdle_listener_pose = [this](const std::shared_ptr<turtlesim::msg::Pose> msg)
    {
      geometry_msgs::msg::TransformStamped tf_stamped;
      tf_stamped.header.stamp = this->get_clock()->now();
      tf_stamped.header.frame_id = this->frame_id;
      tf_stamped.child_frame_id = this->child_frame_id;
      tf_stamped.transform.translation.x = msg->x;
      tf_stamped.transform.translation.y = msg->y;
      tf_stamped.transform.translation.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, msg->theta);
      tf_stamped.transform.rotation.x = q.x();
      tf_stamped.transform.rotation.y = q.y();
      tf_stamped.transform.rotation.z = q.z();
      tf_stamped.transform.rotation.w = q.w();
      this->tf_broadcaster->sendTransform(tf_stamped);
    };
    this->subscription = this->create_subscription<turtlesim::msg::Pose>(
        this->topic_position, 1, hdle_listener_pose);
  }

private:
  std::string frame_id;
  std::string child_frame_id;
  std::string topic_position;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameTF2Broadcaster>());
  rclcpp::shutdown();
  return 0;
}
