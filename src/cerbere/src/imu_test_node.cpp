#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class imu_test_node : public rclcpp::Node
{
public:
    imu_test_node() : Node("imu_test_node")
    {
        subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
            "/cerbere/imu", 10, std::bind(&imu_test_node::imu_callback, this, std::placeholders::_1));

        publisher_orientation = this->create_publisher<geometry_msgs::msg::PoseStamped>("/imu_test_node/orientation", 10);
        publisher_angular_velocity = this->create_publisher<geometry_msgs::msg::PoseStamped>("/imu_test_node/angular_velocity", 10);
        publisher_linear_acceleration = this->create_publisher<geometry_msgs::msg::PoseStamped>("/imu_test_node/linear_acceleration", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_orientation;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_angular_velocity;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_linear_acceleration;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped orientation_pose;
        orientation_pose.header = msg->header;
        orientation_pose.pose.position.x = 0.0;
        orientation_pose.pose.position.y = 0.0;
        orientation_pose.pose.position.z = 0.0;
        orientation_pose.pose.orientation = msg->orientation;
        publisher_orientation->publish(orientation_pose);

        geometry_msgs::msg::PoseStamped angular_velocity_pose;
        angular_velocity_pose.header = msg->header;
        angular_velocity_pose.pose.position.x = msg->angular_velocity.x;
        angular_velocity_pose.pose.position.y = msg->angular_velocity.y;
        angular_velocity_pose.pose.position.z = msg->angular_velocity.z;
        angular_velocity_pose.pose.orientation.w = 1.0;
        angular_velocity_pose.pose.orientation.x = 0.0;
        angular_velocity_pose.pose.orientation.y = 0.0;
        angular_velocity_pose.pose.orientation.z = 0.0;
        publisher_angular_velocity->publish(angular_velocity_pose);

        geometry_msgs::msg::PoseStamped linear_acceleration_pose;
        linear_acceleration_pose.header = msg->header;
        linear_acceleration_pose.pose.position.x = msg->linear_acceleration.x;
        linear_acceleration_pose.pose.position.y = msg->linear_acceleration.y;
        linear_acceleration_pose.pose.position.z = msg->linear_acceleration.z;
        linear_acceleration_pose.pose.orientation.w = 1.0;
        linear_acceleration_pose.pose.orientation.x = 0.0;
        linear_acceleration_pose.pose.orientation.y = 0.0;
        linear_acceleration_pose.pose.orientation.z = 0.0;
        publisher_linear_acceleration->publish(linear_acceleration_pose);

        RCLCPP_INFO(this->get_logger(), "imu_test_node: orientation (%f, %f, %f, %f), angular_velocity (%f, %f, %f), linear_acceleration (%f, %f, %f)",
                    msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w,
                    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
                    msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<imu_test_node>());
    rclcpp::shutdown();
    return 0;
}
