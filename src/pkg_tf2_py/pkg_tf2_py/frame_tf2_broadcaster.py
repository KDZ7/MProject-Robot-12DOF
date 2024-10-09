import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose


def quaternion_from_euler(roll, pitch, yaw):
    cos_half_roll = math.cos(roll / 2.0)
    sin_half_roll = math.sin(roll / 2.0)
    cos_half_pitch = math.cos(pitch / 2.0)
    sin_half_pitch = math.sin(pitch / 2.0)
    cos_half_yaw = math.cos(yaw / 2.0)
    sin_half_yaw = math.sin(yaw / 2.0)
    q = np.empty((4,), dtype=np.float64)  # i, j, k, w
    q[0] = (
        sin_half_roll * cos_half_pitch * cos_half_yaw
        - cos_half_roll * sin_half_pitch * sin_half_yaw
    )
    q[1] = (
        cos_half_roll * sin_half_pitch * cos_half_yaw
        + sin_half_roll * cos_half_pitch * sin_half_yaw
    )
    q[2] = (
        cos_half_roll * cos_half_pitch * sin_half_yaw
        - sin_half_roll * sin_half_pitch * cos_half_yaw
    )
    q[3] = (
        cos_half_roll * cos_half_pitch * cos_half_yaw
        + sin_half_roll * sin_half_pitch * sin_half_yaw
    )
    return q


class FrameTf2Broadcaster(Node):
    def __init__(self):
        super().__init__("frame_tf2_broadcaster")
        self.frame_id = (
            self.declare_parameter("--frame-id", "parent")
            .get_parameter_value()
            .string_value
        )
        self.child_frame_id = (
            self.declare_parameter("--child-frame-id", "child")
            .get_parameter_value()
            .string_value
        )
        self.topic_position = (
            self.declare_parameter("--topic-position", "")
            .get_parameter_value()
            .string_value
        )
        if not self.topic_position:
            self.get_logger().error("No topic position provided.")
            raise ValueError("No topic position provided.")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Pose, self.topic_position, self.handle_listener_pose, 1
        )
        self.subscription  # prevent unused variable warning

    def handle_listener_pose(self, msg):
        tf_stamped = TransformStamped()
        tf_stamped.header.stamp = self.get_clock().now().to_msg()
        tf_stamped.header.frame_id = self.frame_id
        tf_stamped.child_frame_id = self.child_frame_id
        tf_stamped.transform.translation.x = msg.x
        tf_stamped.transform.translation.y = msg.y
        tf_stamped.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, msg.theta)
        tf_stamped.transform.rotation.x = q[0]
        tf_stamped.transform.rotation.y = q[1]
        tf_stamped.transform.rotation.z = q[2]
        tf_stamped.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf_stamped)


def main():
    logger = rclpy.logging.get_logger("logger")
    rclpy.init()
    try:
        node = FrameTf2Broadcaster()
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt, shutting down...")
    except Exception as e:
        logger.error(f"An error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
