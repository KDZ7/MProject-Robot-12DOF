import math
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, TransformListener
from tf2_ros.buffer import Buffer
from turtlesim.srv import Spawn


class Tf2Listener(Node):
    def __init__(self):
        super().__init__("frame_tf2_listener")
        # source_frame ---> target_frame
        self.source_frame = (
            self.declare_parameter("--source-frame", "")
            .get_parameter_value()
            .string_value
        )
        self.target_frame = (
            self.declare_parameter("--target-frame", "")
            .get_parameter_value()
            .string_value
        )
        self.service_comm = (
            self.declare_parameter("--service-comm", "")
            .get_parameter_value()
            .string_value
        )
        if not self.source_frame or not self.target_frame:
            self.get_logger().error(
                "No source, target frame or service communication provided."
            )
            raise ValueError(
                "No source, target frame or service communication provided."
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.spawner = self.create_client(Spawn, self.service_comm)
        self.frame_spawning_service_ready = False
        self.frame_spawned = False
        self.publisher = self.create_publisher(Twist, f"{self.source_frame}/cmd_vel", 1)
        self.timer = self.create_timer(1.0, self.handle_timer)

    def handle_timer(self):
        if self.frame_spawning_service_ready:
            if self.frame_spawned:
                try:
                    t = self.tf_buffer.lookup_transform(
                        self.target_frame, self.source_frame, rclpy.time.Time()
                    )
                except TransformException as ex:
                    self.get_logger().info(
                        f"Could not transform {self.source_frame} to {self.target_frame}: {ex}"
                    )
                    return
                msg = Twist()
                scale_rotation_rate = 1.0
                msg.angular.z = scale_rotation_rate * math.atan2(
                    t.transform.translation.y, t.transform.translation.x
                )
                scale_forward_speed = 0.5
                msg.linear.x = scale_forward_speed * math.sqrt(
                    t.transform.translation.x**2 + t.transform.translation.y**2
                )
                self.publisher.publish(msg)
            else:
                if self.result.done():
                    self.get_logger().info("Frame spawned.")
                    self.frame_spawned = True
                else:
                    self.get_logger().info("Spawn is not finished.")
        else:
            if self.spawner.service_is_ready():
                self.get_logger().info("Service is ready.")
                request = Spawn.Request()
                request.name = self.source_frame
                request.x = 5.0
                request.y = 5.0
                request.theta = 0.0
                request.name = self.source_frame
                self.result = self.spawner.call_async(request)
                self.frame_spawning_service_ready = True
            else:
                self.get_logger().info("Service is not ready.")


def main():
    rclpy.init()
    try:
        node = Tf2Listener()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
