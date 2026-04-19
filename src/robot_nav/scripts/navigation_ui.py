#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class NavigationUI(Node):

    def __init__(self):
        super().__init__('navigation_ui')

        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.get_logger().info("Navigation UI ready")

        self.timer = self.create_timer(0.1, self.user_input_loop)

    def user_input_loop(self):
        # stop timer after first run (we use blocking input)
        self.timer.cancel()

        try:
            while rclpy.ok():

                print("\n--- Insert goal ---")
                x = float(input("x: "))
                y = float(input("y: "))
            #   theta = float(input("theta (ignored for now or quaternion later): "))

                msg = PoseStamped()
                msg.header.frame_id = "odom"
                msg.header.stamp = self.get_clock().now().to_msg()

                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.position.z = 0.0

                msg.pose.orientation.w = 1.0

                self.publisher_.publish(msg)

                self.get_logger().info(
                    f"Goal sent: x={x}, y={y}" 
                ) #theta={theta}

        except KeyboardInterrupt:
            self.get_logger().info("UI stopped")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()