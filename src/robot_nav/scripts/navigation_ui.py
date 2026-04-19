#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
import math


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
        self.timer.cancel()

        try:
            while rclpy.ok():

                print("\n--- MENU ---")
                print("g = send goal")
                print("s = STOP robot")
                cmd = input("Command: ").strip()
                
                # STOP
                if cmd == "s":
                    msg = PoseStamped()
                    msg.header.frame_id = "STOP"
                    self.publisher_.publish(msg)
                    self.get_logger().warn("STOP sent")
                    continue

                # SEND GOAL
                elif cmd == "g":
                    print("\n--- Insert goal ---")

                    # X
                    while True:
                        try:
                            x = float(input("x: "))
                            break
                        except ValueError:
                            print("ERROR: invalid x. Insert a valid value of x")

                    # Y
                    while True:
                        try:
                            y = float(input("y: "))
                            break
                        except ValueError:
                            print("ERROR: invalid y. Insert a valid value of y")

                    # THETA
                    print("\n theta (default = 0)")
                    print("d = degrees")
                    print("r = radians")

                    mode = input("Unit (d/r, default=r): ") or "r"
                    mode = mode.lower()

                    while mode not in ("d", "r"):
                        print("Error: insert 'd' or 'r'")
                        mode = input("Unit (d/r, default=r): ") or "r"
                        mode = mode.lower()

                    while True:
                        try:
                            theta_input = input("theta (default=0): ") or "0"
                            theta = float(theta_input)
                            break
                        except ValueError:
                            print("Error: you have to insert a number")

                    if mode == "d":
                        theta = math.radians(theta)

                    print("final theta (radians):", theta)

                    # BUILD MESSAGE
                    msg = PoseStamped()
                    msg.header.frame_id = "odom"
                    msg.header.stamp = self.get_clock().now().to_msg()

                    msg.pose.position.x = x
                    msg.pose.position.y = y
                    msg.pose.position.z = 0.0

                    qz = math.sin(theta / 2.0)
                    qw = math.cos(theta / 2.0)

                    msg.pose.orientation.z = qz
                    msg.pose.orientation.w = qw

                    self.publisher_.publish(msg)

                    self.get_logger().info(
                        f"Goal sent: x={x}, y={y}, theta={theta}"
                    )

                # INVALID COMMAND
                else:
                    print("invalid command")

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

