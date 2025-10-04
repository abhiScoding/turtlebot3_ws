#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        # Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Point, '/goal_point', self.goal_callback, 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Goal state
        self.goal_x = None
        self.goal_y = None

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Orientation -> yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg: Point):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.get_logger().info(f"Received new goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}")

    def control_loop(self):
        if self.goal_x is None or self.goal_y is None:
            return  # no goal yet

        # Compute distance and angle to goal
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.yaw

        # Normalize angle error to [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()

        if distance > 0.05:  # not yet at goal
            if abs(angle_error) > 0.1:
                # Rotate towards goal
                cmd.angular.z = 0.5 * angle_error
            else:
                # Move forward
                cmd.linear.x = 0.2 * distance
                cmd.angular.z = 0.3 * angle_error
        else:
            # Stop when goal reached
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("Goal reached!")

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()