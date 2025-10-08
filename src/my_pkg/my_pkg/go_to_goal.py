#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry


class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal_final')

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

        # PID parameters (tuned for smooth, real-world motion)
        self.Kp_lin = 0.4
        self.Ki_lin = 0.0
        self.Kd_lin = 0.1

        self.Kp_ang = 0.7
        self.Ki_ang = 0.0
        self.Kd_ang = 0.05

        # PID state
        self.prev_lin_error = 0.0
        self.integral_lin = 0.0
        self.prev_ang_error = 0.0
        self.integral_ang = 0.0
        self.derivative_ang_smooth = 0.0

        # Smoothing memory
        self.linear_prev = 0.0

        # Control loop timer
        self.timer_period = 0.2  # 5 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info("🚀 GoToGoal Final Node Initialized.")

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def goal_callback(self, msg: Point):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.get_logger().info(f"🎯 New goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}")

    def control_loop(self):
        if self.goal_x is None or self.goal_y is None:
            return

        dt = self.timer_period

        # --- Compute errors ---
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance_error = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # normalize

        # --- Linear PID ---
        self.integral_lin += distance_error * dt
        derivative_lin = (distance_error - self.prev_lin_error) / dt
        linear_output = (
            self.Kp_lin * distance_error
            + self.Ki_lin * self.integral_lin
            + self.Kd_lin * derivative_lin
        )

        # --- Angular PID with smoothing ---
        self.integral_ang += angle_error * dt
        raw_derivative_ang = (angle_error - self.prev_ang_error) / dt
        alpha = 0.8  # low-pass filter
        self.derivative_ang_smooth = (
            alpha * self.derivative_ang_smooth + (1 - alpha) * raw_derivative_ang
        )
        angular_output = (
            self.Kp_ang * angle_error
            + self.Ki_ang * self.integral_ang
            + self.Kd_ang * self.derivative_ang_smooth
        )

        # Save previous errors
        self.prev_lin_error = distance_error
        self.prev_ang_error = angle_error

        # --- Compose command ---
        cmd = Twist()

        # Stop when goal reached
        if distance_error < 0.05:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("✅ Goal reached!")
        else:
            # Allow rotation until roughly aligned
            if abs(angle_error) > 0.15:
                cmd.linear.x = 0.0
                cmd.angular.z = angular_output
            else:
                # Move forward while fine-tuning heading
                cmd.linear.x = max(0.05, linear_output)  # ensure small forward creep
                cmd.angular.z = angular_output

        # --- Angular deadband to remove jitter ---
        if abs(angular_output) < 0.05:
            cmd.angular.z = 0.0

        # --- Clamp velocities ---
        cmd.linear.x = max(min(cmd.linear.x, 0.25), -0.25)
        cmd.angular.z = max(min(cmd.angular.z, 0.8), -0.8)

        # --- Smooth linear velocity ramp ---
        cmd.linear.x = 0.8 * self.linear_prev + 0.2 * cmd.linear.x
        self.linear_prev = cmd.linear.x

        # --- Publish command ---
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
