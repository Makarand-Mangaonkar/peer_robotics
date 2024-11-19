#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class LidarOdometryNavigation(Node):
    def __init__(self):
        super().__init__('lidar_odometry_navigation')

        # Subscription to LiDAR odometry (from rf2o_laser_odometry)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom_rf2o',  # LiDAR odometry topic
            self.odom_callback,
            10)

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Robot's current pose and the goal pose
        self.current_pose = None
        self.goal_x = 0.0
        self.goal_y = 0.0

        # PID controllers for linear and angular velocity
        self.linear_pid = PIDController(0.5, 0.0, 0.1)  # Modify gains if necessary
        self.angular_pid = PIDController(2.0, 0.0, 0.1)  # Modify gains if necessary

        self.get_logger().info('LiDAR Odometry Navigation Node Started.')

        # Ask for goal position in terminal
        self.get_goal_position()

    def get_goal_position(self):
        """Get the goal position from terminal input."""
        try:
            self.goal_x = float(input("Enter goal x coordinate: "))
            self.goal_y = float(input("Enter goal y coordinate: "))
            self.get_logger().info(f'Goal set to: x={self.goal_x}, y={self.goal_y}')
        except ValueError:
            self.get_logger().error('Invalid input. Please enter numeric values.')
            self.get_goal_position()

    def odom_callback(self, msg):
        """Callback function for LiDAR odometry."""
        self.current_pose = msg.pose.pose
        self.navigate_to_goal()

    def navigate_to_goal(self):
        """Navigate towards the goal using LiDAR odometry."""
        if self.current_pose is not None:
            # Calculate the distance and angle to the goal
            dx = self.goal_x - self.current_pose.position.x
            dy = self.goal_y - self.current_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)

            # Calculate the angle to the goal
            angle_to_goal = math.atan2(dy, dx)

            # Get the current yaw of the robot from LiDAR odometry
            current_yaw = self.get_yaw_from_pose(self.current_pose)

            # Calculate angular error
            angular_error = self.normalize_angle(angle_to_goal - current_yaw)

            # Use PID controllers for velocity control
            linear_velocity = self.linear_pid.control(distance, 0.1)
            angular_velocity = self.angular_pid.control(angular_error, 0.1)

            # Create and publish the velocity command
            twist = Twist()

            if distance > 0.1:  # Keep moving until close to the goal
                twist.linear.x = min(linear_velocity, 0.5)  # Limit the max speed
                twist.angular.z = angular_velocity
            else:
                # Stop the robot when it's close enough
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Goal reached!')
                self.goal_x = None  # Clear the goal to stop further commands

            self.cmd_vel_publisher.publish(twist)

    def get_yaw_from_pose(self, pose):
        """Extract yaw from quaternion."""
        orientation_q = pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Normalize angle between -pi and pi."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


class PIDController:
    """A simple PID controller implementation."""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def control(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output


def main(args=None):
    rclpy.init(args=args)
    node = LidarOdometryNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
