# walk.py (v2 - Smarter Navigation)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class SmartWalker(Node):
    def __init__(self):
        super().__init__('smart_walker')
        
        # Publisher to send velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to LIDAR sensor data
        self.subscription = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.sensor_callback,
            10)
            
        # Timer to publish commands consistently
        self.timer = self.create_timer(0.1, self.publish_command)

        # --- Constants for Tuning ---
        self.FORWARD_SPEED = 0.25      # Base forward speed (m/s)
        self.ROTATION_SPEED = 0.6     # Base rotation speed (rad/s)
        self.FRONT_THRESHOLD = 0.6    # Obstacle distance threshold for the front (m)
        self.SIDE_THRESHOLD = 0.4     # Obstacle distance threshold for the sides (m)

        # --- Robot Command Variables ---
        # These are updated by the sensor_callback and used by publish_command
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        
        self.get_logger().info("SmartWalker node has been started.")

    def sensor_callback(self, msg):
        """
        Processes LIDAR data and determines the robot's next move.
        """
        # The pioneer2dx LIDAR has 181 readings over 180 degrees.
        # Index 0: Right | Index 90: Front | Index 180: Left

        # 1. Divide the LIDAR scan into three regions
        # We ignore the very edges to prevent noise from side-readings
        right_view = msg.ranges[10:60]
        front_view = msg.ranges[60:121] # A 60-degree cone in front
        left_view = msg.ranges[121:171]

        # 2. Find the minimum distance in each region, ignoring 'inf' values
        min_dist_right = min([r for r in right_view if not math.isinf(r)] or [100])
        min_dist_front = min([r for r in front_view if not math.isinf(r)] or [100])
        min_dist_left = min([r for r in left_view if not math.isinf(r)] or [100])
        
        # --- Decision Logic ---
        
        # Priority 1: Obstacle directly ahead.
        if min_dist_front < self.FRONT_THRESHOLD:
            self.get_logger().info(f'FRONT BLOCKED ({min_dist_front:.2f}m). Turning.')
            # Stop moving forward
            self.target_linear_velocity = 0.0
            # Turn toward the side with more space
            if min_dist_left > min_dist_right:
                self.target_angular_velocity = self.ROTATION_SPEED # Turn left
            else:
                self.target_angular_velocity = -self.ROTATION_SPEED # Turn right

        # Priority 2: Obstacle on the left.
        elif min_dist_left < self.SIDE_THRESHOLD:
            self.get_logger().info(f'SIDE-LEFT BLOCKED ({min_dist_left:.2f}m). Steering right.')
            # Move forward but steer away from the left wall
            self.target_linear_velocity = self.FORWARD_SPEED * 0.5 # Slow down a bit
            self.target_angular_velocity = -self.ROTATION_SPEED * 0.7 # Gentle turn right

        # Priority 3: Obstacle on the right.
        elif min_dist_right < self.SIDE_THRESHOLD:
            self.get_logger().info(f'SIDE-RIGHT BLOCKED ({min_dist_right:.2f}m). Steering left.')
            # Move forward but steer away from the right wall
            self.target_linear_velocity = self.FORWARD_SPEED * 0.5 # Slow down a bit
            self.target_angular_velocity = self.ROTATION_SPEED * 0.7 # Gentle turn left
            
        # Default Case: Path is clear.
        else:
            self.get_logger().info('Path is clear. Moving forward.')
            # Go straight
            self.target_linear_velocity = self.FORWARD_SPEED
            self.target_angular_velocity = 0.0

    def publish_command(self):
        """
        Creates and publishes the Twist message.
        """
        twist_msg = Twist()
        twist_msg.linear.x = self.target_linear_velocity
        twist_msg.angular.z = self.target_angular_velocity
        self.cmd_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    smart_walker_node = SmartWalker()
    rclpy.spin(smart_walker_node)
    
    smart_walker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
