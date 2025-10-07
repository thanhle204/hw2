# walk.py (v3 - Goal-Oriented Navigation)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry  # Import Odometry message
import math

class GoalOrientedWalker(Node):
    def __init__(self):
        super().__init__('goal_oriented_walker')
        
        # --- Publishers and Subscribers ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.sensor_callback,
            10)
        # NEW: Subscriber for odometry to track position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ground_truth',
            self.odometry_callback,
            10)
            
        self.timer = self.create_timer(0.1, self.publish_command)

        # --- Constants for Tuning ---
        # NOTE: You might need to change TARGET_DISTANCE based on the starting position
        # Position 1 (-7, -7) -> Target: 19.0
        # Position 2 (5, -3.5) -> Target: 14.0
        self.TARGET_DISTANCE = 19.0      # Set this to the max distance for the current test
        self.FORWARD_SPEED = 0.4         # Increased for better distance coverage
        self.ROTATION_SPEED = 0.8        # Increased for faster turns
        self.FRONT_THRESHOLD = 0.7       # Increased slightly for earlier turns

        # --- State and Command Variables ---
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        
        # --- Position and Goal Tracking Variables ---
        self.start_pos = None
        self.current_distance = 0.0
        self.goal_reached = False
        
        self.get_logger().info(f"GoalOrientedWalker started. Target distance: {self.TARGET_DISTANCE}m")

    def odometry_callback(self, msg):
        """
        NEW: Processes odometry data to track distance from start.
        """
        # Get current position
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # If this is the first odometry message, record the starting position
        if self.start_pos is None:
            self.start_pos = (current_x, current_y)
            self.get_logger().info(f"Starting position recorded at: ({self.start_pos[0]:.2f}, {self.start_pos[1]:.2f})")

        # Calculate the straight-line distance from the start
        dx = current_x - self.start_pos[0]
        dy = current_y - self.start_pos[1]
        self.current_distance = math.sqrt(dx*dx + dy*dy)

        # Check if the goal has been reached
        if self.current_distance >= self.TARGET_DISTANCE and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info(f"SUCCESS: Goal of {self.TARGET_DISTANCE}m reached!")
            self.get_logger().info("Robot stopping.")


    def sensor_callback(self, msg):
        """
        Processes LIDAR data to determine robot's next move.
        This logic is now SIMPLIFIED to prioritize moving straight.
        """
        # We only make decisions if the goal has not been reached yet
        if self.goal_reached:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0
            return

        # Using a wider range for the LIDAR based on your previous code.
        # Index 0: Right | Index 90: Front | Index 180: Left
        front_view = msg.ranges[60:121]
        left_view = msg.ranges[121:171]
        right_view = msg.ranges[10:60]

        min_dist_front = min([r for r in front_view if not math.isinf(r)] or [100])
        min_dist_left = min([r for r in left_view if not math.isinf(r)] or [100])
        min_dist_right = min([r for r in right_view if not math.isinf(r)] or [100])
        
        # --- REVISED Decision Logic ---
        # The logic is now much simpler to avoid circling.
        
        # Priority 1: Obstacle directly ahead. Stop and turn.
        if min_dist_front < self.FRONT_THRESHOLD:
            self.get_logger().info(f'FRONT BLOCKED ({min_dist_front:.2f}m). Turning.')
            self.target_linear_velocity = 0.0
            # Turn toward the side with more space
            if min_dist_left > min_dist_right:
                self.target_angular_velocity = self.ROTATION_SPEED  # Turn left
            else:
                self.target_angular_velocity = -self.ROTATION_SPEED # Turn right
        
        # Default Case: Path is clear. Go straight.
        else:
            self.get_logger().info(f'Path clear. Moving forward. Current distance: {self.current_distance:.2f}m')
            self.target_linear_velocity = self.FORWARD_SPEED
            self.target_angular_velocity = 0.0

    def publish_command(self):
        """
        Creates and publishes the Twist message.
        """
        twist_msg = Twist()
        # If goal is reached, force velocities to zero regardless of sensor readings
        if self.goal_reached:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        else:
            twist_msg.linear.x = self.target_linear_velocity
            twist_msg.angular.z = self.target_angular_velocity
            
        self.cmd_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    walker_node = GoalOrientedWalker()
    rclpy.spin(walker_node)
    
    walker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
