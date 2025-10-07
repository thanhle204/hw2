# walk.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class Walker(Node):
    def __init__(self):
        super().__init__('walker')
        
        # Publisher to send velocity commands to the robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to get LIDAR sensor data
        self.subscription = self.create_subscription(
            LaserScan,
            '/base_scan',
            self.sensor_callback,
            10)
            
        # Timer to run the main logic loop at 10 Hz
        self.timer = self.create_timer(0.1, self.main_logic_loop)

        # --- Constants and Variables ---
        self.FORWARD_SPEED = 0.2  # meters/sec
        self.ROTATION_SPEED = 0.5 # radians/sec
        
        # Threshold distance to detect an obstacle (in meters)
        self.OBSTACLE_THRESHOLD = 0.5 

        # State machine for the robot's behavior
        # 0: Go forward
        # 1: Obstacle detected, turning
        self.state = 0
        
        self.get_logger().info("Walker node has been started.")

    def sensor_callback(self, msg):
        """
        This function is called every time new LIDAR data is received.
        It processes the data to decide the robot's state.
        """
        # The 'ranges' array contains distance readings from the LIDAR.
        # The pioneer2dx in Stage has a 180-degree LIDAR scan in front of it.
        # The array has 181 elements, from right (0) to left (180).
        # Index 90 is directly in front.

        # We will focus on a 40-degree cone in front of the robot.
        # This corresponds to indices 70 through 110.
        front_view = msg.ranges[70:111]
        
        # Find the minimum distance in this front-facing cone
        # We use a filter to ignore 'inf' values if any
        min_distance = min([r for r in front_view if not math.isinf(r)])

        # --- State Machine Logic ---
        if self.state == 0: # If currently moving forward
            if min_distance < self.OBSTACLE_THRESHOLD:
                # Obstacle detected! Change state to "turning"
                self.state = 1
                self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m. Changing state to TURNING.')
        
        elif self.state == 1: # If currently turning
            if min_distance >= self.OBSTACLE_THRESHOLD:
                # Path is clear! Change state back to "go forward"
                self.state = 0
                self.get_logger().info(f'Path is clear. Changing state to FORWARD.')
    
    def main_logic_loop(self):
        """
        This function runs on a timer and sends the velocity commands
        based on the current state.
        """
        # Create a Twist message to control robot velocity
        twist_msg = Twist()

        if self.state == 0:
            # Go forward
            twist_msg.linear.x = self.FORWARD_SPEED
            twist_msg.angular.z = 0.0
        elif self.state == 1:
            # Turn left
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = self.ROTATION_SPEED
        
        # Publish the command
        self.cmd_pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    walker_node = Walker()
    rclpy.spin(walker_node)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    walker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
