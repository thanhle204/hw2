import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Walk(Node):
    def __init__(self):
        super().__init__('walk_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/base_scan', self.sensor_callback, 10
        )

        self.max_linear_speed = 0.25
        self.min_linear_speed = 0.05
        self.max_angular_speed = 1.5
        self.move_cmd = Twist()

        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')

        self.timer = self.create_timer(0.1, self.timer_callback)

    def sensor_callback(self, msg: LaserScan):
        ranges = msg.ranges
        n = len(ranges)

        self.left = min(ranges[int(3*n/4):n])      
        self.front = min(ranges[int(n/3):int(2*n/3)])  
        self.right = min(ranges[0:int(n/4)])       

        self.get_logger().info(
            f'L: {self.left:.2f}, F: {self.front:.2f}, R: {self.right:.2f}'
        )

    def timer_callback(self):
        self.move_cmd = Twist()

        if self.front < 1.0:
            # Obstacle nearby → slow down and turn
            proximity = max(0.0, min(1.0, (1.0 - self.front)))  # 0 to 1
            self.move_cmd.linear.x = self.max_linear_speed * (1 - proximity)
            self.move_cmd.angular.z = self.max_angular_speed * proximity

            # Direction choice
            if self.left < 0.8 and self.right < 0.8:
                self.move_cmd.linear.x = -0.1  # back up
                self.get_logger().info('Backing up!')
            elif self.left > self.right:
                self.move_cmd.angular.z = abs(self.move_cmd.angular.z)  # turn left
                self.get_logger().info('Turning LEFT')
            else:
                self.move_cmd.angular.z = -abs(self.move_cmd.angular.z)  # turn right
                self.get_logger().info('Turning RIGHT')
        else:
            # Clear path
            self.move_cmd.linear.x = self.max_linear_speed
            self.move_cmd.angular.z = 0.0
            self.get_logger().info('Moving FORWARD')

        # Stop if something is dangerously close
        if self.front < 0.3:
            self.move_cmd.linear.x = 0.0
            self.get_logger().info('EMERGENCY STOP!')

        self.cmd_pub.publish(self.move_cmd)
