import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class SmartWalker(Node):
    def __init__(self):
        super().__init__('smart_walker')

        # === Parameters (Tunable constants) ===
        self.FWD = 0.25               # Base forward speed (m/s)
        self.YAW = 0.7                # Base angular speed (rad/s)
        self.FRONT_TH = 0.60          # Threshold distance for obstacles in front (m)
        self.SIDE_TH  = 0.40          # Threshold for side obstacles (m)
        self.GOAL = (5.0, -3.5)       # Target position (Stage world)
        self.GOAL_R = 0.35            # Radius for goal reached condition (m)
        self.TARGET_DIST = 12.0       # Required travel distance 

        # === ROS2 Interfaces ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/base_scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/ground_truth', self.odom_cb, 10)

        # Timer for publishing velocity commands at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_cmd)

        # === State Variables ===
        self.lin = 0.0
        self.ang = 0.0
        self.pose = None              # (x, y, yaw)
        self.start_pos = None         # starting (x0, y0)
        self.in_avoid = False         # obstacle-avoid flag
        self.reached = False          # stop flag

        self.get_logger().info("SmartWalker node initialized.")

    # --- Odometry callback ---
    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation

        # extract yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.pose = (x, y, yaw)
        if self.start_pos is None:
            self.start_pos = (x, y)

    # --- Compute total distance traveled ---
    def dist_traveled(self):
        if self.start_pos is None or self.pose is None:
            return 0.0
        x0, y0 = self.start_pos
        x, y, _ = self.pose
        return math.hypot(x - x0, y - y0)

    # --- LIDAR callback ---
    def scan_cb(self, msg: LaserScan):
        if self.reached:
            return

        right = [r for r in msg.ranges[10:60] if not math.isinf(r)]
        front = [r for r in msg.ranges[60:121] if not math.isinf(r)]
        left  = [r for r in msg.ranges[121:171] if not math.isinf(r)]

        dR = min(right) if right else 100.0
        dF = min(front) if front else 100.0
        dL = min(left) if left else 100.0

        # --- Obstacle avoidance ---
        if self.in_avoid:
            if dF < self.FRONT_TH:
                self.lin = 0.0
                self.ang = self.YAW if dL > dR else -self.YAW
                return
            else:
                self.in_avoid = False

        if dF < self.FRONT_TH:
            self.in_avoid = True
            self.lin = 0.0
            self.ang = self.YAW if dL > dR else -self.YAW
            return

        if dL < self.SIDE_TH:
            self.lin = 0.5 * self.FWD
            self.ang = -0.7 * self.YAW
            return

        if dR < self.SIDE_TH:
            self.lin = 0.5 * self.FWD
            self.ang = 0.7 * self.YAW
            return

        # --- Move toward goal ---
        if self.pose is not None:
            x, y, yaw = self.pose
            gx, gy = self.GOAL
            dx, dy = gx - x, gy - y
            dist_to_goal = math.hypot(dx, dy)

            # Stop if goal reached
            if dist_to_goal < self.GOAL_R:
                self.lin = 0.0
                self.ang = 0.0
                self.reached = True
                self.get_logger().info("Goal reached successfully!")
                return

            # Stop if traveled too far (prevent higher-than-12 error)
            total = self.dist_traveled()
            if total > self.TARGET_DIST:
                self.lin = 0.0
                self.ang = 0.0
                self.reached = True
                self.get_logger().warn(
                    f"⚠️ Went beyond {self.TARGET_DIST:.1f} m ({total:.2f} m traveled) — stopping!"
                )
                return

            # Compute heading correction
            heading = math.atan2(dy, dx)
            err = self._angle_norm(heading - yaw)
            self.ang = max(-self.YAW, min(self.YAW, 1.2 * err))
            self.lin = self.FWD * (0.2 if abs(err) > 0.7 else 1.0)
        else:
            self.lin = self.FWD
            self.ang = 0.0

    # --- Utility ---
    def _angle_norm(self, a):
        while a > math.pi:  a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.lin
        msg.angular.z = self.ang
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SmartWalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
