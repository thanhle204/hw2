# === Updated from groupmate's version, with comments marking what was added/changed ===

import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class GoalOrientedWalker(Node):
    def __init__(self):
        super().__init__('goal_oriented_walker')

        # ---------- Tunables ----------
        # 🔹 NEW: Define goal coordinates (the green point)
        self.GOAL = (0.0, -6.0)     # <-- change to your actual goal (x, y)
        self.GOAL_R = 0.35          # 🔹 NEW: stop if within 0.35m radius around goal

        # 🔸 CHANGED: Instead of only checking total distance, combine both goal & distance
        self.TARGET_DISTANCE = 14.0   # 12 / 14 / 19 depending on start position

        # 🔸 Slightly adjusted movement constants for smoother navigation
        self.FORWARD_SPEED = 0.35
        self.ROTATION_SPEED = 1.0
        self.FRONT_THRESHOLD = 0.60
        self.SIDE_WALL = 0.40
        self.K_lin = 0.8             # 🔹 NEW: proportional gain for linear motion
        self.K_ang = 1.8             # 🔹 NEW: proportional gain for angular correction
        self.ERR_SLOW = 0.6          # 🔹 NEW: slow down if heading error > 0.6 rad

        # ---------- State ----------
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0

        self.start_pos = None
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.yaw  = 0.0
        self.goal_reached = False
        self.distance_reached = False
        self.in_avoid = False
        self.start_time = time.time()

        # ---------- ROS ----------
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/base_scan', self.sensor_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ground_truth', self.odometry_callback, 10)
        self.timer    = self.create_timer(0.1, self.publish_command)

        self.get_logger().info(
            f"Started. Goal={self.GOAL}, goal_R={self.GOAL_R}, target_dist={self.TARGET_DISTANCE}m"
        )

    # ---------- Helpers ----------
    @staticmethod
    def _ang_norm(a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    # 🔹 NEW: Compute total distance traveled since start
    def dist_traveled(self):
        if self.start_pos is None:
            return 0.0
        dx = self.cur_x - self.start_pos[0]
        dy = self.cur_y - self.start_pos[1]
        return math.hypot(dx, dy)

    # ---------- Odometry Callback ----------
    def odometry_callback(self, msg: Odometry):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y

        # 🔹 NEW: Extract yaw angle from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.start_pos is None:
            self.start_pos = (self.cur_x, self.cur_y)
            self.get_logger().info(f"Start pos: ({self.cur_x:.2f}, {self.cur_y:.2f})")

        # 🔹 NEW: Stop once required distance reached (avoid "higher than 12" issue)
        if not self.distance_reached and self.dist_traveled() >= self.TARGET_DISTANCE:
            self.distance_reached = True
            self.get_logger().info(f"Reached target distance {self.TARGET_DISTANCE:.1f} m. Stopping.")

    # ---------- LIDAR Callback ----------
    def sensor_callback(self, msg: LaserScan):
        # 🔸 CHANGED: Stop immediately if either goal or distance is achieved
        if self.goal_reached or self.distance_reached:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0
            return

        # Split LIDAR data into regions
        right = [r for r in msg.ranges[10:60]  if r > 0.0 and not math.isinf(r)]
        front = [r for r in msg.ranges[60:121] if r > 0.0 and not math.isinf(r)]
        left  = [r for r in msg.ranges[121:171]if r > 0.0 and not math.isinf(r)]

        dR = min(right) if right else 99.0
        dF = min(front) if front else 99.0
        dL = min(left)  if left  else 99.0

        # ---------- Obstacle Avoidance ----------
        # 🔸 CHANGED: Add persistent avoidance state to turn fully until front is clear
        if self.in_avoid or dF < self.FRONT_THRESHOLD:
            self.in_avoid = dF < self.FRONT_THRESHOLD
            turn_left = dL > dR
            self.target_linear_velocity  = 0.0
            self.target_angular_velocity = self.ROTATION_SPEED if turn_left else -self.ROTATION_SPEED
            return

        # Side wall corrections
        if dL < self.SIDE_WALL:
            self.target_linear_velocity  = 0.5 * self.FORWARD_SPEED
            self.target_angular_velocity = -0.6 * self.ROTATION_SPEED
            return
        if dR < self.SIDE_WALL:
            self.target_linear_velocity  = 0.5 * self.FORWARD_SPEED
            self.target_angular_velocity =  0.6 * self.ROTATION_SPEED
            return

        # ---------- Go-to-Goal Steering ----------
        gx, gy = self.GOAL
        dx, dy = gx - self.cur_x, gy - self.cur_y
        dist_to_goal = math.hypot(dx, dy)

        # 🔹 NEW: Stop when robot enters goal radius
        if dist_to_goal <= self.GOAL_R:
            self.goal_reached = True
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0
            self.get_logger().info("✅ Goal region reached.")
            return

        # 🔹 NEW: Compute heading error (angle between robot yaw & goal direction)
        desired = math.atan2(dy, dx)
        err = self._ang_norm(desired - self.yaw)

        # 🔹 NEW: Angular velocity (Proportional control)
        ang = self.K_ang * err
        ang = max(-self.ROTATION_SPEED, min(self.ROTATION_SPEED, ang))

        # 🔹 NEW: Slow down if turning sharply, else move faster
        if abs(err) > self.ERR_SLOW:
            lin = 0.08
        else:
            lin = min(self.FORWARD_SPEED, self.K_lin * dist_to_goal)

        self.target_linear_velocity  = lin
        self.target_angular_velocity = ang

    # ---------- Timer Publish ----------
    def publish_command(self):
        twist = Twist()
        # 🔹 NEW: Add timeout stop after 295s for HW2 spec
        if self.goal_reached or self.distance_reached or (time.time() - self.start_time) > 295:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = self.target_linear_velocity
            twist.angular.z = self.target_angular_velocity
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GoalOrientedWalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
