
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import math
import time

# ---------------- PID Controller ----------------
class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.min_output, self.max_output = output_limits
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def __call__(self, measurement):
        now = time.time()
        error = self.setpoint - measurement
        dt = now - self.last_time if self.last_time else 0.0
        self.last_time = now

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.integral += error * dt
        out = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.min_output is not None:
            out = max(self.min_output, out)
        if self.max_output is not None:
            out = min(self.max_output, out)

        self.prev_error = error
        return out

# ---------------- Wall Follower Node ----------------
class WallFollower(Node):
    def __init__(self, side='right', d_star=0.3):
        super().__init__('wall_follower')

        self.side = side
        self.d_star = d_star
        self.min_safe_distance = 0.2
        self.max_forward_speed = 0.2
        self.min_forward_speed = 0.05

        self.last_scan = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.timer = self.create_timer(0.05, self.on_timer)

        self.get_logger().info(f"WallFollower initialized: side={self.side}, target distance={self.d_star} m")

    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def get_wall_distance_and_angle(self, scan: LaserScan):
        """
        Returns (side_distance, wall_angle) relative to robot.
        Uses two rays: one directly to the side, one slightly forward.
        """
        if scan is None or not scan.ranges:
            return None, None

        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        # Corrected for LDS-02: right side ≈ 3π/2 (4.71 rad), left ≈ π/2 (1.57 rad)
        if self.side == 'right':
            angle1 = 3 * math.pi / 2           # ~270° right
            angle2 = angle1 - math.radians(20) # slightly backward-right
        else:
            angle1 = math.pi / 2               # ~90° left
            angle2 = angle1 + math.radians(20) # slightly backward-left

        def get_range_at_angle(target_angle):
            # Wrap-around handling
            target_angle = (target_angle + 2 * math.pi) % (2 * math.pi)
            idx = (np.abs(angles - target_angle)).argmin()
            r = ranges[idx]
            return r if np.isfinite(r) and r > 0.05 else None

        r1 = get_range_at_angle(angle1)
        r2 = get_range_at_angle(angle2)

        if r1 is None or r2 is None:
            return None, None

        wall_theta = math.atan2(r2 * math.sin(angle2) - r1 * math.sin(angle1),
                                r2 * math.cos(angle2) - r1 * math.cos(angle1))
        side_dist = r1
        return side_dist, wall_theta

    def on_timer(self):
        if self.last_scan is None:
            self.get_logger().warn("No LiDAR data received yet")
            return

        side_dist, wall_theta = self.get_wall_distance_and_angle(self.last_scan)
        cmd = Twist()

        if side_dist is None:
            self.get_logger().warn("Side wall lost, moving forward cautiously")
            cmd.linear.x = self.min_forward_speed
            cmd.angular.z = 0.0
            self.pub.publish(cmd)
            return

        # Main control law
        K_dist = 0.6
        K_angle = 0.3 if self.side == 'right' else -0.3  # flip direction based on side
        omega = -K_dist * (side_dist - self.d_star) - K_angle * wall_theta
        omega = max(-0.5, min(0.5, omega))

        front_ranges = np.array(self.last_scan.ranges)
        valid_front = front_ranges[np.isfinite(front_ranges)]
        front_min = np.min(valid_front) if len(valid_front) > 0 else 1.0

        v = self.max_forward_speed
        if front_min < 0.5:
            v = max(self.min_forward_speed, self.max_forward_speed * front_min / 0.5)
        if front_min < self.min_safe_distance:
            v = 0.0
            omega = 0.0
            self.get_logger().warn(f"Too close to obstacle ahead: stopping! Distance={front_min:.2f} m")

        cmd.linear.x = v
        cmd.angular.z = omega
        self.pub.publish(cmd)

        self.get_logger().info(
            f"WallFollower: side_dist={side_dist:.2f}, wall_theta={math.degrees(wall_theta):.1f}°, "
            f"front_min={front_min:.2f}, v={v:.2f}, omega={omega:.2f}"
        )

def main():
    rclpy.init()
    node = WallFollower(side='right', d_star=0.3)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

