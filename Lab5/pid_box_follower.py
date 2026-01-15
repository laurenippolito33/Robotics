import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from simple_pid import PID
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
import matplotlib.pyplot as plt
from threading import Thread
import time

class PIDBoxFollower(Node):
    def __init__(self):
        super().__init__('pid_box_follower')

        # Parameters
        self.declare_parameter('desired_distance', 0.5)
        self.declare_parameter('max_follow_distance', 1.5)
        self.declare_parameter('fov_deg', 180)
        self.declare_parameter('pid_kp', 1.5)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.1)

        self.desired_distance = self.get_parameter('desired_distance').value
        self.max_follow_distance = self.get_parameter('max_follow_distance').value
        self.fov_deg = self.get_parameter('fov_deg').value
        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value

        # PID controllers
        self.pid_linear = PID(kp, ki, kd, setpoint=self.desired_distance)
        self.pid_linear.output_limits = (-0.2, 0.3)

        self.pid_angular = PID(kp*2, ki, kd, setpoint=0.0)
        self.pid_angular.output_limits = (-1.0, 1.0)

        # QoS for LIDAR
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscribers / Publishers
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # For plotting
        self.distance_log = []
        self.linear_log = []
        self.angular_log = []
        self.time_log = []
        self.start_time = time.time()

        # Start plotting thread
        self.plot_thread = Thread(target=self.plot_pid_data, daemon=True)
        self.plot_thread.start()

        self.get_logger().info('PIDBoxFollower started with live plotting.')

    def scan_callback(self, msg: LaserScan):
        num_points = len(msg.ranges)
        center_index = num_points // 2

        half_fov_rad = np.deg2rad(self.fov_deg) / 2.0
        half_span = int(half_fov_rad / msg.angle_increment)

        start_idx = max(center_index - half_span, 0)
        end_idx = min(center_index + half_span, num_points)

        front_ranges = np.array(msg.ranges[start_idx:end_idx])
        front_ranges = front_ranges[np.isfinite(front_ranges)]

        if front_ranges.size == 0:
            self.get_logger().warn("No valid LIDAR data in front sector.")
            return

        min_dist = np.min(front_ranges)
        min_idx = np.argmin(front_ranges)
        angle_to_box = (start_idx + min_idx - center_index) * msg.angle_increment
        angular_cmd = self.pid_angular(-angle_to_box)

        if min_dist > self.max_follow_distance:
            linear_cmd = 0.0
            angular_cmd = 0.0
        else:
            linear_cmd = self.pid_linear(min_dist)

        cmd = Twist()
        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd
        self.pub.publish(cmd)

        # Logging for plotting
        current_time = time.time() - self.start_time
        self.time_log.append(current_time)
        self.distance_log.append(min_dist)
        self.linear_log.append(linear_cmd)
        self.angular_log.append(angular_cmd)

        self.get_logger().info(
            f'Dist: {min_dist:.2f} m | Linear: {linear_cmd:.2f} | '
            f'Angular: {angular_cmd:.2f} | Angle_to_box: {angle_to_box:.2f}'
        )

    def plot_pid_data(self):
        plt.ion()
        fig, ax = plt.subplots(2, 1, figsize=(8, 6))

        while rclpy.ok():
            if self.time_log:
                ax[0].cla()
                ax[0].plot(self.time_log, self.distance_log, label='Distance')
                ax[0].plot(self.time_log, [self.desired_distance]*len(self.time_log), 'r--', label='Desired')
                ax[0].set_ylabel('Distance (m)')
                ax[0].legend()
                ax[0].grid(True)

                ax[1].cla()
                ax[1].plot(self.time_log, self.linear_log, label='Linear PID')
                ax[1].plot(self.time_log, self.angular_log, label='Angular PID')
                ax[1].set_xlabel('Time (s)')
                ax[1].set_ylabel('Velocity')
                ax[1].legend()
                ax[1].grid(True)

                plt.pause(0.05)
            else:
                time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = PIDBoxFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

