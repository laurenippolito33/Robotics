import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import sqrt
from simple_pid import PID

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

class PIDMove(Node):
    def __init__(self):
        super().__init__('pid_move_forward')

        self.x0 = None
        self.x = 0.0

        
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

       

        # PID controller
        self.pid = PID(1.5, 0.0, 0.1, setpoint=3.0)
        self.pid.output_limits = (0.0, 0.2)

       
        # 20 Hz timer
        self.timer = self.create_timer(0.05, self.control_loop)
      
        
        self.time_data = []
        self.error_data = []
        self.output_data = []
        self.start_time = time.time()
        
        self.plot_thread = threading.Thread(target=self.plot_live)
        self.plot_thread.daemon = True
        self.plot_thread.start()
        
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        if self.x0 is None:
            self.x0 = self.x

        
    def control_loop(self):
        if self.x0 is None:
           return  # wait for odom

        distance = abs(self.x - self.x0)
        error = self.pid.setpoint - distance
        v = self.pid(distance)

        cmd = Twist(); cmd.linear.x = v; cmd.angular.z = 0.0
        #msg.linear.x = v
        self.pub.publish(cmd)
        print('here')
        
        t = time.time() - self.start_time
        self.time_data.append(t)
        self.error_data.append(error)
        self.output_data.append(v)


        if abs(3.0 - distance) < 0.01:
            self.pub.publish(Twist())
            self.get_logger().info("Reached 3 meters.")
            rclpy.shutdown()
            
    def plot_live(self):
        """Live matplotlib plot that updates as data comes in."""
        plt.style.use('seaborn-darkgrid')
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 6))
        fig.suptitle('PID Response')
         
        def update(_):
            if not self.time_data:
                return
            ax1.clear(); ax2.clear()
            ax1.plot(self.time_data, self.error_data, color='red')
            ax1.set_ylabel('Error (m)')
            ax1.set_xlabel('Time (s)')
            ax1.set_title('Distance Error')

            ax2.plot(self.time_data, self.output_data, color='blue')
            ax2.set_ylabel('Velocity (m/s)')
            ax2.set_xlabel('Time (s)')
            ax2.set_title('PID Output (cmd_vel)')

        ani = FuncAnimation(fig, update, interval=200)
        plt.tight_layout()
        plt.show()

def main():
    rclpy.init()
    node = PIDMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
