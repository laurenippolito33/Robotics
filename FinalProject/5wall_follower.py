###turtlebot wall follower
##subscribes to arm node and waits until it recieves notice that the highlighters are placed
##follows right wall throught the maze
##when lidar detects wall in front of the turtlebot, turn right
## when lidar loses right wall, turn left

import rclpy
from std_msgs.msg import Bool
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
    def __init__(self, side='right', d_star=0.4):
        super().__init__('wall_follower')

        self.side = side
        self.d_star = d_star
        self.min_safe_distance = 0.25
        self.max_forward_speed = 0.2
        self.min_forward_speed = 0.05

        self.last_scan = None
        self.turning = False
        self.turn_start_time = None
        self.turning_right = False
        self.wall_lost_time = None
        self.wall_lost_delay = 0.1  # seconds required before turning right
        self.active = False
        self.started = False
        self.destination = False

        ##create subscriber
        self.sub = self.create_subscription(Bool, 'ready',self.activation_callback,10)
        self.sub_1 = self.create_subscription(Bool, 'destination',self.at_destination_callback,10)

        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.timer = self.create_timer(0.05, self.on_timer)
        self.get_logger().info(f"WallFollower initialized: side={self.side}, target distance={self.d_star} m")
        

    def activation_callback(self,msg):
        if not self.started and msg.data:
            self.started = True
            self.active = True
            print('starting turtlebot')

    def at_destination_callback(self,msg):
        if msg.data:
            self.destination = True
        
        
    def on_scan(self, msg: LaserScan):
        self.last_scan = msg
        

    def get_wall_distance_and_angle(self, scan: LaserScan):
        
        if scan is None or not scan.ranges:
            return None, None

        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        if self.side == 'right':
            angle1 = 3 * math.pi / 2
            angle2 = angle1 - math.radians(20)
        else:
            angle1 = math.pi / 2
            angle2 = angle1 + math.radians(20)

        def get_range_at_angle(target_angle):
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
        
        if not self.active:
            return
        
        if self.destination:
            while 1:
                print('STOPPING')
                cmd = Twist()
                cmd.linear.x = 0.05
                cmd.angular.z = 0.0
                self.pub.publish(cmd)
            #time.sleep(30)
            
        
        # If turning, ignore wall following and just perform timed turn
    
        if self.turning:
            now = time.time()
            cmd = Twist()
            
            cmd.linear.x = -0.05
            cmd.angular.z = 0.5
            
            
            if hasattr(self, 'turning_right') and self.turning_right:
                cmd.linear.x = 0.05
                cmd.angular.z = -0.5  # right turn
          
            self.pub.publish(cmd)
 

            if now - self.turn_start_time >= 3.25:
                # Stop robot
              #  cmd = Twist()
                cmd.linear.x = 0.1
                cmd.angular.z = 0.0
                self.pub.publish(cmd)
                
                if hasattr(self, 'turning_right') and self.turning_right:
                    self.get_logger().info("Completed right turn. Resuming wall following.")
                    self.turning_right = False
                else:
                    self.get_logger().info("Completed left turn. Resuming wall following.")
                
                self.turning = False
              #  self.turn_start_time = None
          

            if now - self.turn_start_time >= 5.0:
                # Stop robot
                cmd = Twist()
                self.pub.publish(cmd)
                
              #  if hasattr(self, 'turning_right') and self.turning_right:
                #    self.get_logger().info("Completed right turn. Resuming wall following.")
                  #  self.turning_right = False
             #   else:
            #        self.get_logger().info("Completed left turn. Resuming wall following.")
                
               # self.turning = False
                self.turn_start_time = None
               
            return

        if self.last_scan is None:
            return



        
        side_dist, wall_theta = self.get_wall_distance_and_angle(self.last_scan)
        cmd = Twist()

        if side_dist is None:
            if self.wall_lost_time is None:
                self.wall_lost_time = time.time()
                return
               
            if time.time() - self.wall_lost_time >= self.wall_lost_delay:
                self.get_logger().warn("Lost right wall. Turning 90° right to reacquire.")
            
            
                self.turning = True
                self.turn_start_time = time.time()
                self.turning_right = True
                self.wall_lost_time = None
            return
            
        else:
            self.wall_lost_time = None

        K_dist = 0.6
        K_angle = 0.3 if self.side == 'right' else -0.3
        omega = -K_dist * (side_dist - self.d_star) - K_angle * wall_theta
        omega = max(-0.5, min(0.5, omega))

        front_ranges = np.array(self.last_scan.ranges)
        valid_front = front_ranges[np.isfinite(front_ranges)]
        front_min = np.min(valid_front) if len(valid_front) else 1.0
        
        front_angles_turn = np.concatenate((front_ranges[0:15], front_ranges[-15:]))
        valid_front_turn = front_angles_turn[np.isfinite(front_angles_turn)]
        front_min_turn = np.min(valid_front_turn) if len(valid_front_turn) else 1.0

        v = self.max_forward_speed
        if front_min < 0.5:
            v = max(self.min_forward_speed, self.max_forward_speed * front_min / 0.5)

        # ---- Trigger turn + shutdown event ----
        if front_min_turn < self.min_safe_distance:
  #      if front_min < 0.3:
            self.get_logger().warn("Wall detected ahead. Initiating 90° turn then shutdown.")
            self.turning = True
            self.turn_start_time = time.time()
            return

        cmd.linear.x = v
        cmd.angular.z = omega
        self.pub.publish(cmd)

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
