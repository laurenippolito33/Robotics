
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time



class OpenLoop(Node):

    def __init__(self):
        super().__init__('open_loop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def forward(self, v=0.12,duration=2.0):
        msg = Twist(); msg.linear.x = float(v); msg.angular.z = 0.0
        t0=time.time()
        while rclpy.ok() and time.time()-t0<duration:
            self.pub.publish(msg)
            time.sleep(0.05)
        self.pub.publish(Twist()) #stop
    
    def turn_in_place(self,w=0.6,duration=1.5):
        msg=Twist(); msg.linear.x=0.0; msg.angular.z=float(w)
        t0=time.time()
        while rclpy.ok() and time.time()-t0<duration:
            self.pub.publish(msg)
            time.sleep(0.05)
        self.pub.publish(Twist())
def main():
    rclpy.init()
    node=OpenLoop()
    node.forward(0.12,2.0)
    node.turn_in_place(0.6,1.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
