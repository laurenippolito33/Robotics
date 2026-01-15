
##reads from camera node that detects aruco tag location, determines distance between tags,
#publishes to turtlebot to activate end movement when tags are close

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
from std_msgs.msg import Bool # Import the boolean message type

class DoubleArucoDetect(Node):
    def __init__(self):
        super().__init__('double_aruco_node')

        self.method1called=False
        self.method2called=False
        self.floor_pos_x = None
        self.floor_pos_y = None
        self.bot_pos_x = None
        self.bot_pos_y = None
        self.status = False
        self.tag_distance=5

        #create publisher to tell turtlebot it arrived at drop location
        self.pub_destination = self.create_publisher(Bool,'destination',10)
        ##no data 
        print("no data recieved, pub false")
        msg = Bool()
        msg.data = False
        self.pub_destination.publish(msg)

        ##subscribe to aruco detection nodes
        self.sub_posefloor = self.create_subscription(PoseStamped, '/aruco_pose_floor', self.floor_pose_callback, 10)
        self.sub_pose2bot = self.create_subscription(PoseStamped, '/aruco_pose_bot', self.bot_pose_callback, 10)
        
        ##create timer function
        self.timer = self.create_timer(0.05, self.on_timer)


    def floor_pose_callback (self, msg):
        self.floor_pos_x= msg.pose.position.x
        self.floor_pos_y= msg.pose.position.y

        #self.compare_locations()


    def bot_pose_callback (self, msg):
        self.bot_pos_x= msg.pose.position.x
        self.bot_pos_y= msg.pose.position.y

        #self.compare_locations()

    #def compare_locations(self):
    def on_timer(self):

        # ensure data exists
        if None in (self.floor_pos_x, self.floor_pos_y, self.bot_pos_x, self.bot_pos_y):
            ##no data 
            print("no data recieved, pub false")
            msg = Bool()
            msg.data = False
            self.pub_destination.publish(msg)
            return 

        #calculate distance between tags
        self.tag_distance= math.sqrt((self.bot_pos_x - self.floor_pos_x)**2 + (self.bot_pos_y - self.floor_pos_y)**2)
        #self.tag_distance=self.bot_pos_x - self.floor_pos_x
        #publish true if close or false if far
        msg = Bool()
        if self.tag_distance < 0.5:
            print('robot is near destination, publishing True')
            print('')
            msg.data = True
        else:
            print('tag is not near destination, publishing False')
            msg.data = False

        self.pub_destination.publish(msg)
        

# ---------------------------------------------
# Main
# ---------------------------------------------
def main():
    rclpy.init()
    node = DoubleArucoDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
