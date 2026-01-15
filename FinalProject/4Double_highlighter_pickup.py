#### puts arm control into a node
###reads in camera coordinates
#converts to coordinates relative to arm position
###picks up from starting position given from camera


import sys
from std_msgs.msg import Bool
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import math
###
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped##change based on message type beign sent


##create class for node 
class PickAndPlace(Node):
    def __init__(self):
        super().__init__('PickAndPlace')

        
        self.pickup_coordinates= None
        self.pick_location_x = None
        self.pick_location_y = None
        self.pick_location_x2 = None
        self.pick_location_y2 = None
        self.place_location_x = None
        self.place_location_y = None
        self.place_location_x2 = None
        self.place_location_y2 = None

        self.method1called=False
        self.method2called=False

        ##create publisher
        self.pub = self.create_publisher(Bool, 'ready', 10)

        ##subscribe to recieve pick up coordinates from camera
        ##may need to change names
        self.subscription = self.create_subscription(
            PointStamped,###message type//change probably
            '/blob_board_yellow', ###topic name
            self.listener_callback, ##callback function
            10 ###queue size
        )
        self.subscription = self.create_subscription(
            PointStamped,###message type//change probably
            '/blob_board_pink', ###topic name
            self.listener_callback2, ##callback function
            10 ###queue size
        )
        self.get_logger().info('pick and place node started, waiting for coordinates')

        #publishes false initially
        msg = Bool ()
        msg.data= False
        self.pub.publish(msg)
    
    def listener_callback(self,msg):
        #if blob_board is not None and t0==0.0:
            ##get pick locations from message from camera once
        og_location_x = msg.point.x
        og_location_y = msg.point.y
        t0=1
        ######################################################################
        ###convert x_original and y_original to actual pick
        ##y from camera is -y actual
        
        self.pick_location_x=(og_location_x)*1000
        self.pick_location_y= (og_location_y)*-1000	
   
   # Right edge    middle
        if ((abs(self.pick_location_y))/2) < (self.pick_location_x) < abs(self.pick_location_y) and self.pick_location_y < 0 :
        	self.pick_location_y = self.pick_location_y + 5
        	self.pick_location_x = self.pick_location_x + 60
        	print('right edge middle')
       # Right edge    extreme
        elif (abs(self.pick_location_y)/2) > (self.pick_location_x/2) and self.pick_location_y < 0:
        	self.pick_location_y = self.pick_location_y + 10
        	self.pick_location_x = self.pick_location_x + 80
        	print('right edge extreme')
        	
        	  # Right Center    
        
        elif abs(self.pick_location_y) < self.pick_location_x and self.pick_location_y < 0:
        	self.pick_location_y = self.pick_location_y - 5
        	self.pick_location_x = self.pick_location_x + 60
        	print('right center')
        
    
    # Left Edge
        
        elif abs(self.pick_location_y) > self.pick_location_x and self.pick_location_y > 0:
        	self.pick_location_y = self.pick_location_y - 60
        	self.pick_location_x = self.pick_location_x + 65
        	print('left edge')
     # Left Center
        
        elif abs(self.pick_location_y) < self.pick_location_x and self.pick_location_y > 0:
        	self.pick_location_y = self.pick_location_y - 30
        	self.pick_location_x = self.pick_location_x + 47
        	print('left center')
     

        ########################################################################################
        ##set coordinates for placing
        self.place_location_x = 1 ###change to actual location (mm)
        self.place_location_y = 200 ##change to actual location (mm)
        
        self.method1called=True
        if self.method2called:
            self.calculation_move()
    ###end listener1 callback

    def listener_callback2(self,msg):
        #if blob_board is not None and t0==0.0:
            ##get pick locations from message from camera once
        og_location_x2 = msg.point.x
        og_location_y2 = msg.point.y
        t0=1
        ######################################################################
        ###convert x_original and y_original to actual pick
        ##y from camera is -y actual
        
        self.pick_location_x2=(og_location_x2)*1000
        self.pick_location_y2= (og_location_y2)*-1000	
   
   # Right edge    middle
        if ((abs(self.pick_location_y2))/2) < (self.pick_location_x2) < abs(self.pick_location_y2) and self.pick_location_y2 < 0 :
        	self.pick_location_y2 = self.pick_location_y2 + 5
        	self.pick_location_x2 = self.pick_location_x2 + 60
        	print('right edge middle')
       # Right edge    extreme
        elif (abs(self.pick_location_y2)/2) > (self.pick_location_x2/2) and self.pick_location_y2 < 0:
        	self.pick_location_y2 = self.pick_location_y2 + 10
        	self.pick_location_x2 = self.pick_location_x2 + 80
        	print('right edge extreme')
        	
        	  # Right Center    
        
        elif abs(self.pick_location_y2) < self.pick_location_x2 and self.pick_location_y2 < 0:
        	self.pick_location_y2 = self.pick_location_y2 - 5
        	self.pick_location_x2 = self.pick_location_x2 + 60
        	print('right center')
        
    
    # Left Edge
        
        elif abs(self.pick_location_y2) > self.pick_location_x2 and self.pick_location_y2 > 0:
        	self.pick_location_y2 = self.pick_location_y2 - 60
        	self.pick_location_x2 = self.pick_location_x2 + 65
        	print('left edge')
     # Left Center
        
        elif abs(self.pick_location_y2) < self.pick_location_x2 and self.pick_location_y2 > 0:
        	self.pick_location_y2 = self.pick_location_y2 - 30
        	self.pick_location_x2 = self.pick_location_x2 + 47
        	print('left center')
     

        ########################################################################################
        ##set coordinates for placing
        self.place_location_x2 = 1 ###change to actual location (mm)
        self.place_location_y2 = 200 ##change to actual location (mm)
        
        self.method2called=True
        if self.method1called:
            self.calculation_move()
    ###end listener callback


    def calculation_move(self):
            
        # Arm Size Numbers
        l1 = 106 #mm
        l2 = 100 #mm
        shoulder_angle = math.atan((35/100))
        
        ##possible intermediate drop spot
        if self.pick_location_y2 >0:
            mid_pick_location_y2= self.pick_location_y2-100
            mid_pick_location_x2=self.pick_location_x2
        else:
            mid_pick_location_y2= self.pick_location_y2+100
            mid_pick_location_x2=self.pick_location_x2

        # Calculation rotation angle of waist
        #1
        waist_angle_pick = math.atan2(self.pick_location_y , self.pick_location_x)#1
        waist_angle_place = math.atan2(self.place_location_y , self.place_location_x)#1
        #2
        waist_angle_pick2 = math.atan2(self.pick_location_y2 , self.pick_location_x2)#2
        waist_angle_place2 = math.atan2(self.place_location_y2 , self.place_location_x2)#2
        ##middle 
        mid_waist_angle_pick2 = math.atan2(mid_pick_location_y2 , mid_pick_location_x2)

        
        # Calculate new x value once rotated
        #1
        x_rot_pick = math.sqrt(((self.pick_location_x ** 2) + (self.pick_location_y ** 2)))
        x_rot_place = math.sqrt(((self.place_location_x ** 2) + (self.place_location_y ** 2)))
        #2
        x_rot_pick2 = math.sqrt(((self.pick_location_x2 ** 2) + (self.pick_location_y2 ** 2)))
        x_rot_place2 = math.sqrt(((self.place_location_x2 ** 2) + (self.place_location_y2 ** 2)))
        #middle
        mid_x_rot_pick2 = math.sqrt(((mid_pick_location_x2 ** 2) + (mid_pick_location_y2 ** 2))) + 50
        
        if x_rot_pick < 180:
            x_rot_pick = x_rot_pick + 12
            print('close1')

        if x_rot_pick2 < 180:
            x_rot_pick2 = x_rot_pick2 + 12
            print('close2')
        
        # Find the x and z positions of the wrist joint
        z_pick = 10
        z_place=100
        #1
        x_wrist_pick = x_rot_pick-90 
        x_wrist_place = x_rot_place-90 
        #2
        x_wrist_pick2 = x_rot_pick2-90 
        x_wrist_place2 = x_rot_place2-90 
        #middle
        mid_x_wrist_pick2 = mid_x_rot_pick2-90 

        # Calculate Elbow Angle
        #1
        elbow_angle_pick = math.acos((((x_wrist_pick ** 2) + (z_pick ** 2)) - (l2 ** 2) - (l1 ** 2)) / (2 * l1 * l2))* -1
        elbow_angle_place = math.acos((((x_wrist_place ** 2) + (z_place ** 2)) - (l2 ** 2) - (l1 ** 2)) / (2 * l1 * l2))* -1
        #2
        elbow_angle_pick2 = math.acos((((x_wrist_pick2 ** 2) + (z_pick ** 2)) - (l2 ** 2) - (l1 ** 2)) / (2 * l1 * l2))* -1
        elbow_angle_place2 = math.acos((((x_wrist_place2 ** 2) + (z_place ** 2)) - (l2 ** 2) - (l1 ** 2)) / (2 * l1 * l2))* -1
        #middle
        mid_elbow_angle_pick2 = math.acos((((mid_x_wrist_pick2 ** 2) + (z_pick ** 2)) - (l2 ** 2) - (l1 ** 2)) / (2 * l1 * l2))* -1

        # Calculate Shoulder Angle
        #1
        var1=math.atan2(z_pick,x_wrist_pick)
        var2=math.atan2((l2 * math.sin(elbow_angle_pick)), (l1 + l2 * math.cos(elbow_angle_pick)))
        shoulder_angle_pick = (var1 - var2) #SAME EQN JUST SPLIT INTO VARIABLES TO SEE IT BETTER
        var3=math.atan2(z_place,x_wrist_place)
        var4=math.atan2((l2 * math.sin(elbow_angle_place)), (l1 + l2 * math.cos(elbow_angle_place)))
        shoulder_angle_place = (var3 - var4) #SAME EQN JUST SPLIT INTO VARIABLES TO SEE IT BETTER
        #2
        var12=math.atan2(z_pick,x_wrist_pick2)
        var22=math.atan2((l2 * math.sin(elbow_angle_pick2)), (l1 + l2 * math.cos(elbow_angle_pick2)))
        shoulder_angle_pick2 = (var12 - var22) #SAME EQN JUST SPLIT INTO VARIABLES TO SEE IT BETTER
        var32=math.atan2(z_place,x_wrist_place2)
        var42=math.atan2((l2 * math.sin(elbow_angle_place2)), (l1 + l2 * math.cos(elbow_angle_place2)))
        shoulder_angle_place2 = (var32 - var42) #SAME EQN JUST SPLIT INTO VARIABLES TO SEE IT BETTER
        #middle
        mid_var12=math.atan2(z_pick,mid_x_wrist_pick2)
        mid_var22=math.atan2((l2 * math.sin(mid_elbow_angle_pick2)), (l1 + l2 * math.cos(mid_elbow_angle_pick2)))
        mid_shoulder_angle_pick2 = (mid_var12 - mid_var22) #SAME EQN JUST SPLIT INTO VARIABLES TO SEE IT BETTER



        # Calculate Wrist Angle
        #1
        wrist_angle_pick = (0 - elbow_angle_pick - shoulder_angle_pick)
        wrist_angle_place = (0 - elbow_angle_place - shoulder_angle_place)
        #2
        wrist_angle_pick2 = (0 - elbow_angle_pick2 - shoulder_angle_pick2)
        wrist_angle_place2 = (0 - elbow_angle_place2 - shoulder_angle_place2)
        #middle
        mid_wrist_angle_pick2 = (0 - mid_elbow_angle_pick2 - mid_shoulder_angle_pick2)

        ##########################################################################################################
        
        # Convert Angles to Proper Directions
        #1
        pick_shoulder = (math.pi / 2) - shoulder_angle_pick - shoulder_angle
        place_shoulder = (math.pi / 2) - shoulder_angle_place - shoulder_angle
        #2
        pick_shoulder2 = (math.pi / 2) - shoulder_angle_pick2 - shoulder_angle
        place_shoulder2 = (math.pi / 2) - shoulder_angle_place2 - shoulder_angle
        #middle
        mid_pick_shoulder2 = (math.pi / 2) - mid_shoulder_angle_pick2 - shoulder_angle
        
        #1
        pick_elbow = ((math.pi / 2) - shoulder_angle + elbow_angle_pick) * -1
        place_elbow = ((math.pi / 2) - shoulder_angle + elbow_angle_place) * -1
        #2
        pick_elbow2 = ((math.pi / 2) - shoulder_angle + elbow_angle_pick2) * -1
        place_elbow2 = ((math.pi / 2) - shoulder_angle + elbow_angle_place2) * -1
        #middle
        mid_pick_elbow2 = ((math.pi / 2) - shoulder_angle + mid_elbow_angle_pick2) * -1

        #1
        pick_wrist = (wrist_angle_pick) * -1
        place_wrist = (wrist_angle_place) * -1
        #2
        pick_wrist2 = (wrist_angle_pick2) * -1
        place_wrist2 = (wrist_angle_place2) * -1
        #middle
        mid_pick_wrist2 = (mid_wrist_angle_pick2) * -1

        # Make input arrays
        #1
        pick_joint_positions = [waist_angle_pick, pick_shoulder, pick_elbow, pick_wrist]
        place_joint_positions = [waist_angle_place, place_shoulder, place_elbow, place_wrist]
        yellow_pick_scrunch = [waist_angle_pick, -1, 1.5, 0]
        yellow_place_high = [waist_angle_place, -1, 0, 0]
        yellow_place_scrunch = [waist_angle_place, -1, 1.5, 0]
        #2
        pick_joint_positions2 = [waist_angle_pick2, pick_shoulder2, pick_elbow2, pick_wrist2]
        place_joint_positions2 = [waist_angle_place2, place_shoulder2, place_elbow2, place_wrist2]
        pink_pick_scrunch = [waist_angle_pick2, -1, 1.5, 0]
        pink_place_scrunch = [waist_angle_place2, -1, 1.5, 0]
        pink_place_high = [waist_angle_place2, -1, 0, 0]
        int_joint_positions4 = [waist_angle_place2, -1, 1.5, 0]
        #middle
        mid_pick_joint_positions2 = [mid_waist_angle_pick2, mid_pick_shoulder2, mid_pick_elbow2, mid_pick_wrist2]
        pink_new_scrunch = [mid_waist_angle_pick2, -1, 1.5, -0.5]
        
    ##################################################################################################
        bot = InterbotixManipulatorXS(
            robot_model='px100',
            group_name='arm',
            gripper_name='gripper'
        )
        robot_startup()
        ##################################################################################################
        ###if marker 2 is infront of marker 1, move marker 2 aside so you can move marker 1 then move marker 2
        min_angle = math.pi / 12
        between_angle = abs(waist_angle_pick-waist_angle_pick2)
        if between_angle < min_angle and x_rot_pick2 < x_rot_pick:##turn angle is same and marker 2 is infront
            print('marker 2 blocks marker 1')
        ###move marker 2 to the side 
            bot.arm.set_joint_positions(pink_pick_scrunch)
            bot.arm.set_joint_positions(pick_joint_positions2)
            bot.gripper.grasp(0.5)
           
            bot.arm.set_joint_positions(pink_pick_scrunch)
            bot.arm.set_joint_positions(pink_new_scrunch)
            bot.arm.set_joint_positions(mid_pick_joint_positions2)
            bot.gripper.release(0.5)
            bot.arm.set_joint_positions(pink_new_scrunch)
            
            
        #move arm
        ##first marker
            bot.arm.set_joint_positions(yellow_pick_scrunch)
            bot.arm.set_joint_positions(pick_joint_positions)
            bot.gripper.grasp(0.8)
            bot.arm.set_joint_positions(yellow_pick_scrunch)
            bot.arm.set_joint_positions(yellow_place_scrunch)
            bot.arm.set_joint_positions(yellow_place_high)
            bot.arm.set_joint_positions(place_joint_positions)
            bot.gripper.release(0.8)
            bot.arm.set_joint_positions(yellow_place_high)
            bot.arm.set_joint_positions(yellow_place_scrunch)
           
        ##second marker
            bot.arm.set_joint_positions(pink_new_scrunch)
            bot.arm.set_joint_positions(mid_pick_joint_positions2)
            bot.gripper.grasp(0.8)
            bot.arm.set_joint_positions(yellow_place_scrunch)
            bot.arm.set_joint_positions(yellow_place_high)
            bot.arm.set_joint_positions(place_joint_positions)
            bot.gripper.release(0.8)
            #bot.arm.set_joint_positions(mid_int_joint_positions3)
            #bot.arm.go_to_home_pose()
           # bot.arm.set_joint_positions(int_joint_positions4)
           # bot.arm.set_joint_positions(place_joint_positions2)
            bot.gripper.release(0.8)
            bot.arm.set_joint_positions(yellow_place_high)
            bot.arm.set_joint_positions(yellow_place_scrunch)
           # bot.arm.set_joint_positions(int_joint_positions4)

            bot.arm.go_to_sleep_pose()
            
            msg = Bool ()
            msg.data= True
            self.pub.publish(msg)
            robot_shutdown()

        else:##if marker 2 isn't blocking 1
        #move arm
        ##first marker
            bot.arm.set_joint_positions(yellow_pick_scrunch)
            bot.arm.set_joint_positions(pick_joint_positions)
            bot.gripper.grasp(0.5)
            bot.arm.set_joint_positions(yellow_pick_scrunch)
            bot.arm.set_joint_positions(yellow_place_scrunch)
            bot.arm.set_joint_positions(yellow_place_high)
            bot.arm.set_joint_positions(place_joint_positions)
            bot.gripper.release(0.5)
            bot.arm.set_joint_positions(yellow_place_high)
            bot.arm.set_joint_positions(yellow_place_scrunch)
            
            
        ##second marker
            bot.arm.set_joint_positions(pink_pick_scrunch)
            bot.arm.set_joint_positions(pick_joint_positions2)
            bot.gripper.grasp(0.5)
            bot.arm.set_joint_positions(pink_pick_scrunch)
            bot.arm.set_joint_positions(pink_place_scrunch)
            bot.arm.set_joint_positions(pink_place_high)
            bot.arm.set_joint_positions(place_joint_positions2)
            bot.gripper.release(0.5)
            bot.arm.set_joint_positions(yellow_place_high)
            bot.arm.set_joint_positions(yellow_place_scrunch)
            
            msg = Bool ()
            msg.data= True
            self.pub.publish(msg)

            bot.arm.go_to_sleep_pose()

            robot_shutdown()

    ########################################################################################################
        #print angles
        print(waist_angle_pick, pick_shoulder, pick_elbow, pick_wrist)
        print(waist_angle_place, place_shoulder, place_elbow, place_wrist)
        print(waist_angle_pick2, pick_shoulder2, pick_elbow2, pick_wrist2)
        print(waist_angle_place2, place_shoulder2, place_elbow2, place_wrist2)


        #after action, publishes true
        
    #end calculation_move node    
##end node

##main
def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
