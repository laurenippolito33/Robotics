import sys
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import math



def main():

    # Get coords from user (mm)
    if len(sys.argv) > 1:
        pick_location_x = int(sys.argv[1])
        pick_location_y = int(sys.argv[2])
        place_location_x = int(sys.argv[3])
        place_location_y = int(sys.argv[4])
    else:
        print('Please provide Pick and Place coordinates')
		
    # Arm Size Numbers
    l1 = 106 #mm
    l2 = 100 #mm
    shoulder_angle = math.atan((35/100))
    
    # Calculation rotation angle of waist
    waist_angle_pick = math.atan((pick_location_y / pick_location_x))
    waist_angle_place = math.atan((place_location_y / place_location_x))
    
    # Calculate new x value once rotated
    x_rot_pick = math.sqrt(((pick_location_x ** 2) + (pick_location_y ** 2)))
    x_rot_place = math.sqrt(((place_location_x ** 2) + (place_location_y ** 2)))
    
    # Find the x and z positions of the wrist joint
    z = 0 
    x_wrist_pick = x_rot_pick-90 
    x_wrist_place = x_rot_place-90 
    
    # Calculate Elbow Angle
    elbow_angle_pick = math.acos((((x_wrist_pick ** 2) + (z ** 2)) - (l2 ** 2) - (l1 ** 2)) / (2 * l1 * l2))* -1
    elbow_angle_place = math.acos((((x_wrist_place ** 2) + (z ** 2)) - (l2 ** 2) - (l1 ** 2)) / (2 * l1 * l2))* -1
    
    # Calculate Shoulder Angle
    var1=math.atan2(z,x_wrist_pick)
    var2=math.atan2((l2 * math.sin(elbow_angle_pick)), (l1 + l2 * math.cos(elbow_angle_pick)))
    shoulder_angle_pick = (var1 - var2) #SAME EQN JUST SPLIT INTO VARIABLES TO SEE IT BETTER
    var3=math.atan2(z,x_wrist_place)
    var4=math.atan2((l2 * math.sin(elbow_angle_place)), (l1 + l2 * math.cos(elbow_angle_place)))
    shoulder_angle_place = (var3 - var4) #SAME EQN JUST SPLIT INTO VARIABLES TO SEE IT BETTER

    # Calculate Wrist Angle
    wrist_angle_pick = (0 - elbow_angle_pick - shoulder_angle_pick)
    wrist_angle_place = (0 - elbow_angle_place - shoulder_angle_place)

    
    ##########################################################################################################
    
    # Convert Angles to Proper Directions
    pick_shoulder = (math.pi / 2) - shoulder_angle_pick - shoulder_angle
    place_shoulder = (math.pi / 2) - shoulder_angle_place - shoulder_angle
    
    pick_elbow = ((math.pi / 2) - shoulder_angle + elbow_angle_pick) * -1
    place_elbow = ((math.pi / 2) - shoulder_angle + elbow_angle_place) * -1
    
    pick_wrist = (wrist_angle_pick) * -1
    place_wrist = (wrist_angle_place) * -1
    
    # Make input arrays
    pick_joint_positions = [waist_angle_pick, pick_shoulder, pick_elbow, pick_wrist]
    place_joint_positions = [waist_angle_place, place_shoulder, place_elbow, place_wrist]
    int_joint_positions = [waist_angle_pick, -1, 1.5, 0]
    int_joint_positions2 = [waist_angle_place, -1, 1.5, 0]
		
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
)
    robot_startup()
    
    print(waist_angle_pick, pick_shoulder, pick_elbow, pick_wrist)
    print(waist_angle_place, place_shoulder, place_elbow, place_wrist)
    #bot.arm.go_to_home_pose()
    bot.arm.set_joint_positions(int_joint_positions)
    bot.arm.set_joint_positions(pick_joint_positions)
    bot.gripper.grasp(2.0)
    #bot.gripper.release(2.0) #### dont release yet!!!!!
    bot.arm.go_to_home_pose()
    bot.arm.set_joint_positions(place_joint_positions)
    bot.gripper.release(2.0)
    bot.arm.set_joint_positions(int_joint_positions2)
    #bot.gripper.grasp(2.0) ### already holding onto object
    #bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

    robot_shutdown()

if __name__ == '__main__':
    main()
