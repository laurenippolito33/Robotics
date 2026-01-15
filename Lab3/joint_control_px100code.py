import sys

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
def main():
    # TODO: Define the joint angles in radians considering the joint limits
    joint1_angle=1.5
    joint2_angle=0.3
    joint3_angle=0.3
    joint4_angle=0.5
    joint_positions = [joint1_angle, joint2_angle, joint3_angle, joint4_angle]

    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )
    robot_startup()
    
    bot.arm.go_to_home_pose()
    bot.arm.set_joint_positions(joint_positions)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    robot_shutdown()

if __name__ == '__main__':
    main()
