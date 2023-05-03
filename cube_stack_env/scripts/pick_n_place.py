from moveit_handler import MoveItHandler
import rospy
import time

BASE_Z_OFFSET = 0.0

def main():
    robot_mp = MoveItHandler(robot_description='/cube_stack_arm/robot_description', z_offset=BASE_Z_OFFSET, scaling=0.1)

    robot_mp.home()    
    goal_pos = [0.2, 0.0, 0.0]
    robot_mp.gripper_open()
    robot_mp.move_to([goal_pos[0], goal_pos[1], goal_pos[2]-BASE_Z_OFFSET+0.01])
    robot_mp.gripper_close()
    robot_mp.home()

main()