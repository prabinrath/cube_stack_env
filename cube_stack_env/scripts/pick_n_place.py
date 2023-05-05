from moveit_handler_hardware import MoveItHandler
import rospy

BASE_Z_OFFSET = 0.055
DUMMY_AXIS_OFFSET = 0.01

def main():
    rospy.init_node("pick_n_place")
    robot_mp = MoveItHandler(robot_description='/cube_stack_arm/robot_description', z_offset=BASE_Z_OFFSET, scaling=2.0)

    for i in range(2):
        robot_mp.home()    
        goal_pos = [0.2, 0.1, 0.0+DUMMY_AXIS_OFFSET]
        robot_mp.gripper_open()
        robot_mp.move_to([goal_pos[0], goal_pos[1], goal_pos[2]])
        robot_mp.gripper_close()
        # robot_mp.home()
        rospy.loginfo(f"Task {i+1} completed")
    
main()