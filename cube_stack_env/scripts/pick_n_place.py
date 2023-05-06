from moveit_handler_hardware import MoveItHandler
import rospy

BASE_Z_OFFSET = 0.055
DUMMY_AXIS_OFFSET = 0.01

def main():
    rospy.init_node("pick_n_place")
    robot_mp = MoveItHandler(robot_description='/cube_stack_arm/robot_description', z_offset=BASE_Z_OFFSET)

    # for i in range(5):
    robot_mp.home()
    robot_mp.gripper_open()
    goal_pos = [0.15, -0.1, -0.04+DUMMY_AXIS_OFFSET]
    robot_mp.move_to([goal_pos[0], goal_pos[1], goal_pos[2]])
    robot_mp.gripper_close()
    goal_pos = [0.1, -0.0, 0.025+DUMMY_AXIS_OFFSET]
    robot_mp.move_to([goal_pos[0], goal_pos[1], goal_pos[2]])
    robot_mp.gripper_open()
    robot_mp.home()
    # rospy.loginfo(f"Task {i+1} completed")
    
main()