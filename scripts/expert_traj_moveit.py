from cube_stack_env import CubeStackEnv
from moveit_handler import MoveItHandler
import rospy
BASE_Z_OFFSET = 0.05

def main():
    dummy_env_config={
            'dist_threshold': 0.05,
            'max_iter': 100,
            'obstacle': False,
            'num_obstacles': 2
        }
    env = CubeStackEnv(dummy_env_config)
    robot_mp = MoveItHandler(robot_description='/cube_stack_arm/robot_description')

    # robot_mp.home()
    rospy.wait_for_service('/gazebo/get_link_state')
    cube_state = env.getlink_proxy('cube_pick::wood_cube_2_5cm_red::link', 'world').link_state.pose.position # location of red cube
    robot_mp.gripper_open()
    robot_mp.move_to([cube_state.x, cube_state.y, cube_state.z-BASE_Z_OFFSET])
    robot_mp.gripper_close()
    rospy.wait_for_service('/gazebo/get_link_state')
    cube_state = env.getlink_proxy('cube_base::wood_cube_2_5cm_blue::link', 'world').link_state.pose.position # location of red cube
    robot_mp.move_to([cube_state.x, cube_state.y, cube_state.z-BASE_Z_OFFSET+0.05])    
    robot_mp.gripper_open()
    robot_mp.home()

main()