from cube_stack_env import CubeStackEnv
from moveit_handler import MoveItHandler
import rospy
import time

BASE_Z_OFFSET = 0.05

def main():
    dummy_env_config={
            'dist_threshold': 0.05,
            'max_iter': 100,
            'obstacle': True,
            'num_obstacles': 3
        }
    env = CubeStackEnv(dummy_env_config)
    robot_mp = MoveItHandler(robot_description='/cube_stack_arm/robot_description', z_offset=BASE_Z_OFFSET)

    robot_mp.home()
    for itr in range(15):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            env.pause()
        except:
            raise Exception('Pause Physics Failed')
        env.reset_cubes() # also resets obstacles if present
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            env.unpause()
        except:
            raise Exception('Unpause Physics Failed')
        
        start = time.perf_counter()
        if dummy_env_config['obstacle']:
            rospy.wait_for_service('/gazebo/get_link_state')
            obstacles = [env.getlink_proxy('obs_'+str(i+1)+'::obstacle_sphere', 'world').link_state.pose for i in range(dummy_env_config['num_obstacles'])]
            robot_mp.add_obstacles(obstacles)

        # expert reacher
        rospy.wait_for_service('/gazebo/get_link_state')
        cube_state = env.getlink_proxy('cube_pick::wood_cube_2_5cm_red::link', 'world').link_state.pose.position # location of red cube
        robot_mp.gripper_open()
        robot_mp.move_to([cube_state.x, cube_state.y, cube_state.z-BASE_Z_OFFSET+0.01])

        # expert stacker
        robot_mp.gripper_close()
        rospy.wait_for_service('/gazebo/get_link_state')
        cube_state = env.getlink_proxy('cube_base::wood_cube_2_5cm_blue::link', 'world').link_state.pose.position # location of red cube
        robot_mp.move_to([cube_state.x, cube_state.y, cube_state.z-BASE_Z_OFFSET+0.075]) # fine tuning maybe required 
        robot_mp.gripper_open()

        robot_mp.home()
        if dummy_env_config['obstacle']:
            robot_mp.remove_obstacles(dummy_env_config['num_obstacles'])
        duration = time.perf_counter() - start
        rospy.loginfo('Task {} completed in {} secs'.format(itr+1, duration))

main()