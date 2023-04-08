from cube_stack_env import CubeStackEnv
from moveit_handler import MoveItHandler
import rospy
import time
from threading import Thread
from imitation.data.types import Trajectory, save
import rospkg
import copy

BASE_Z_OFFSET = 0.05
THREAD_SLEEP = 0.05
THREAD_CONTROL = False
N_EPISODES = 100

dummy_env_config={
            'dist_threshold': 0.05,
            'max_iter': 300,
            'obstacle': True,
            'num_obstacles': 3 # fixed
        }
env = CubeStackEnv(dummy_env_config)


observation_buffer = []
action_buffer = []

def data_collection():
    global THREAD_CONTROL
    print('starting data collection')
    while THREAD_CONTROL:
        observation_buffer.append(env.get_obs())
        action_buffer.append(env.action)
        time.sleep(THREAD_SLEEP)
    print('ending data collection')

def main():
    global THREAD_CONTROL
    robot_mp = MoveItHandler(robot_description='/cube_stack_arm/robot_description', z_offset=BASE_Z_OFFSET)

    trajectories = []
    home_pos = [0,]*5
    robot_mp.home(home_pos)
    for itr in range(N_EPISODES):
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
        
        observation_buffer.append(env.get_obs())
        THREAD_CONTROL = True
        Thread(target=data_collection).start()

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
        # robot_mp.gripper_close()
        # rospy.wait_for_service('/gazebo/get_link_state')
        # cube_state = env.getlink_proxy('cube_base::wood_cube_2_5cm_blue::link', 'world').link_state.pose.position # location of red cube
        # robot_mp.move_to([cube_state.x, cube_state.y, cube_state.z-BASE_Z_OFFSET+0.075]) # fine tuning maybe required 
        # robot_mp.gripper_open()        

        THREAD_CONTROL = False
        print('Trajectory size: obs - {}, acts - {}'.format(len(observation_buffer), len(action_buffer)))
        trajectories.append(Trajectory(obs=copy.deepcopy(observation_buffer), acts=copy.deepcopy(action_buffer),infos=None,terminal=True))
        observation_buffer.clear()
        action_buffer.clear()

        robot_mp.home(home_pos)
        if dummy_env_config['obstacle']:
            robot_mp.remove_obstacles(dummy_env_config['num_obstacles'])
        duration = time.perf_counter() - start

        rospy.loginfo('Task {} completed in {} secs'.format(itr+1, duration))
    
    save(rospkg.RosPack().get_path('cube_stack_env')+'/demonstrations/sample_.traj', trajectories)

main()