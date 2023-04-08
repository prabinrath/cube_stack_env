import gym
from gym.spaces import Box

import numpy as np
import time
from threading import Lock
import random
import copy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetLinkState, SpawnModel, SetLinkState, SetLinkStateRequest, SetModelConfiguration, SetModelConfigurationRequest
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker
import rospy
import rospkg

class CubeStackEnv(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(self, env_config):
        super(CubeStackEnv, self).__init__()
        rospy.init_node('cube_stack_env_'+str(random.randint(0,1e5)))

        self.action_space = Box(-1, 1, (5,), np.float32)
        self.obstacle = env_config['obstacle']
        if self.obstacle:
            # 10 for joint states (pos and vel), 3 for grip pos, 3 for cube pick, 3 for cube base, 9 for three obstacles
            self.observation_space = Box(-np.pi, np.pi, (28,), np.float32)
            self.observation = np.zeros((28,), dtype=np.float32)
        else:
            # 10 for joint states (pos and vel), 3 for grip pos, 3 for cube pick, 3 for cube base
            self.observation_space = Box(-np.pi, np.pi, (19,), np.float32)
            self.observation = np.zeros((19,), dtype=np.float32)

        self.action = np.zeros((5,), dtype=np.float32) # for BC
        
        self.obs_lock = Lock()
        self.dist_threshold = env_config['dist_threshold']
        self.reward = None
        self.max_iter = env_config['max_iter']
        self.iter = 0        
        self.variable_horizon = env_config['variable_horizon']
        rospy.set_param('/use_sim_time', True)

        self.joint_pos_lims = {
            'pan': (-2.617, 2.617),
            'shoulder': (-2.617, 2.617),
            'elbow': (-2.617, 2.617),
            'wrist': (-1.745, 1.745),
            'gripper': (-0.40, 0.40)
        }

        self.setlink_proxy = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState) # setting cube locations after reset
        self.getlink_proxy = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState) # reward calculation
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty) # reset gazebo
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty) # reset gazebo
        self.reset_model = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration) # set model joints to init positions
        self.agent_pub = rospy.Publisher('/cube_stack_arm/arm_effort_controller/command', Float64MultiArray, queue_size=2) # effort control
        self.joint_state_sub = rospy.Subscriber('/cube_stack_arm/joint_states', JointState, self.joint_state_callback, queue_size=2) # robot joint position observation

        if self.obstacle:
            self.num_obstacles = 3 # fixed
            assert(self.num_obstacles<=3)
            self.spawn_model_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel) # spawn obstacles
            self.obstacle_model_sdf = open(rospkg.RosPack().get_path('cube_stack_env')+'/worlds/obstacle_sphere.sdf', 'r').read()
            rospy.wait_for_service('/gazebo/set_model_state')
            for i in range(self.num_obstacles):
                pose = Pose()
                pose.position.x = 0.15
                pose.position.y = 0.0
                pose.position.z = i*0.3/self.num_obstacles + 0.15
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0, 0, 0, 1
                self.spawn_model_proxy('obs_'+str(i+1), self.obstacle_model_sdf, "", pose, 'world')
        # self.grip_viz = rospy.Publisher('/ee', Marker, queue_size=2)
    
    def joint_state_callback(self, msg):
        position = np.array([msg.position[2], msg.position[1], msg.position[0], msg.position[3], msg.position[4]])
        velocity = np.array([msg.velocity[2], msg.velocity[1], msg.velocity[0], msg.velocity[3], msg.velocity[4]])
        self.action = np.array([msg.effort[2], msg.effort[1], msg.effort[0], msg.effort[3], msg.effort[4]])
        grip_pos = self.get_grip_pos('cube_stack_arm::arm_wrist_flex_link', np.array([[0.0],[0.0],[0.1]]))
        self.obs_lock.acquire()
        self.observation[:13] = np.concatenate((position, velocity, grip_pos), dtype=np.float32)
        self.obs_lock.release()
    
    def get_obs(self):
        self.obs_lock.acquire()
        obs = copy.deepcopy(self.observation)
        self.obs_lock.release()
        return obs

    def reset(self, seed=None, options=None):
        # super().reset(seed=seed)

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except:
            raise Exception('Unpause Physics Failed')
        
        # reset controller
        for _ in range(10):
            self.agent_pub.publish(Float64MultiArray(data=np.array([0.0, 0.0, 0.0, 0.0, 0.0])))
            time.sleep(0.05)
        
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except:
            raise Exception('Pause Physics Failed')
        
        # reset model
        rospy.wait_for_service('/gazebo/set_model_configuration')
        try:
            req = SetModelConfigurationRequest()
            req.model_name = 'cube_stack_arm'
            req.urdf_param_name = '/cube_stack_arm/robot_description'
            req.joint_names = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint', 'gripper_joint']
            req.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.reset_model(req)
        except:
            raise Exception('Model Reset Failed')

        # reset cubes
        self.reset_cubes()
            
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except:
            raise Exception('Unpause Physics Failed')
        
        time.sleep(0.1) # wait for observation
        obs = self.get_obs()
        self.iter = 0
        rospy.loginfo('Environment Reset Done')

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except:
            raise Exception('Pause Physics Failed')

        # return obs, {}
        return obs
    
    def reset_cubes(self):
        rospy.wait_for_service('/gazebo/set_link_state')
        cube_link_info = [('cube_base::wood_cube_2_5cm_blue::link', 0.025), ('cube_pick::wood_cube_2_5cm_red::link', 0.0125)]
        idx = random.choice([0,1])
        try:
            req = SetLinkStateRequest()
            req.link_state.link_name = cube_link_info[idx][0]
            pose = Pose()
            dist = random.uniform(0.13, 0.18)
            th = random.uniform(-np.pi/2, np.pi/2)
            cube1_pos = (dist*np.cos(th), dist*np.sin(th))
            pose.position.x = cube1_pos[0]
            pose.position.y = cube1_pos[1]
            pose.position.z = cube_link_info[idx][1]
            if 'red' in cube_link_info[idx][0]:
                self.observation[13:16] = np.array([pose.position.x, pose.position.y, pose.position.z])
            else:
                self.observation[16:19] = np.array([pose.position.x, pose.position.y, pose.position.z])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0, 0, 0, 1
            req.link_state.pose = pose
            req.link_state.reference_frame = 'world'
            self.setlink_proxy(req)
            req = SetLinkStateRequest()
            req.link_state.link_name = cube_link_info[1-idx][0]
            pose = Pose()
            dist = random.uniform(0.13, 0.18)
            th = random.uniform(-np.pi/2, np.pi/2)
            cube2_pos = (dist*np.cos(th), dist*np.sin(th))
            while abs(cube2_pos[0]-cube1_pos[0]) + abs(cube2_pos[1]-cube1_pos[1]) < 0.12:
                dist = random.uniform(0.13, 0.18)
                th = random.uniform(-np.pi/2, np.pi/2)
                cube2_pos = (dist*np.cos(th), dist*np.sin(th))
            pose.position.x = cube2_pos[0]
            pose.position.y = cube2_pos[1]
            pose.position.z = cube_link_info[1-idx][1]
            if 'red' in cube_link_info[1-idx][0]:
                self.observation[13:16] = np.array([pose.position.x, pose.position.y, pose.position.z])
            else:
                self.observation[16:19] = np.array([pose.position.x, pose.position.y, pose.position.z])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0, 0, 0, 1
            req.link_state.pose = pose
            req.link_state.reference_frame = 'world'
            self.setlink_proxy(req)
            if self.obstacle:
                for i in range(self.num_obstacles):
                    req = SetLinkStateRequest()
                    req.link_state.link_name = 'obs_'+str(i+1)+'::obstacle_sphere'
                    dist = random.uniform(0.13, 0.18)
                    th = random.uniform(-np.pi/2, np.pi/2)
                    obs_pos = (dist*np.cos(th), dist*np.sin(th))
                    pose.position.x = obs_pos[0]
                    pose.position.y = obs_pos[1]
                    pose.position.z = i*0.3/self.num_obstacles + 0.2
                    self.observation[19+i*3:19+(i+1)*3] = np.array([pose.position.x, pose.position.y, pose.position.z])
                    req.link_state.pose = pose
                    req.link_state.reference_frame = 'world'
                    self.setlink_proxy(req)
        except:
            raise Exception('Set Link State Failed')
    
    def get_marker(self, grip_pos):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.get_rostime()
        marker.ns = ""
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = grip_pos[0][0]
        marker.pose.position.y = grip_pos[1][0]
        marker.pose.position.z = grip_pos[2][0]
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x, marker.scale.y, marker.scale.z = 0.01, 0.01, 0.01 
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def get_grip_pos(self, ref_link, local_offset):
        rospy.wait_for_service('/gazebo/get_link_state')
        gripper_state = self.getlink_proxy(ref_link, 'world')
        link_pos = np.expand_dims(np.array([gripper_state.link_state.pose.position.x, 
                            gripper_state.link_state.pose.position.y, 
                            gripper_state.link_state.pose.position.z]), axis=1)
        quat = gripper_state.link_state.pose.orientation
        rot_mat = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])[0:3,0:3]
        # local_offset is distance to end-effector in ref_link frame
        grip_pos = link_pos + rot_mat @ local_offset
        return np.squeeze(grip_pos)

    def get_reward(self, action):
        grip_pos = self.get_grip_pos('cube_stack_arm::arm_wrist_flex_link', np.array([[0.0],[0.0],[0.1]]))
        # self.grip_viz.publish(self.get_marker(grip_pos))

        rospy.wait_for_service('/gazebo/get_link_state')
        cube_state = self.getlink_proxy('cube_pick::wood_cube_2_5cm_red::link', 'world') # location of red cube
        dist = np.linalg.norm(grip_pos -
                       np.array([cube_state.link_state.pose.position.x, 
                                 cube_state.link_state.pose.position.y, 
                                 cube_state.link_state.pose.position.z]))
        reward = -dist - 0.1*np.linalg.norm(action)
        return dist, reward

    def step(self, action):
        self.iter += 1
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except:
            raise Exception('Unpause Physics Failed')
        
        for _ in range(10):
            self.agent_pub.publish(Float64MultiArray(data=action))
            time.sleep(0.001)

        obs = self.get_obs()

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except:
            raise Exception('Pause Physics Failed')
        
        dist, reward = self.get_reward(action)
        if self.variable_horizon: # essential for imitation learning
            done = dist < self.dist_threshold
        else:
            done = False
        truncated = self.iter > self.max_iter
        # return obs, reward, done, truncated, {}
        return obs, reward, (done or truncated), {}
