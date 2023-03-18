import gym
from gym import spaces

import numpy as np
import time
from threading import Lock
import cv2
from cv_bridge import CvBridge
import random

from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetLinkState, GetLinkState
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker
import rospy

class CubeStackEnv(gym.Env):
    metadata = {"render.modes": ["human"]}

    def __init__(self, env_config):
        super(CubeStackEnv, self).__init__()
        rospy.init_node('cube_stack_rl_'+str(random.randint(0,1e5)))

        self.action_space = spaces.Box(-1, 1, (5,), np.float32)
        self.observation_space = spaces.Box(0, 255, (224, 224, 4), np.uint8)
        self.rgb_img, self.depth_img = None, None
        self.bridge = CvBridge()
        self.observation = np.zeros((224, 224, 4), dtype=np.uint8)
        self.obs_lock = Lock()
        self.dist_threshold = env_config['dist_threshold']
        self.reward = None
        self.max_iter = env_config['max_iter']
        self.iter = 0
        self.obstacle = env_config['obstacle']
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
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty) # reset gazebo
        self.agent_pub = rospy.Publisher('/cube_stack_arm/arm_effort_controller/command', Float64MultiArray, queue_size=2) # effort control
        self.agent_rgb_sub = rospy.Subscriber('/depth_camera/rgb/image_raw', Image, self.rgb_callback) # rgb observation
        self.agent_depth_sub = rospy.Subscriber('/depth_camera/depth/image_raw', Image, self.depth_callback) # depth observation

        # self.grip_viz = rospy.Publisher('/ee', Marker, queue_size=2)
    
    def rgb_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img = cv2.resize(img, (224, 224))
        self.obs_lock.acquire()
        self.observation[:,:,:3] = img
        self.obs_lock.release()

    def depth_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        img = cv2.resize(img, (224, 224))
        self.obs_lock.acquire()
        self.observation[:,:,3] = (img*255.0).astype(np.uint8)
        self.obs_lock.release()
    
    def get_obs(self):
        self.obs_lock.acquire()
        obs = self.observation
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

        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except:
            raise Exception('Reset Simulation Failed')
        
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except:
            raise Exception('Pause Physics Failed')

        rospy.wait_for_service('/gazebo/set_link_state')
        cube_link_names = ['cube_base::wood_cube_2_5cm_blue::link', 'cube_pick::wood_cube_2_5cm_red::link']
        idx = random.choice([0,1])
        try:
            req = LinkState()
            req.link_name = cube_link_names[idx]
            pose = Pose()
            pose.position.x = random.uniform(0.15, 0.18)
            pose.position.y = random.uniform(0.05, 0.18)
            pose.position.z = 0.0125
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0, 0, 0, 1
            req.pose = pose
            req.reference_frame = 'world'
            self.setlink_proxy(req)
            req = LinkState()
            req.link_name = cube_link_names[1-idx]
            pose = Pose()
            pose.position.x = random.uniform(0.15, 0.18)
            pose.position.y = random.uniform(-0.05, -0.18)
            pose.position.z = 0.0125
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0, 0, 0, 1
            req.pose = pose
            req.reference_frame = 'world'
            self.setlink_proxy(req)
            if self.obstacle:
                req = LinkState()
                req.link_name = 'obstacle_sphere::obstacle_sphere'
                pose = Pose()
                pose.position.x = random.uniform(0.15, 0.18)
                pose.position.y = random.uniform(-0.18,0.18)
                pose.position.z = 0.2
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0, 0, 0, 1
                req.pose = pose
                req.reference_frame = 'world'
                self.setlink_proxy(req)
        except:
            raise Exception('Set Link State Failed')
            
        time.sleep(0.1) # wait for observation update
        obs = self.get_obs()
        self.iter = 0
        rospy.loginfo('Environment Reset Done')

        # return obs, {}
        return obs
    
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

    def get_reward(self, action):
        rospy.wait_for_service('/gazebo/get_link_state')
        gripper_state = self.getlink_proxy('cube_stack_arm::arm_wrist_flex_link', 'world')
        link_pos = np.expand_dims(np.array([gripper_state.link_state.pose.position.x, 
                            gripper_state.link_state.pose.position.y, 
                            gripper_state.link_state.pose.position.z]), axis=1)
        quat = gripper_state.link_state.pose.orientation
        rot_mat = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])[0:3,0:3]
        local_offset = np.array([[0.0],[0.0],[0.1]]) # distance to end-effector in arm_wrist_flex_link frame
        grip_pos = link_pos + rot_mat @ local_offset
        # self.grip_viz.publish(self.get_marker(grip_pos))

        rospy.wait_for_service('/gazebo/get_link_state')
        cube_state = self.getlink_proxy('cube_pick::wood_cube_2_5cm_red::link', 'world') # location of red cube
        dist = np.linalg.norm(np.squeeze(grip_pos) -
                       np.array([cube_state.link_state.pose.position.x, 
                                 cube_state.link_state.pose.position.y, 
                                 cube_state.link_state.pose.position.z]))
        reward = -dist - 1e-3*np.linalg.norm(action)
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
        done = dist < self.dist_threshold
        truncated = self.iter > self.max_iter
        # return obs, reward, done, truncated, {}
        return obs, reward, (done or truncated), {}
