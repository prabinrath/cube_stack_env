import gym
from gym import spaces

import numpy as np
from math import pi
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
import rospy

class CubeStackEnv(gym.Env):
    def __init__(self, distance_threshold=0.005):
        self.action_space = spaces.Box(-pi, pi, (6,), np.float32)
        self.observation_space = spaces.Box(0, 255, (4, 224, 224), np.float32)
        self.rgb_img, self.depth_img = None, None
        self.bridge = CvBridge()
        self.observation = np.zeros((4, 224, 224), dtype=np.float32)
        self.obs_lock = Lock()
        self.distance_threshold = distance_threshold
        self.reward = None
        # rospy.set_param('/use_sim_time', True)

        self.joint_lims = {
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
    
    def rgb_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        cv2.imwrite('sample.jpg', img)
        img = cv2.resize(img, (224, 224))
        self.obs_lock.acquire()
        self.observation[:3,:,:] = np.moveaxis(img.astype(np.float32)/255.0, -1, 0)
        self.obs_lock.release()

    def depth_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        img = cv2.resize(img, (224, 224))
        self.obs_lock.acquire()
        self.observation[3,:,:] = img
        self.obs_lock.release()
    
    def get_obs(self):
        self.obs_lock.acquire()
        obs = self.observation
        self.obs_lock.release()
        return obs

    def reset(self, seed=None):
        super().reset(seed=seed)
        self.agent_pub.publish(Float64MultiArray(data=np.array([0.0, 0.0, 0.0, 0.0, 0.0])))

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except:
            raise Exception('Pause Physics Failed')

        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except:
            raise Exception('Reset Simulation Failed')
        
        rospy.wait_for_service('/gazebo/set_link_state')
        try:
            req = LinkState()
            req.link_name = 'cube_base::wood_cube_2_5cm_blue::link'
            pose = Pose()
            pose.position.x = random.uniform(0.15, 0.18)
            pose.position.y = random.uniform(0.05, 0.18)
            pose.position.z = 0.0125
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0, 0, 0, 1
            req.pose = pose
            req.reference_frame = 'world'
            self.setlink_proxy(req)
            req = LinkState()
            req.link_name = 'cube_pick::wood_cube_2_5cm_red::link'
            pose = Pose()
            pose.position.x = random.uniform(0.15, 0.18)
            pose.position.y = random.uniform(-0.05, -0.18)
            pose.position.z = 0.0125
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = 0, 0, 0, 1
            req.pose = pose
            req.reference_frame = 'world'
            self.setlink_proxy(req)
        except:
            raise Exception('Set Link State Failed')
        
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except:
            raise Exception('Unpause Physics Failed')

        rospy.loginfo('Environment Reset Done')
    
    def step(self, action):
        act = Float64MultiArray()
        act.data = action
        self.agent_pub.publish(act)   
