import rospy
from cube_stack_env import CubeStackEnv
import cv2
import numpy as np
import time

def render_obs(obs):
    rgb = np.moveaxis(obs[:3,:,:], 0, -1)
    depth = obs[3,:,:]
    cv2.imshow('rgb', rgb)
    cv2.imshow('depth', depth)
    cv2.waitKey(10)

def main():
    rospy.init_node('cube_stack_test')
    env = CubeStackEnv(max_iter=100)
    for e in range(5):
        obs, _ = env.reset()
        done = False
        episode_reward = 0
        start = time.perf_counter()
        while not done:        
            action = np.random.uniform(-1, 1, (5,))*10
            obs, reward, done, _ = env.step(action)
            episode_reward += reward
            render_obs(obs)

        duration = time.perf_counter() - start
        rospy.loginfo('Episode ' + str(e+1) + ' Reward: ' + str(episode_reward) + ' Duration: ' + str(duration))

main()
