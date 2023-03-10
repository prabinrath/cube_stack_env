import rospy
from cube_stack_env import CubeStackEnv
import cv2
import numpy as np
import time

def render_obs(obs):
    rgb = obs[:,:,:3]
    depth = obs[:,:,3]
    cv2.imshow('rgb', rgb)
    cv2.imshow('depth', depth)
    cv2.waitKey(10)

def main():
    env_config={
            'dist_threshold': 0.05,
            'max_iter': 100,
        }
    env = CubeStackEnv(env_config)
    for e in range(5):
        # obs, _ = env.reset()
        obs = env.reset()
        done = False
        truncated = False
        episode_reward = 0
        start = time.perf_counter()
        while not (done or truncated):     
            action = env.action_space.sample()
            # obs, reward, done, truncated, _ = env.step(action)
            obs, reward, done, _ = env.step(action)
            episode_reward += reward
            render_obs(obs)

        duration = time.perf_counter() - start
        rospy.loginfo('Episode ' + str(e+1) + ' Reward: ' + str(episode_reward) + ' Duration: ' + str(duration))

main()
