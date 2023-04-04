import rospy
from cube_stack_env import CubeStackEnv
from stable_baselines3.common.env_checker import check_env
import numpy as np
import time

def main():
    env_config={
            'dist_threshold': 0.05,
            'max_iter': 100,
            'obstacle': False
        }
    env = CubeStackEnv(env_config)
    # check_env(env)

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
            print(obs)

        duration = time.perf_counter() - start
        rospy.loginfo('Episode ' + str(e+1) + ' Reward: ' + str(episode_reward) + ' Duration: ' + str(duration))

main()
