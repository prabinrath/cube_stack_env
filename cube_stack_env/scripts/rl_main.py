import rospy
from stable_baselines3 import SAC
from cube_stack_env import CubeStackEnv
import cv2
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
            'obstacle': False
        }
    env = CubeStackEnv(env_config)

    # model = SAC("MultiInputPolicy", 
    #             env, 
    #             buffer_size=10000,
    #             batch_size=32,
    #             verbose=1)
    
    # # print(model.policy)
    # model.learn(total_timesteps=500000, log_interval=10)
    # model.save("sac_cube_stack")

    model = SAC.load('sac_cube_stack')

    for e in range(15):
        # obs, _ = env.reset()
        obs = env.reset()
        done = False
        truncated = False
        episode_reward = 0
        start = time.perf_counter()
        while not (done or truncated):     
            action, _ = model.predict(obs, deterministic=True)
            # obs, reward, done, truncated, _ = env.step(action)
            obs, reward, done, _ = env.step(action)
            episode_reward += reward
            render_obs(obs["visual"])

        duration = time.perf_counter() - start
        rospy.loginfo('Episode ' + str(e+1) + ' Reward: ' + str(episode_reward) + ' Duration: ' + str(duration))

main()