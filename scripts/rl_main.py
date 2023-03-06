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
        }
    env = CubeStackEnv(env_config)
    model = SAC("CnnPolicy", 
                env, 
                buffer_size=10000,
                batch_size=32,
                train_freq=(4, 'step'),
                target_update_interval=5000,
                verbose=1)
    model.learn(total_timesteps=100000, log_interval=10)
    model.save("sac_cube_stack")

    for e in range(5):
        # obs, _ = env.reset()
        obs = env.reset()
        done = False
        truncated = False
        episode_reward = 0
        start = time.perf_counter()
        while not (done or truncated):     
            action, _states = model.predict(obs, deterministic=True)
            # obs, reward, done, truncated, _ = env.step(action)
            obs, reward, done, _ = env.step(action)
            episode_reward += reward
            render_obs(obs)

        duration = time.perf_counter() - start
        rospy.loginfo('Episode ' + str(e+1) + ' Reward: ' + str(episode_reward) + ' Duration: ' + str(duration))

main()