from imitation.data.types import load
from imitation.algorithms.bc import BC 
from ..cube_stack_env import CubeStackEnv
import rospkg
import numpy as np
rng = np.random.default_rng(0)

def main():
    dummy_env_config={
                'dist_threshold': 0.05,
                'max_iter': 300,
                'obstacle': True,
                'num_obstacles': 3, # fixed
                'variable_horizon': False
            }
    env = CubeStackEnv(dummy_env_config)

    trajectories = load(rospkg.RosPack().get_path('cube_stack_env')+'/demonstrations/sample.traj')
    bc_trainer = BC(
        observation_space=env.observation_space,
        action_space=env.action_space,
        demonstrations=trajectories,
        rng=rng,
    )

    avg_reward = 0
    for e in range(5):
        # obs, _ = env.reset()
        obs = env.reset()
        done = False
        truncated = False
        episode_reward = 0
        while not (done or truncated):     
            action, _ = bc_trainer.policy.predict(obs, deterministic=False)
            # obs, reward, done, truncated, _ = env.step(action)
            obs, reward, done, _ = env.step(action)
            episode_reward += reward
        if e == 0:
            avg_reward = episode_reward
        else:
            avg_reward = (avg_reward+episode_reward)/2
    print('Avg reward before training: ', avg_reward)

    bc_trainer.train(n_epochs=1)

    avg_reward = 0
    for e in range(5):
        # obs, _ = env.reset()
        obs = env.reset()
        done = False
        truncated = False
        episode_reward = 0
        while not (done or truncated):     
            action, _ = bc_trainer.policy.predict(obs, deterministic=True)
            # obs, reward, done, truncated, _ = env.step(action)
            obs, reward, done, _ = env.step(action)
            episode_reward += reward
        if e == 0:
            avg_reward = episode_reward
        else:
            avg_reward = (avg_reward+episode_reward)/2
    print('Avg reward after training: ', avg_reward)

main()