from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv
from imitation.data import rollout
from imitation.data.wrappers import RolloutInfoWrapper
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
                'variable_horizon': True
            }
    env = CubeStackEnv(dummy_env_config)

    expert = SAC.load(rospkg.RosPack().get_path('cube_stack_env')+"/checkpoints/sac_cube_stack", env=env)
    rollouts = rollout.rollout(
        expert,
        DummyVecEnv([lambda: RolloutInfoWrapper(env)]),
        rollout.make_sample_until(min_timesteps=None, min_episodes=50),
        rng=rng,
        )
    transitions = rollout.flatten_trajectories(rollouts)

    bc_trainer = BC(
        observation_space=env.observation_space,
        action_space=env.action_space,
        demonstrations=transitions,
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