import numpy as np
import rospkg
from stable_baselines3 import SAC
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env import DummyVecEnv

from imitation.algorithms.adversarial.airl import AIRL
from imitation.data.types import load
from imitation.rewards.reward_nets import BasicShapedRewardNet
from imitation.util.networks import RunningNorm

from ..cube_stack_env import CubeStackEnv

def main():
    rng = np.random.default_rng(0)
    dummy_env_config={
                'dist_threshold': 0.05,
                'max_iter': 300,
                'obstacle': True,
                'num_obstacles': 3, # fixed
                'variable_horizon': False
            }
    venv = DummyVecEnv([lambda: CubeStackEnv(dummy_env_config)])

    learner = SAC("MlpPolicy", venv) 
    reward_net = BasicShapedRewardNet(
        venv.observation_space,
        venv.action_space,
        normalize_input_layer=RunningNorm,
    )

    trajectories = load(rospkg.RosPack().get_path('cube_stack_env')+'/demonstrations/sample.traj')
    airl_trainer = AIRL(
        demonstrations=trajectories,
        demo_batch_size=1024,
        gen_replay_buffer_capacity=2048,
        n_disc_updates_per_round=4,
        venv=venv,
        gen_algo=learner,
        reward_net=reward_net,
    )

    reward, _ = evaluate_policy(learner, venv, 10)
    print("Avg reward before training:", reward)
    airl_trainer.train(10000)
    reward, _ = evaluate_policy(learner, venv, 10)
    print("Avg reward after training:", reward)

main()