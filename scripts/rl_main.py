import torch
from garage.envs import GymEnv
from garage import wrap_experiment
from garage.torch import set_gpu_mode
from garage.trainer import Trainer
from garage.torch.algos import PPO
from garage.sampler import RaySampler

from cube_stack_env import CubeStackEnv

@wrap_experiment
def cube_stack_experiment(ctxt=None, seed=None):
    env = GymEnv(CubeStackEnv, is_image=True, max_episode_length=1000)
    trainer = Trainer(ctxt)

    # if torch.cuda.is_available():
    #     set_gpu_mode(True)
    #     algo.to()
    
    # trainer.setup(algo, env)
    # trainer.train(n_epochs=100, batch_size=32)

cube_stack_experiment()
