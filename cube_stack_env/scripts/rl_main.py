from stable_baselines3.common.monitor import Monitor
from stable_baselines3 import SAC
from stable_baselines3.common.evaluation import evaluate_policy
from cube_stack_env import CubeStackEnv

def main():
    env_config={
            'dist_threshold': 0.05,
            'max_iter': 300,
            'obstacle': True,
            'num_obstacles': 3 # fixed
        }
    env = CubeStackEnv(env_config)
    
    # Train
    # model = SAC("MlpPolicy", 
    #             env, 
    #             buffer_size=100000,
    #             batch_size=32,
    #             verbose=1)    
    # # print(model.policy)
    # model.learn(total_timesteps=250000, log_interval=10)
    # model.save("checkpoints/sac_cube_stack")
    # model.save_replay_buffer("checkpoints/sac_cube_stack_replay_buffer")

    # Load
    model = SAC.load("checkpoints/sac_cube_stack", env=env)

    # Resume Training
    # model.load_replay_buffer("checkpoints/sac_cube_stack_replay_buffer")
    # model.learn(total_timesteps=250000, log_interval=10)
    # model.save("sac_cube_stack_new")

    env = Monitor(env, 'sac')
    reward, _ = evaluate_policy(model.policy, env, 10)
    print("Average Reward:", reward)

main()