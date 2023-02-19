import rospy
from cube_stack_env import CubeStackEnv
import cv2
import numpy as np

def main():
    rospy.init_node('cube_stack_test')
    env = CubeStackEnv()
    
    while True:
        env.reset()
        obs = env.get_obs()
        rgb = np.moveaxis(obs[:3,:,:], 0, -1)
        depth = obs[3,:,:]
        cv2.imshow('rgb', rgb)
        cv2.imshow('depth', depth)
        cv2.waitKey(100)

        # env.step(np.array([0.5, 0.5, 0.5, 0.5, 0.0]))

        rospy.sleep(2)

main()
