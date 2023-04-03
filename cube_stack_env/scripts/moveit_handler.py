import sys
import rospy
import moveit_commander
import geometry_msgs
import numpy as np

class MoveItHandler():
    def __init__(self, robot_description, z_offset):
        self.z_offset = z_offset
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander(robot_description=robot_description)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_arm = moveit_commander.MoveGroupCommander("arm", robot_description=robot_description)
        self.move_arm.set_max_velocity_scaling_factor(0.2)
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper", robot_description=robot_description)
        print('Arm Reference: ', self.move_arm.get_pose_reference_frame(), '--->', self.move_arm.get_end_effector_link())

        self.n_pillars = 10
        # ground plane
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = - self.z_offset - 0.005 
        box_name = "ground"
        self.scene.add_box(box_name, box_pose, size=(0.8,0.8,0.01))
        # back plane
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = -0.15
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = - self.z_offset + 0.2
        box_name = "back"
        self.scene.add_box(box_name, box_pose, size=(0.01,0.4,0.4))
    
    def __del__(self):
        rospy.loginfo('Shutting Down MoveIt Commander')
        moveit_commander.roscpp_shutdown()

    def move_to(self, goal_position):
        for i in range(self.n_pillars):
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = self.robot.get_planning_frame()
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.x = goal_position[0] + 0.06*np.sin(i*2*np.pi/self.n_pillars)
            box_pose.pose.position.y = goal_position[1] + 0.06*np.cos(i*2*np.pi/self.n_pillars)
            box_pose.pose.position.z = - self.z_offset + 0.025
            box_name = "pillar_obs_"+str(i)
            self.scene.add_box(box_name, box_pose, size=(0.01,0.01,0.05))

        self.move_arm.set_position_target(goal_position)
        self.move_arm.set_goal_tolerance(0.001)
        success = self.move_arm.go(wait=True)
        if not success:
            rospy.logwarn('Plan/Execution Failed')
        else:
            rospy.loginfo('Move Done')
        self.move_arm.stop()
        self.move_arm.clear_pose_targets()

        for i in range(self.n_pillars):
            self.scene.remove_world_object("pillar_obs_"+str(i))
        return success
    
    def gripper_close(self):
        self.move_gripper.set_named_target('gripper_close')
        self.move_gripper.set_goal_tolerance(0.001)
        success = self.move_gripper.go(wait=True)
        if not success:
            rospy.logwarn('Plan/Execution Failed')
        else:
            rospy.loginfo('Gripper Closed')
        self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()
        return success
    
    def gripper_open(self):
        self.move_gripper.set_named_target('gripper_open')
        self.move_gripper.set_goal_tolerance(0.0001)
        success = self.move_gripper.go(wait=True)
        if not success:
            rospy.logwarn('Plan/Execution Failed')
        else:
            rospy.loginfo('Gripper Opened')
        self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()
        return success
    
    def home(self):
        self.move_arm.set_named_target('home')
        self.move_arm.set_goal_tolerance(0.001)
        success = self.move_arm.go(wait=True)
        if not success:
            rospy.logwarn('Plan/Execution Failed')
        else:
            rospy.loginfo('Reached Home')
        self.move_arm.stop()
        self.move_arm.clear_pose_targets()
        return success

    def add_obstacles(self, obs_poses):
        for i in range(len(obs_poses)):
            obs_pose = geometry_msgs.msg.PoseStamped()
            obs_pose.header.frame_id = self.robot.get_planning_frame()
            obs_pose.pose = obs_poses[i]
            obs_pose.pose.position.z -= self.z_offset
            obs_name = "static_obs_"+str(i)
            # safe trajectories can be planned by just inflating the obstacle radius in planning scene
            self.scene.add_sphere(obs_name, obs_pose, radius=0.03)
    
    def remove_obstacles(self, num_obs):
        for i in range(num_obs):
            obs_name = "static_obs_"+str(i)
            self.scene.remove_world_object(obs_name)
