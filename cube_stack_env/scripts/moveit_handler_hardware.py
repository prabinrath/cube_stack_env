import sys
import rospy
import moveit_commander
import geometry_msgs
import numpy as np
from scipy.interpolate import CubicHermiteSpline, CubicSpline
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class MoveItHandler():
    def __init__(self, robot_description, z_offset, scaling=1.0):
        self.z_offset = z_offset
        self.scaling = scaling
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander(robot_description=robot_description)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_arm = moveit_commander.MoveGroupCommander("arm", robot_description=robot_description)
        self.command_pub = rospy.Publisher('/cube_stack_arm/command', Float64MultiArray, queue_size=10)
        self.joint_state_sub = rospy.Subscriber('/cube_stack_arm/joint_states', JointState, self.joint_state_callback, queue_size=10)
        print('Arm Reference: ', self.move_arm.get_pose_reference_frame(), '--->', self.move_arm.get_end_effector_link())

        self.n_pillars = 10
        self.joint_states = np.zeros((5,), dtype=np.float64)
        # helps in preventing the jerk while switching moveit group
        self.arm_command_bkp = np.zeros((5,), dtype=np.float64)
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
        self.scene.add_box(box_name, box_pose, size=(0.01,0.8,0.8))
        # top plane
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = - self.z_offset + 0.3
        box_name = "top"
        self.scene.add_box(box_name, box_pose, size=(0.8,0.8,0.01))
        # left plane
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = -0.2
        box_pose.pose.position.z = - self.z_offset + 0.2
        box_name = "left"
        self.scene.add_box(box_name, box_pose, size=(0.8,0.01,0.8))
        # right plane
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.2
        box_pose.pose.position.z = - self.z_offset + 0.2
        box_name = "right"
        self.scene.add_box(box_name, box_pose, size=(0.8,0.01,0.8))
        # shoulder safety pillars
        for i in range(self.n_pillars):
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = self.robot.get_planning_frame()
            box_pose.pose.orientation.w = 1.0
            box_pose.pose.position.x = 0.06*np.sin(i*2*np.pi/self.n_pillars)
            box_pose.pose.position.y = 0.06*np.cos(i*2*np.pi/self.n_pillars)
            box_pose.pose.position.z = 0.03
            box_name = "pillar_shoulder_safety_"+str(i)
            self.scene.add_box(box_name, box_pose, size=(0.01,0.01,0.06))
    
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
            box_pose.pose.position.z = - self.z_offset + 0.04
            box_name = "pillar_obs_"+str(i)
            self.scene.add_box(box_name, box_pose, size=(0.01,0.01,0.08))

        self.move_arm.set_position_target(goal_position)
        self.move_arm.set_goal_tolerance(0.001)
        success, traj, _, _ = self.move_arm.plan()
        if not success:
            rospy.logwarn('Plan Failed')
        else:
            success = self.execute_trajectory(traj, "arm") and success
        self.move_arm.clear_pose_targets()

        for i in range(self.n_pillars):
            self.scene.remove_world_object("pillar_obs_"+str(i))
        return success, traj
    
    def gripper_close(self):
        success = self.execute_trajectory(0.1, "gripper")
        return success, None 
    
    def gripper_open(self):
        success = self.execute_trajectory(-0.1, "gripper")
        return success, None 
    
    def home(self):
        self.move_arm.set_named_target('home')
        self.move_arm.set_goal_tolerance(0.001)
        success, traj, _, _ = self.move_arm.plan()
        if not success:
            rospy.logwarn('Plan Failed')
        else:
            success = self.execute_trajectory(traj, "arm") and success
        self.move_arm.clear_pose_targets()
        return success, traj
    
    def joint_state_callback(self, msg):
        self.joint_states = np.array(msg.position, dtype=np.float64)

    def execute_trajectory(self, traj, group):
        if group == "arm":
            via_points = traj.joint_trajectory.points
            data = np.zeros((len(via_points),3,5))
            for i in range(len(via_points)):
                point = via_points[i]
                data[i,0,:4] = np.array(point.positions)
                data[i,1,:4] = np.array(point.velocities)
                data[i,2,:4] = np.array(point.accelerations)
            tx = np.linspace(0, 1, len(via_points))
            y = data[:,0,:]
            # dy = data[:,1,:]
            # interp = CubicHermiteSpline(tx, y, dy)
            interp = CubicSpline(tx, y)
            yq = interp(np.linspace(0, 1, 200))
            grab_pos = self.joint_states[-1]
            yq[:,4] = grab_pos
            for yi in yq:
                msg = Float64MultiArray()
                msg.data = yi.tolist()
                error = np.linalg.norm(yi-self.joint_states)
                while error > 0.2:
                    error = np.linalg.norm(yi-self.joint_states)
                    self.command_pub.publish(msg)               
                    rospy.sleep(self.scaling/1000) 
                self.arm_command_bkp = yi
            rospy.sleep(2)
            return True
        elif group == "gripper":
            msg = Float64MultiArray()
            msg.data = self.arm_command_bkp
            msg.data[-1] = traj
            self.command_pub.publish(msg)
            rospy.sleep(2)
            return True
        rospy.logwarn('Execution Failed')
        return False       

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
