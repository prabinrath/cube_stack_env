import sys
import rospy
import moveit_commander

class MoveItHandler():
    def __init__(self, robot_description='robot_description'):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander(robot_description=robot_description)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_arm = moveit_commander.MoveGroupCommander("arm", robot_description=robot_description)
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper", robot_description=robot_description)
        print('Arm Reference:', self.move_arm.get_pose_reference_frame(), '--->', self.move_arm.get_end_effector_link())
    
    def __del__(self):
        rospy.loginfo('Shutting Down MoveIt Commander')
        moveit_commander.roscpp_shutdown()

    def move_to(self, goal_position):
        self.move_arm.set_position_target(goal_position)
        self.move_arm.set_goal_tolerance(0.001)
        success = self.move_arm.go(wait=True)
        if not success:
            rospy.logwarn('Plan/Execution Failed')
        self.move_arm.stop()
        self.move_arm.clear_pose_targets()
    
    def gripper_close(self):
        self.move_gripper.set_named_target('gripper_close')
        self.move_gripper.set_goal_tolerance(0.0001)
        success = self.move_gripper.go(wait=True)
        if not success:
            rospy.logwarn('Plan/Execution Failed')
        self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()
    
    def gripper_open(self):
        self.move_gripper.set_named_target('gripper_open')
        self.move_gripper.set_goal_tolerance(0.0001)
        success = self.move_gripper.go(wait=True)
        if not success:
            rospy.logwarn('Plan/Execution Failed')
        self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()
    
    def home(self):
        self.move_arm.set_named_target('home')
        self.move_arm.set_goal_tolerance(0.001)
        success = self.move_arm.go(wait=True)
        if not success:
            rospy.logwarn('Plan/Execution Failed')
        self.move_arm.stop()
        self.move_arm.clear_pose_targets()
