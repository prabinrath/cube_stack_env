import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
move_arm = moveit_commander.MoveGroupCommander("arm")
move_gripper = moveit_commander.MoveGroupCommander("gripper")
print('Arm Reference:', move_arm.get_pose_reference_frame(), '--->', move_arm.get_end_effector_link())
# wpose = move_group.get_current_pose().pose
# print(wpose)

pose_goal = [0.1, -0.1, 0.05]
move_arm.set_position_target(pose_goal)
# move_arm.set_named_target("home")
move_arm.set_goal_tolerance(0.001)
success, trajectory, _, error_code = move_arm.plan()
if success:
    print(trajectory)
    move_arm.execute(trajectory, wait=True)
else:
    rospy.logerr("Plan Failed")

# pose_goal = [0.1, -0.1, 0.05]
# move_arm.set_position_target(pose_goal)
# # move_arm.set_named_target("home")
# move_arm.set_goal_tolerance(0.001)
# success = move_arm.go(wait=True)
# move_arm.stop()
# move_arm.clear_pose_targets()

# # move_gripper.set_named_target("gripper_close")
# move_gripper.set_named_target("gripper_close")
# move_gripper.set_goal_tolerance(0.0001)
# success = move_gripper.go(wait=True)
# move_gripper.stop()
# move_gripper.clear_pose_targets()

# pose_goal = [0.1, 0.1, 0.05]
# move_arm.set_position_target(pose_goal)
# # move_arm.set_named_target("home")
# move_arm.set_goal_tolerance(0.001)
# success = move_arm.go(wait=True)
# move_arm.stop()
# move_arm.clear_pose_targets()

# # move_gripper.set_named_target("gripper_close")
# move_gripper.set_named_target("gripper_open")
# move_gripper.set_goal_tolerance(0.0001)
# success = move_gripper.go(wait=True)
# move_gripper.stop()
# move_gripper.clear_pose_targets()

moveit_commander.roscpp_shutdown()