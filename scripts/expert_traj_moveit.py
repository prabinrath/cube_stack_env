import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

robot = moveit_commander.RobotCommander()

group_name = "arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
print('Reference:', move_group.get_pose_reference_frame(), '--->', move_group.get_end_effector_link())
# wpose = move_group.get_current_pose().pose
# print(wpose)

# pose_goal = geometry_msgs.msg.Pose()
# pose_goal.orientation.w = 1.0
# pose_goal.position.x = 0.1
# pose_goal.position.y = 0.1
# pose_goal.position.z = 0.1
# move_group.set_pose_target(pose_goal)
# move_group.set_goal_tolerance(0.01)

# pose_goal = [0.1, 0.1, 0.1]
# move_group.set_position_target(pose_goal)
# move_group.set_goal_tolerance(0.01)

move_group.set_named_target("home")

success = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()

moveit_commander.roscpp_shutdown()