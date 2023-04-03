search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=cube_stack.srdf
robot_name_in_srdf=cube_stack
moveit_config_pkg=cube_stack_moveit_config
robot_name=cube_stack
planning_group_name=arm
ikfast_plugin_pkg=cube_stack_arm_ikfast_plugin
base_link_name=arm_base_link
eef_link_name=gripper_active2_link
ikfast_output_path=/home/prabin/catkin_ws/src/cube_stack_env/cube_stack_arm_ikfast_plugin/src/cube_stack_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
