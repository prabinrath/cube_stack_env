cmake_minimum_required(VERSION 3.1.3)
project(cube_stack_arm_ikfast_plugin)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf2_kdl
  tf2_eigen
)
moveit_build_options()

find_package(LAPACK REQUIRED)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

catkin_package()

set(IKFAST_LIBRARY_NAME cube_stack_arm_moveit_ikfast_plugin)
add_library(${IKFAST_LIBRARY_NAME} src/cube_stack_arm_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})
# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(${IKFAST_LIBRARY_NAME} PRIVATE -Wno-unused-variable -Wno-unused-parameter)

install(TARGETS
  ${IKFAST_LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  FILES
  cube_stack_arm_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
