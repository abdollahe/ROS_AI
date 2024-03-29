cmake_minimum_required(VERSION 2.8.3)
project(ur_robot_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ur_robot
  ur_robot_moveit_config
  message_generation
  rosbag

)

#find_package(gazebo REQUIRED)
#link_directories(${GAZEBO_LIBRARY_DIRS})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_package(ignition-math4 QUIET)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   StampedJointVelocity.msg
   StampedJointPosition.msg
   StampedGripperState.msg
   JointStateSimple.msg
   PoseMessageSimple.msg
   RosBagConfig.msg
   SystemConfig.msg
   BagConfig.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Attach.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
#   rosgraph_msgs

 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_robot_gazebo
#  CATKIN_DEPENDS roscpp rospy ur_robot ur_robot_moveit_config
#  DEPENDS system_lib
   DEPENDS
        roscpp
        gazebo_ros
        message_runtime
   INCLUDE_DIRECTORIES
        include


)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ur_robot_gazebo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ur_robot_gazebo_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur_robot_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


add_library(schunk_gripper_plugin plugins/schunk_gripper_plugin.cpp)
add_library(gazebo_ros_link_attacher plugins/gazebo_ros_link_attacher.cpp)
add_library(object_pose_plugin plugins/ObjectPosePlugin.cpp)


target_link_libraries(schunk_gripper_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(gazebo_ros_link_attacher ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(object_pose_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})


add_executable(joint_state_listener_node src/JointInfoListenerNode.cpp src/JointInfoListenerClass.cpp)
target_link_libraries(joint_state_listener_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(target_pose_rosbag_node src/TargetPoseRosBag.cpp)
target_link_libraries(target_pose_rosbag_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(goal_pose_rosbag_node src/GoalPoseRosBag.cpp )
target_link_libraries(goal_pose_rosbag_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(rosbag_read_node src/ROSbagReadNode.cpp)
target_link_libraries(rosbag_read_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(magnetic_gripper_state_rosbag_node src/MagneticGripperRosBag.cpp)
target_link_libraries(magnetic_gripper_state_rosbag_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(joint_position_rosbag_node src/JointPositionRosbagNode.cpp)
target_link_libraries(joint_position_rosbag_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(joint_velocity_rosbag_node src/JointVelocityRosbagNode.cpp)
target_link_libraries(joint_velocity_rosbag_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(arm_position_rosbag_node src/ArmTfRosbagNode.cpp)
target_link_libraries(arm_position_rosbag_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(main_controller src/MainController.cpp)
target_link_libraries(main_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} config_file_setup)

#add_executable(config_file_setup src/ConfigFileSetup.cpp)
#target_link_libraries(config_file_setup ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



add_library(config_file_setup src/ConfigFileSetup.cpp)
target_link_libraries(config_file_setup
        ${GAZEBO_LIBRARIES}
        ${catkin_LIBRARIES}
        )

### Create the libObjectDisposalPlugin.so library.
###set(object_disposal_plugin_name ObjectDisposalPlugin)
#add_library(ContainPluginReceiver plugins/ContainPluginReceiver.cpp)
#target_link_libraries(ContainPluginReceiver
#        ${GAZEBO_LIBRARIES}
#        ${catkin_LIBRARIES}
#        )