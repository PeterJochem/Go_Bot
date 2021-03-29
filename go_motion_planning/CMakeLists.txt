cmake_minimum_required(VERSION 3.0.2)
project(go_motion_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    interbotix_sdk
    roscpp
    rospy
    std_msgs
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    message_generation
)

find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
   FILES
   home_position.srv
   pickup_piece.srv
   place_piece_in_unused.srv
   remove_piece.srv
   pickup_unused_piece.srv
   place_piece.srv
   play_piece.srv
   pickup_set_of_pieces.srv
   spawn_piece.srv
   remove_pieces_from_gazebo.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES go_motion_planning 
   CATKIN_DEPENDS interbotix_sdk roscpp rospy std_msgs message_runtime
   DEPENDS
   EIGEN3
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Add executable nodes
add_executable(${PROJECT_NAME}_pick_and_place_node src/pick_and_place.cpp)
set_target_properties(${PROJECT_NAME}_pick_and_place_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_pick_and_place_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_pick_and_place_node ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   src/spawn_items_gazebo.py
   src/demo_pick_and_place.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest_gtest(test_go_motion_planning test/test_go_motion_planning.launch test/test_go_motion_planning.cpp)
	target_link_libraries(test_go_motion_planning gtest_main ${catkin_LIBRARIES})
endif()
