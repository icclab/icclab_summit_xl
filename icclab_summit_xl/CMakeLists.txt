cmake_minimum_required(VERSION 3.5)
project(icclab_summit_xl)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config maps robots urdf rviz meshes models
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# ## System dependencies are found with CMake's conventions
# # find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_cmake REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometric_shapes REQUIRED)
  # pcl_ros
  # pcl_conversions
find_package(moveit_msgs)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)




# ###################################
# ## catkin specific configuration ##
# ###################################
# ## The catkin_package macro generates cmake config files for your package
# ## Declare things to be passed to dependent projects
# ## INCLUDE_DIRS: uncomment this if you package contains header files
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES icclab_summit_xl
# #  CATKIN_DEPENDS rospy std_msgs
# #  DEPENDS system_lib
# )

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# # include_directories(include)
# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

# ## Declare a C++ library
# # add_library(icclab_summit_xl
# #   src/${PROJECT_NAME}/icclab_summit_xl.cpp
# # )

# ## Add cmake target dependencies of the library
# ## as an example, code may need to be generated before libraries
# ## either from message generation or dynamic reconfigure
# # add_dependencies(icclab_summit_xl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Declare a C++ executable
# add_executable(dock_arm_no_constraints src/dock_arm_no_constraints.cpp)
# add_executable(dock_arm_with_constraints src/dock_arm_with_constraints.cpp)
add_executable(move_arm_to_pose src/move_arm_to_pose.cpp)
# add_executable(move_arm_to_pose_with_ik src/move_arm_to_pose_with_ik.cpp)
# add_executable(move_arm_to_pose_lab02 src/move_arm_to_pose_lab02.cpp)
# ## Add cmake target dependencies of the executable
# ## same as for the library above
# # add_dependencies(icclab_summit_xl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Specify libraries to link a library or executable target against
# target_link_libraries(dock_arm_no_constraints
#    ${catkin_LIBRARIES}
#  )

# target_link_libraries(dock_arm_with_constraints
#    ${catkin_LIBRARIES}
#  )
 
# target_link_libraries(move_arm_to_pose_with_ik
#    ${catkin_LIBRARIES}
#  )

install(TARGETS move_arm_to_pose
  DESTINATION lib/${PROJECT_NAME})

ament_target_dependencies(move_arm_to_pose rclcpp moveit_ros_planning_interface)

# target_link_libraries(move_arm_to_pose_lab02
#    ${catkin_LIBRARIES}
#  )

ament_export_dependencies(ros_gz_bridge)
ament_export_dependencies(ros_gz_sim)

ament_package()