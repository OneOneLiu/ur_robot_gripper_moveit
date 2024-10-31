cmake_minimum_required(VERSION 3.22)
project(ur5_gripper_moveit_config)

set(THIS_PACKAGE_INCLUDE_DEPENDS
ament_cmake
Boost
control_msgs
Eigen3
interactive_markers
moveit_core
moveit_msgs
moveit_ros_planning
moveit_ros_planning_interface
moveit_servo
moveit_task_constructor_core
moveit_visual_tools
pluginlib
rclcpp
rclcpp_action
rviz_visual_tools
tf2_geometry_msgs
tf2_geometry_msgs
tf2_ros
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()

ament_package() # ament_package()声明这是一个AMENT包，帮助在ROS 2中管理和打包依赖关系。

# Build a robot_model_and_robot_state_tutorial executable
add_executable(robot_model_and_robot_state_tutorial src/robot_model_and_robot_state_tutorial.cpp)

target_include_directories(robot_model_and_robot_state_tutorial
    PUBLIC include)
ament_target_dependencies(robot_model_and_robot_state_tutorial
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# Install the executable
install(TARGETS robot_model_and_robot_state_tutorial
  DESTINATION lib/${PROJECT_NAME})

# build a motion_planning_api_tutorial executable
add_executable(motion_planning_api_tutorial src/motion_planning_api_tutorial.cpp)

target_include_directories(motion_planning_api_tutorial
    PUBLIC include)
ament_target_dependencies(motion_planning_api_tutorial
    ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

# Install the executable
install(TARGETS motion_planning_api_tutorial
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
