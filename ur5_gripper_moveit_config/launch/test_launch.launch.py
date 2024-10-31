from launch import LaunchDescription 
from launch_ros.actions import Node 
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description(): 
    moveit_config = MoveItConfigsBuilder("ur5_gripper").to_moveit_configs() 
    
    tutorial_node = Node(
        package="ur5_gripper_moveit_config", 
        executable="robot_model_and_robot_state_tutorial", 
        output="screen", 
        parameters=[
            moveit_config.robot_description, 
            moveit_config.robot_description_semantic, 
            moveit_config.robot_description_kinematics, 
            ], 
        ) 
    
    return LaunchDescription([tutorial_node])

'''
我这个程序在执行时会有警告，但是不影响程序的运行，警告如下：
[robot_model_and_robot_state_tutorial-1] Warning: Link 'frame' is not known to URDF. Cannot disable/enable collisons.
[robot_model_and_robot_state_tutorial-1]          at line 589 in /catkin_ws/src/srdfdom/src/model.cpp
[robot_model_and_robot_state_tutorial-1] Warning: Link 'frame' is not known to URDF. Cannot disable/enable collisons.
[robot_model_and_robot_state_tutorial-1]          at line 589 in /catkin_ws/src/srdfdom/src/model.cpp
[robot_model_and_robot_state_tutorial-1] Warning: Link 'frame' is not known to URDF. Cannot disable/enable collisons.
[robot_model_and_robot_state_tutorial-1]          at line 589 in /catkin_ws/src/srdfdom/src/model.cpp
[robot_model_and_robot_state_tutorial-1] Warning: Link 'realsense' is not known to URDF. Cannot disable/enable collisons.
[robot_model_and_robot_state_tutorial-1]          at line 589 in /catkin_ws/src/srdfdom/src/model.cpp
[robot_model_and_robot_state_tutorial-1] Warning: Link 'realsense' is not known to URDF. Cannot disable/enable collisons.
[robot_model_and_robot_state_tutorial-1]          at line 589 in /catkin_ws/src/srdfdom/src/model.cpp

这是因为我在moveit_setup_assistant中时使用的urdf文件并没有包含这些frame和realsense的信息，是后续手动加上的，所以会有这个警告。
'''