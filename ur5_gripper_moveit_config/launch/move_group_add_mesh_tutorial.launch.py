from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5_gripper").to_moveit_configs()

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="move_group_add_mesh",
        package="ur5_gripper_moveit_config",
        executable="move_group_add_mesh",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_demo])
