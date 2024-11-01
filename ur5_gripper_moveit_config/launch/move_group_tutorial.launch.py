import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur5_gripper")
        .robot_description(file_path="config/ur5_with_robotiq_2f_85.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("ur5_gripper_moveit_config") + "/launch/moveit_rviz.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    '''
    这段代码使用ROS 2中的tf2_ros包，创建一个静态坐标变换（Static Transform）发布节点，定义一个固定的坐标变换关系。这段代码的主要作用是在“world”坐标系和“panda_link0”坐标系之间创建一个静态的TF变换，并将该变换信息发布到TF树中，供其他节点使用。
    
    package：指定ROS 2包名，这里是tf2_ros。
    executable：指定可执行文件名static_transform_publisher。这个节点负责发布静态坐标变换。
    name：节点的名字为static_transform_publisher。
    output：log表示将节点的输出记录到日志文件中。
    arguments：
    前6个参数定义了两个坐标系之间的变换关系（平移和旋转）：
    0.0, 0.0, 0.0：表示平移量（x, y, z），此处无平移。
    0.0, 0.0, 0.0：表示旋转量（roll, pitch, yaw），此处无旋转。
    "world"：父坐标系的名称。
    "base_link"：子坐标系的名称。
    
    '''
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur5_gripper_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "manipulator_controller",
        "gripper_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ]
        + load_controllers
    )

'''
Current_problem:
- The original rivz started by the original launch file contains OMPL planner correctly, but the rivz started by the launch file in this modified ur5_moveit_config package does not contain OMPL planner. This indicates that the problem lies in the ompl_planning.yaml file in the config folder of the panda moveit package. How can I generate such a config file for ur5_moveit_config package?

Some references:
- https://github.com/moveit/moveit2/issues/2256#issuecomment-1655217810
- htt
'''