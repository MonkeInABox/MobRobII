import os

import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# from nav2_common.launch import RewrittenYaml
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    pkg_share = get_package_share_directory("pioneer")

    default_model_path = os.path.join(
        pkg_share, "src", "robots", "pioneer.urdf"
    )
    default_rviz_config_path = os.path.join(pkg_share, "rviz", "config.rviz")

    slam_launch_dir = get_package_share_directory('slam_toolbox')

    slam_launch_file = os.path.join(slam_launch_dir, 'launch', 'online_async_launch.py')

    bridge_config_path = os.path.join(pkg_share, "config", "bridge_config.yaml")

    world_path = os.path.join(pkg_share, "src", "worlds", "basic_urdf.sdf")

    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    mapper_param_path = os.path.join(pkg_share, "config", "slam_toolbox_config.yaml")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": launch_ros.descriptions.ParameterValue(
                    Command(["xacro ", LaunchConfiguration("model")])
                )
            },
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )


    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name="ros_gz_container",
        create_own_container="True",
        use_composition="True",
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name="ros_gz_bridge",
        config_file=bridge_config_path,
        container_name="ros_gz_container",
        create_own_container="False",
        use_composition="True",
    )

    robot = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-topic",
            "robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "3.00",
            "-Y",
            "0",
        ],
        name="spawn robot",
        output="both",
    )
    laser_frame_fix = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_frame_fix",
        arguments=[
            "0",
            "0",
            "0.1",
            "0",
            "0",
            "0",
            "laser_frame",
            "pioneer3at_body/base_link/gpu_lidar",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    velocity_publisher_node = Node(
        package='pioneer',
        executable='velocity_publisher',
        name='velocity_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )


    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config/ekf.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )


    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot model file",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=default_rviz_config_path,
                description="Absolute path to rviz config file",
            ),
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', 'odom_bag', '/odom'],
                output='screen'
            ),
            robot_state_publisher_node,
            ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen"),
            gz_server,
            ros_gz_bridge,
            robot,
            laser_frame_fix,
            robot_localization_node,
            rviz_node,
            velocity_publisher_node,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch_file),
                launch_arguments={
                    'slam_params_file': mapper_param_path,
                    'use_sim_time': 'true'
                }.items(),
            ),
        ]
    )