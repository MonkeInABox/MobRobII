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

    bridge_config_path = os.path.join(pkg_share, "config", "bridge_config.yaml")

    #nav2_config_path = os.path.join(pkg_share, "config", "nav2_params.yaml")

    world_path = os.path.join(pkg_share, "src", "worlds", "basic_urdf.sdf")
    # world_path = os.path.join(pkg_share, "world", "maze.world")

    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    # gz_spawn_model_launch_source = os.path.join(
    #     ros_gz_sim_share, "launch", "gz_spawn_model.launch.py"
    # )

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

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    #     parameters=[{"robot_description": Command(["xacro ", default_model_path])}],
    #     # condition=UnlessCondition(LaunchConfiguration("gui")),
    # )

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

    # spawn_entity = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
    #     launch_arguments={
    #         "world": "jamesOval",
    #         "topic": "/robot_description",
    #         "entity_name": "pioneer",
    #         # "z": "0.65",
    #         "x": "10.0",
    #         "y": "-20.0",
    #         "z": "0.0",
    #     }.items(),
    # )

    robot = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-topic",
            "robot_description",
            # "-x",
            # "10",
            # "-y",
            # "-20",
            # "-z",
            # "0.00",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "1.00",
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

    # janky_map_publisheer = Node(  # TODO: REMOVE THIS
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="janky_map_publisheer",
    #     arguments=[
    #         "0",
    #         "0",
    #         "0",
    #         "0",
    #         "0",
    #         "0",
    #         "map",
    #         "odom",
    #     ],
    #     parameters=[{"use_sim_time": True}],
    #     output="screen",
    # )

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

    # robot_steering = Node(
    #     package="rqt_robot_steering",
    #     executable="rqt_robot_steering",
    # )

    # Nav2

    # bringup_dir = get_package_share_directory("nav2_bringup")
    # configured_params = RewrittenYaml(
    #     source_file=nav2_config_path, root_key="", param_rewrites="", convert_types=True
    # )

    # navigation2_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(bringup_dir, "launch", "navigation_launch.py")
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "True",
    #         "params_file": configured_params,
    #         "autostart": "True",
    #     }.items(),
    # )
    # driveDaRobot = Node(
    #     package="pioneer_drive",
    #     executable="drive_pioneer",
    #     # name="talker",
    #     # parameters=[{"target_frame": LaunchConfiguration("target_frame")}],
    # )

    # sub_and_pub = Node(
    #     package="location_sub_and_pub",
    #     executable="sub_and_pub",
    #     # name="talker",
    #     # parameters=[{"target_frame": LaunchConfiguration("target_frame")}],
    # )

    return LaunchDescription(
        [
            # joint_state_publisher_node,
            # janky_map_publisheer,  # TODO: Remove this
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
            robot_state_publisher_node,
            # robot_localization_node,
            ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen"),
            gz_server,
            ros_gz_bridge,
            robot,
            laser_frame_fix,
            robot_localization_node,
            rviz_node,
            velocity_publisher_node,
            # robot_steering,
            # navigation2_cmd,
            #driveDaRobot,
            #sub_and_pub,
        ]
    )