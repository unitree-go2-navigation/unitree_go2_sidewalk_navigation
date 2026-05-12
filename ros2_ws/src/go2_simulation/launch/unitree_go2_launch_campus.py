import os

import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    unitree_go2_sim = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_sim"
    ).find("unitree_go2_sim")
    go2_simulation = launch_ros.substitutions.FindPackageShare(package="go2_simulation").find("go2_simulation")
    unitree_go2_description = launch_ros.substitutions.FindPackageShare(
        package="unitree_go2_description"
    ).find("unitree_go2_description")

    campus_world_path = os.path.join(
        unitree_go2_description,
        "worlds",
        "campus.sdf",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=campus_world_path,
                description="Gazebo campus world path",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("robot_name", default_value="go2"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("world_init_x", default_value="41.45"),
            DeclareLaunchArgument("world_init_y", default_value="-126.66"),
            DeclareLaunchArgument("world_init_z", default_value="0.325"),
            DeclareLaunchArgument("world_init_roll", default_value="0.0"),
            DeclareLaunchArgument("world_init_pitch", default_value="0.0"),
            DeclareLaunchArgument("world_init_heading", default_value="-0.516"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        unitree_go2_sim,
                        "launch",
                        "unitree_go2_launch.py",
                    )
                ),
                launch_arguments={
                    "world": LaunchConfiguration("world"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "rviz": LaunchConfiguration("rviz"),
                    "robot_name": LaunchConfiguration("robot_name"),
                    "gui": LaunchConfiguration("gui"),
                    "world_init_x": LaunchConfiguration("world_init_x"),
                    "world_init_y": LaunchConfiguration("world_init_y"),
                    "world_init_z": LaunchConfiguration("world_init_z"),
                    "world_init_roll": LaunchConfiguration("world_init_roll"),
                    "world_init_pitch": LaunchConfiguration("world_init_pitch"),
                    "world_init_heading": LaunchConfiguration("world_init_heading"),
                }.items(),
            ),
        ]
    )
