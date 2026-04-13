"""
Launch file for TurtleBot 4 with localization, Nav2, and RViz2.

Uses nav2_bringup's combined launch which handles lifecycle management
for both localization and navigation in the correct order.

Usage:
  ros2 launch turtlebot4_reactive_controller bringup.launch.py \
      map:=/home/lill0017/final_proj/area_map/map_area.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/lill0017/final_proj/area_map/map_area.yaml',
        description='Full path to map yaml file',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')

    # == 1. Nav2 bringup (localization + navigation with unified lifecycle) ==
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_composition': 'False',
            'params_file': PathJoinSubstitution([
                FindPackageShare('turtlebot4_reactive_controller'),
                'config',
                'nav2_params.yaml',
            ]),
        }.items(),
    )

    # == 2. RViz2 ==
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'rviz_launch.py',
            ])
        ),
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        nav2_bringup,
        rviz_launch,
    ])
