"""
Launch file for TurtleBot 4 with localization, Nav2, and RViz2.

Spawns a single component_container_isolated (nav2_container) and loads both
the localization stack (map_server + amcl + lifecycle_manager_localization)
and the nav2 stack (controller, planner, smoother, behaviors, bt_navigator,
waypoint_follower, velocity_smoother) as composable nodes into it. This keeps
lifecycle bonds in-process and avoids the DDS churn that caused AMCL bond
timeouts when localization ran in separate processes.

Usage:
  ros2 launch turtlebot4_reactive_controller bringup.launch.py \
      map:=/home/lill0017/final_proj/area_map/map_area.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/lill0017/final_proj/area_map/map_area.yaml',
        description='Full path to map yaml file',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 (set false when running on the Pi with no display)',
    )

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    params_file = PathJoinSubstitution([
        FindPackageShare('turtlebot4_reactive_controller'),
        'config',
        'nav2_params.yaml',
    ])

    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'localization_launch.py',
            ])
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            ])
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot4_viz'),
                'launch',
                'view_navigation.launch.py',
            ])
        ),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        use_rviz_arg,
        nav2_container,
        localization,
        nav2,
        rviz,
    ])
