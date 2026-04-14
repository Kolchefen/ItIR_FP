"""
Launch file for TurtleBot 4 with localization, Nav2, and RViz2.

Launches map_server, amcl, and the Nav2 navigation stack (controller,
planner, smoother, behaviors, bt_navigator, waypoint_follower,
velocity_smoother) directly, so that our own `lifecycle_manager_*`
nodes are instantiated with `bond_timeout` and `service_timeout` actually
applied. `nav2_bringup/bringup_launch.py` hardcodes the lifecycle
managers with only `autostart` and `node_names` as parameters and
ignores `params_file` on those nodes -- which causes bond formation to
time out at the default 4 s under FastDDS on this hardware.

route_server, collision_monitor, and docking_server are intentionally
omitted: they are not needed for this project and each one adds bond
traffic that makes the DDS bond-up race worse.

Usage:
  ros2 launch turtlebot4_reactive_controller bringup.launch.py \
      map:=/home/lill0017/final_proj/area_map/map_area.yaml
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
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

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level for Nav2 nodes',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    log_level = LaunchConfiguration('log_level')

    params_file = PathJoinSubstitution([
        FindPackageShare('turtlebot4_reactive_controller'),
        'config',
        'nav2_params.yaml',
    ])

    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    nav_cmd_remap = [('cmd_vel', 'cmd_vel_nav')]

    localization_lifecycle_nodes = ['map_server', 'amcl']
    navigation_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'velocity_smoother',
        'bt_navigator',
        'waypoint_follower',
    ]

    lifecycle_manager_params = {
        'autostart': True,
        'bond_timeout': 10.0,
        'service_timeout': 10.0,
    }

    log_args = ['--ros-args', '--log-level', log_level]

    nav2_stack = GroupAction([
        SetParameter('use_sim_time', use_sim_time),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # == Localization ==
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_file}],
            arguments=log_args,
            remappings=tf_remaps,
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file],
            arguments=log_args,
            remappings=tf_remaps,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                **lifecycle_manager_params,
                'node_names': localization_lifecycle_nodes,
            }],
            arguments=log_args,
        ),

        # == Navigation ==
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            arguments=log_args,
            remappings=tf_remaps + nav_cmd_remap,
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file],
            arguments=log_args,
            remappings=tf_remaps,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
            arguments=log_args,
            remappings=tf_remaps,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
            arguments=log_args,
            remappings=tf_remaps + nav_cmd_remap,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file],
            arguments=log_args,
            remappings=tf_remaps,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file],
            arguments=log_args,
            remappings=tf_remaps,
        ),
        # velocity_smoother subscribes to cmd_vel (remapped to cmd_vel_nav)
        # and publishes cmd_vel_smoothed. With collision_monitor removed, we
        # remap the output directly to cmd_vel so the robot receives the
        # smoothed command.
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file],
            arguments=log_args,
            remappings=tf_remaps + nav_cmd_remap + [
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                **lifecycle_manager_params,
                'node_names': navigation_lifecycle_nodes,
            }],
            arguments=log_args,
        ),
    ])

    # == RViz2 (via nav2_bringup's helper so we keep the default Nav2 view) ==
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
        log_level_arg,
        nav2_stack,
        # Short delay so Nav2 service advertisements are visible when the
        # Nav2 panel initializes its action clients.
        TimerAction(period=3.0, actions=[rviz_launch]),
    ])
