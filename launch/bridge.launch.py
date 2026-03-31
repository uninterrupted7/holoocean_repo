"""
bridge.launch.py
================
Launches:
  1. holoocean_bridge  – HoloOcean simulation + ROS2 publisher
  2. static_transform_publisher – fixed TF for sonar, IMU, GPS offsets
  3. (optional) RViz2 for visualisation

Usage:
  ros2 launch holoocean_ros2_bridge bridge.launch.py
  ros2 launch holoocean_ros2_bridge bridge.launch.py use_rviz:=true
  ros2 launch holoocean_ros2_bridge bridge.launch.py \
      scenario_file:=/path/to/your_scenario.json
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('holoocean_ros2_bridge')
    default_scenario = os.path.join(pkg_share, 'config', 'surface_mapping_scenario.json')
    default_rviz     = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    # ── Launch arguments ──────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('scenario_file',
                              default_value=default_scenario,
                              description='Path to HoloOcean scenario JSON'),
        DeclareLaunchArgument('scenario_name',   default_value='PierHarbor-HoveringImagingSonar',
                              description='Built-in HoloOcean scenario name (fallback)'),
        DeclareLaunchArgument('world_frame',     default_value='world'),
        DeclareLaunchArgument('vessel_frame',    default_value='base_link'),
        DeclareLaunchArgument('sonar_frame',     default_value='sonar_link'),
        DeclareLaunchArgument('imu_frame',       default_value='imu_link'),
        DeclareLaunchArgument('gps_frame',       default_value='gps_link'),
        DeclareLaunchArgument('lat_origin',      default_value='0.0'),
        DeclareLaunchArgument('lon_origin',      default_value='0.0'),
        DeclareLaunchArgument('sonar_azimuth_deg',  default_value='120.0'),
        DeclareLaunchArgument('sonar_range_min',    default_value='0.5'),
        DeclareLaunchArgument('sonar_range_max',    default_value='50.0'),
        DeclareLaunchArgument('thrust_scale',       default_value='50.0'),
        DeclareLaunchArgument('auto_mode',         default_value='true',
                              description='Enable autonomous movement pattern'),
        DeclareLaunchArgument('use_rviz',        default_value='false',
                              description='Launch RViz2 for visualisation'),
    ]

    # ── Nodes ─────────────────────────────────────────────────────────────

    bridge_node = Node(
        package='holoocean_ros2_bridge',
        executable='holoocean_bridge',
        name='holoocean_bridge',
        output='screen',
        parameters=[{
            'scenario_file':      LaunchConfiguration('scenario_file'),
            'scenario_name':      LaunchConfiguration('scenario_name'),
            'world_frame':        LaunchConfiguration('world_frame'),
            'vessel_frame':       LaunchConfiguration('vessel_frame'),
            'sonar_frame':        LaunchConfiguration('sonar_frame'),
            'imu_frame':          LaunchConfiguration('imu_frame'),
            'gps_frame':          LaunchConfiguration('gps_frame'),
            'lat_origin':         LaunchConfiguration('lat_origin'),
            'lon_origin':         LaunchConfiguration('lon_origin'),
            'sonar_azimuth_deg':  LaunchConfiguration('sonar_azimuth_deg'),
            'sonar_range_min':    LaunchConfiguration('sonar_range_min'),
            'sonar_range_max':    LaunchConfiguration('sonar_range_max'),
            'thrust_scale':       LaunchConfiguration('thrust_scale'),
            'auto_mode':          LaunchConfiguration('auto_mode'),
        }]
    )

    # Static TF: world → odom (identity for simulation)
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )

    # Optional RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription(
        args + [
            bridge_node,
            static_tf_world_odom,
            rviz_node,
        ]
    )
