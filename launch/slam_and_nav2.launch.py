# ============================================================
#  SLAM + Nav2 Launch File — my_robot (FINAL FIX)
#
#  Root cause of crash: nav2_bringup's bringup_launch.py
#  internally does eval() on the 'slam' argument and chokes
#  on Python bool vs string. Fix: don't use bringup_launch.py
#  at all — launch Nav2 nodes directly instead.
#
#  Usage:
#    # SLAM mapping (build a new map):
#    ros2 launch my_robot_description slam_and_nav2.launch.py
#
#    # Navigation on saved map:
#    ros2 launch my_robot_description slam_and_nav2.launch.py \
#        use_slam:=false map:=/home/user/my_map.yaml
# ============================================================

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path     = get_package_share_directory('my_robot_description')
    nav2_params  = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    slam_params  = os.path.join(pkg_path, 'config', 'slam_params.yaml')
    rviz_config  = os.path.join(pkg_path, 'config', 'display.rviz')

    # ================= LAUNCH ARGUMENTS =================
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='true = SLAM mapping | false = Nav2 on saved map'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_path, 'maps', 'my_map.yaml'),
        description='Path to saved map yaml (only used when use_slam:=false)'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 automatically'
    )

    use_slam = LaunchConfiguration('use_slam')
    map_file = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')

    # ================= 1. GAZEBO + ROBOT =================
    # Reuses gazebo.launch.py (robot_state_publisher + spawn)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'gazebo.launch.py')
        )
    )

    # ================= 2. SLAM TOOLBOX =================
    # Only when use_slam:=true
    # Delayed 5s so /scan and /odom are publishing
    slam_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                condition=IfCondition(use_slam),
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params,
                    {'use_sim_time': True}
                ],
            )
        ]
    )

    # ================= 3. MAP SERVER =================
    # Only when use_slam:=false (navigation on saved map)
    # slam_toolbox already publishes /map during SLAM mode
    map_server_node = TimerAction(
        period=5.0,
        actions=[
            LifecycleNode(
                condition=UnlessCondition(use_slam),
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                namespace='',
                output='screen',
                parameters=[
                    nav2_params,
                    {'yaml_filename': map_file},
                    {'use_sim_time': True}
                ],
            )
        ]
    )

    # ================= 4. NAV2 NODES =================
    # Launched directly — no bringup_launch.py wrapper.
    # Delayed 8s so map frame is available from SLAM or map_server.

    nav2_nodes = TimerAction(
        period=8.0,
        actions=[
            GroupAction(actions=[

                # --- Planner (global path) ---
                LifecycleNode(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    namespace='',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),

                # --- Controller (local path / cmd_vel) ---
                LifecycleNode(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    namespace='',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                    remappings=[('cmd_vel', 'cmd_vel')],
                ),

                # --- Behaviors (recovery: spin, backup, wait) ---
                LifecycleNode(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    namespace='',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),

                # --- BT Navigator (orchestrates everything) ---
                LifecycleNode(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    namespace='',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),

                # --- Waypoint Follower ---
                LifecycleNode(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    namespace='',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),

                # --- Lifecycle Manager ---
                # Activates all Nav2 lifecycle nodes in order.
                # This is what actually starts everything running.
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': [
                            'planner_server',
                            'controller_server',
                            'behavior_server',
                            'bt_navigator',
                            'waypoint_follower',
                        ]}
                    ],
                ),

            ])
        ]
    )

    # ================= 5. RVIZ2 =================
    rviz2 = TimerAction(
        period=6.0,
        actions=[
            Node(
                condition=IfCondition(use_rviz),
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        use_slam_arg,
        map_arg,
        use_rviz_arg,
        gazebo_launch,   # t=0s  — Gazebo + robot (spawn at t=3s inside)
        slam_node,       # t=5s  — SLAM starts after /scan is live
        map_server_node, # t=5s  — map_server (only if use_slam:=false)
        rviz2,           # t=6s  — RViz2
        nav2_nodes,      # t=8s  — all Nav2 nodes + lifecycle manager
    ])