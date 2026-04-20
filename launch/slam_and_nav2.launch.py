# ============================================================
#  SLAM + Nav2 Launch File — my_robot
#
#  Modes:
#  use_slam:=true  → SLAM mapping mode (async_slam_toolbox_node)
#  use_slam:=false → Localization mode (localization_slam_toolbox_node)
#
#  Key change: replaced map_server + amcl with slam_toolbox
#  localization mode — much more robust for small robots.
#
#  Usage:
#    # SLAM mapping:
#    ros2 launch my_robot_description slam_and_nav2.launch.py
#
#    # Navigation on saved map:
#    ros2 launch my_robot_description slam_and_nav2.launch.py use_slam:=false
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

    pkg_path    = get_package_share_directory('my_robot_description')
    nav2_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_path, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_path, 'config', 'display.rviz')

    # ================= LAUNCH ARGUMENTS =================
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='true = SLAM mapping | false = localization on saved map'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 automatically'
    )

    use_slam = LaunchConfiguration('use_slam')
    use_rviz = LaunchConfiguration('use_rviz')

    # ================= 1. GAZEBO + ROBOT =================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'gazebo.launch.py')
        )
    )

    # ================= 2. SLAM TOOLBOX — MAPPING MODE =================
    # Only when use_slam:=true
    # Delayed 5s so /scan and /odom are publishing
    slam_mapping = TimerAction(
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
                    {'use_sim_time': True,
                     'mode': 'mapping'}
                ],
            ),
        ]
    )

    # ================= 3. SLAM TOOLBOX — LOCALIZATION MODE =================
    # Only when use_slam:=false
    # Replaces map_server + amcl entirely.
    # slam_toolbox localization is more robust than amcl for small robots:
    #   - Updates continuously regardless of robot motion
    #   - Uses full scan matching not just particle filter
    #   - No need to set initial pose manually
    slam_localization = TimerAction(
        period=5.0,
        actions=[
            Node(
                condition=UnlessCondition(use_slam),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params,
                    {'use_sim_time': True,
                     'mode': 'localization'},
                ],
            ),
        ]
    )

    # ================= 4. NAV2 NODES =================
    # Delayed 10s — give slam_toolbox localization time to
    # load the map and establish the map→odom transform
    # before Nav2 nodes try to use it.
    nav2_nodes = TimerAction(
        period=10.0,
        actions=[
            GroupAction(actions=[

                LifecycleNode(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    namespace='',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),
                LifecycleNode(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    namespace='',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                    remappings=[('cmd_vel', 'cmd_vel')],
                ),
                LifecycleNode(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    namespace='',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),
                LifecycleNode(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    namespace='',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),
                LifecycleNode(
                    package='nav2_waypoint_follower',
                    executable='waypoint_follower',
                    name='waypoint_follower',
                    namespace='',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}],
                ),

                # --- Lifecycle Manager ---
                # Same node list for both modes —
                # slam_toolbox handles its own lifecycle independently
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
    # Delayed 12s — ensures map is fully loaded before RViz2
    # subscribes, avoiding the "No map received" issue
    rviz2 = TimerAction(
        period=12.0,
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
        use_rviz_arg,
        gazebo_launch,        # t=0s   — Gazebo + robot (spawn at t=3s)
        slam_mapping,         # t=5s   — SLAM mapping (use_slam:=true only)
        slam_localization,    # t=5s   — SLAM localization (use_slam:=false only)
        rviz2,                # t=12s  — RViz2
        nav2_nodes,           # t=10s  — Nav2 stack
    ])