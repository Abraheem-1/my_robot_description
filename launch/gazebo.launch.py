from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('my_robot_description')
    world = os.path.join(pkg_path, 'worlds', 'my_saved_world.world')

    # ================= GAZEBO =================
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # ================= ROBOT DESCRIPTION =================
    robot_description = ParameterValue(
        Command(['xacro ', pkg_path + '/urdf/my_robot.xacro']),
        value_type=str
    )

    # ================= ROBOT STATE PUBLISHER =================
    # Publishes ALL TF transforms defined in the URDF automatically.
    # Do NOT add joint_state_publisher — Gazebo diff-drive plugin
    # publishes the real /joint_states. Adding joint_state_publisher
    # on top overrides them with fake zeros → wheels stuck at origin.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # ================= SPAWN ROBOT =================
    # Delayed 3s to ensure Gazebo is fully ready before spawning.
    spawn = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_entity',
                arguments=[
                    '-entity', 'my_robot',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.05',
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn,
    ])