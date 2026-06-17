import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_courier')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # ── Arguments ──────────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration(
        'map',
        default=os.path.join(pkg, 'maps', 'warehouse.yaml')
    )

    # ── URDF processing ────────────────────────────────────────────────────
    urdf_path = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    robot_desc = xacro.process_file(urdf_path).toxml()

    # ── Gazebo ─────────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg, 'worlds', 'warehouse.world'),
            'verbose': 'false',
        }.items()
    )

    # ── Robot State Publisher ───────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # ── Spawn robot in Gazebo ──────────────────────────────────────────────
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'courier',
            '-x', '-4.0', '-y', '-4.0', '-z', '0.1'
        ],
        output='screen'
    )

    # ── Nav2 bringup ───────────────────────────────────────────────────────
    nav2 = TimerAction(
        period=5.0,   # wait for Gazebo + robot to be ready
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch',
                                 'bringup_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml,
                    'params_file': os.path.join(pkg, 'config',
                                                'nav2_params.yaml'),
                    'autostart': 'true',
                }.items()
            )
        ]
    )

    # ── RViz2 ──────────────────────────────────────────────────────────────
    rviz = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d',
                    os.path.join(nav2_bringup_dir, 'rviz',
                                 'nav2_default_view.rviz')],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg, 'maps', 'warehouse.yaml')
        ),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        nav2,
        rviz,
    ])