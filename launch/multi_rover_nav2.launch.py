import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('hospital_robot_description')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    map_file = os.path.join(pkg_path, 'maps', 'hospital_map.yaml')

    rover1_params = os.path.join(pkg_path, 'config', 'nav2_params_rover1.yaml')
    rover2_params = os.path.join(pkg_path, 'config', 'nav2_params_rover2.yaml')

    # ── Rover 1 Nav2 Stack ─────────────────────────────────────────────────
    rover1_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': 'rover1',
            'map': map_file,
            'params_file': rover1_params,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_namespace': 'true',
        }.items()
    )

    # ── Rover 2 Nav2 Stack ─────────────────────────────────────────────────
    rover2_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': 'rover2',
            'map': map_file,
            'params_file': rover2_params,
            'use_sim_time': 'true',
            'autostart': 'true',
            'use_namespace': 'true',
        }.items()
    )

    # ── Auto-set initial poses for AMCL ───────────────────────────────────
    # AMCL needs an initialpose message to start publishing map→odom TF.
    # We publish this 30s after Nav2 starts (AMCL must be active first).
    initialpose_rover1 = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/rover1/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            ('{header: {frame_id: "map"}, pose: {pose: {'
             'position: {x: 0.0, y: -2.9, z: 0.0},'
             'orientation: {w: 1.0}},'
             'covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0,'
             ' 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'),
        ],
        output='screen',
    )

    initialpose_rover2 = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/rover2/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            ('{header: {frame_id: "map"}, pose: {pose: {'
             'position: {x: 3.0, y: -2.9, z: 0.0},'
             'orientation: {w: 1.0}},'
             'covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0,'
             ' 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}'),
        ],
        output='screen',
    )

    # ── Goal Assignment Manager ────────────────────────────────────────────
    goal_assignment_manager = Node(
        package='hospital_robot_description',
        executable='goal_assignment_manager.py',
        name='goal_assignment_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'rover_names': ['rover1', 'rover2'],
            'progress_check_interval': 3.0,
            'stuck_distance_threshold': 0.05,
            'stuck_consecutive_checks': 3,
            'reswitch_distance_ratio': 0.5,
            'goal_tolerance': 0.5,
        }],
    )

    return LaunchDescription([
        # Nav2 stacks delayed to let Gazebo + robots spawn first
        TimerAction(period=20.0, actions=[rover1_nav2]),
        TimerAction(period=22.0, actions=[rover2_nav2]),
        # Auto-set initial poses so AMCL starts localizing (map frame appears)
        TimerAction(period=32.0, actions=[initialpose_rover1]),
        TimerAction(period=33.0, actions=[initialpose_rover2]),
        # Goal manager after AMCL is ready
        TimerAction(period=38.0, actions=[goal_assignment_manager]),
    ])
