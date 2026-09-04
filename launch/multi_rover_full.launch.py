"""
multi_rover_full.launch.py — ALL-IN-ONE Launch (Gazebo Classic)

Launches everything in a single command:
  1. Gazebo Classic with hospital world + 2 rovers
  2. Nav2 stacks for both rovers
  3. TF Bridge + distributed rover agents + goal dispatcher
  4. RViz with pre-loaded config

Gazebo Classic advantages:
  - Plugins publish directly to ROS 2 (no bridge needed)
  - More stable physics for differential drive
  - Simpler architecture

Usage:
  ros2 launch hospital_robot_description multi_rover_full.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                             ExecuteProcess, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_path = get_package_share_directory('hospital_robot_description')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_file  = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'hospital.world')
    map_file   = os.path.join(pkg_path, 'maps', 'my_ma_p.yaml')
    rviz_cfg   = os.path.join(pkg_path, 'config', 'multi_rover.rviz')

    rover1_params = os.path.join(pkg_path, 'config', 'nav2_params_rover1.yaml')
    rover2_params = os.path.join(pkg_path, 'config', 'nav2_params_rover2.yaml')

    rover1_description = ParameterValue(
        Command(['xacro ', urdf_file, ' robot_name:=rover1']),
        value_type=str)
    rover2_description = ParameterValue(
        Command(['xacro ', urdf_file, ' robot_name:=rover2']),
        value_type=str)

    # ══════════════════════════════════════════════════════════════════
    #  1. GAZEBO CLASSIC
    # ══════════════════════════════════════════════════════════════════

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_path, '..'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items())

    # ══════════════════════════════════════════════════════════════════
    #  2. ROVER 1  (spawn at 0.0, -2.9)
    #     Gazebo Classic plugins publish directly to ROS 2 — no bridge!
    # ══════════════════════════════════════════════════════════════════

    rover1_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='rover1', output='screen',
        parameters=[{'robot_description': rover1_description,
                      'use_sim_time': True, 'frame_prefix': 'rover1/'}])

    spawn_rover1 = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-topic', '/rover1/robot_description',
            '-entity', 'rover1',
            '-x', '0.0', '-y', '-2.9', '-z', '0.15',
        ],
        output='screen')

    # ══════════════════════════════════════════════════════════════════
    #  3. ROVER 2  (spawn at 3.0, -2.9)
    # ══════════════════════════════════════════════════════════════════

    rover2_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='rover2', output='screen',
        parameters=[{'robot_description': rover2_description,
                      'use_sim_time': True, 'frame_prefix': 'rover2/'}])

    spawn_rover2 = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=[
            '-topic', '/rover2/robot_description',
            '-entity', 'rover2',
            '-x', '3.0', '-y', '-2.9', '-z', '0.15',
        ],
        output='screen')

    # ══════════════════════════════════════════════════════════════════
    #  4. TF BRIDGE (still needed for multi-robot namespace handling)
    # ══════════════════════════════════════════════════════════════════

    tf_bridge = Node(
        package='hospital_robot_description',
        executable='multi_robot_tf_bridge.py',
        name='multi_robot_tf_bridge', output='screen',
        parameters=[{'use_sim_time': True,
                      'rover_names': ['rover1', 'rover2']}])

    # ══════════════════════════════════════════════════════════════════
    #  5. NAV2 STACKS
    # ══════════════════════════════════════════════════════════════════

    rover1_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': 'rover1', 'map': map_file,
            'params_file': rover1_params, 'use_sim_time': 'true',
            'autostart': 'true', 'use_namespace': 'true',
        }.items())

    rover2_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': 'rover2', 'map': map_file,
            'params_file': rover2_params, 'use_sim_time': 'true',
            'autostart': 'true', 'use_namespace': 'true',
        }.items())

    # ══════════════════════════════════════════════════════════════════
    #  6. AMCL INITIAL POSES
    # ══════════════════════════════════════════════════════════════════

    init_pose_r1 = ExecuteProcess(cmd=[
        'ros2', 'topic', 'pub', '--once', '/rover1/initialpose',
        'geometry_msgs/msg/PoseWithCovarianceStamped',
        ('{header: {frame_id: "map"}, pose: {pose: {'
         'position: {x: 0.0, y: -2.9, z: 0.0},'
         'orientation: {w: 1.0}},'
         'covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0,'
         ' 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}')
    ], output='screen')

    init_pose_r2 = ExecuteProcess(cmd=[
        'ros2', 'topic', 'pub', '--once', '/rover2/initialpose',
        'geometry_msgs/msg/PoseWithCovarianceStamped',
        ('{header: {frame_id: "map"}, pose: {pose: {'
         'position: {x: 3.0, y: -2.9, z: 0.0},'
         'orientation: {w: 1.0}},'
         'covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0,'
         ' 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}')
    ], output='screen')

    # ══════════════════════════════════════════════════════════════════
    #  7. DISTRIBUTED COMMUNICATION NODES
    # ══════════════════════════════════════════════════════════════════

    rover1_agent = Node(
        package='hospital_robot_description', executable='rover_agent.py',
        name='rover1_agent', output='screen',
        parameters=[{'use_sim_time': True, 'rover_id': 'rover1',
                      'auction_timeout': 2.0,
                      'progress_check_interval': 3.0,
                      'stuck_distance_threshold': 0.05,
                      'stuck_consecutive_checks': 3,
                      'reswitch_distance_ratio': 0.5,
                      'goal_tolerance': 0.5}])

    rover2_agent = Node(
        package='hospital_robot_description', executable='rover_agent.py',
        name='rover2_agent', output='screen',
        parameters=[{'use_sim_time': True, 'rover_id': 'rover2',
                      'auction_timeout': 2.0,
                      'progress_check_interval': 3.0,
                      'stuck_distance_threshold': 0.05,
                      'stuck_consecutive_checks': 3,
                      'reswitch_distance_ratio': 0.5,
                      'goal_tolerance': 0.5}])

    goal_dispatcher = Node(
        package='hospital_robot_description', executable='goal_dispatcher.py',
        name='goal_dispatcher', output='screen',
        parameters=[{'use_sim_time': True}])

    # ══════════════════════════════════════════════════════════════════
    #  8. RVIZ
    # ══════════════════════════════════════════════════════════════════

    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2', output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}])

    # ══════════════════════════════════════════════════════════════════
    #  LAUNCH ORDER
    # ══════════════════════════════════════════════════════════════════

    return LaunchDescription([
        # Phase 1: Gazebo Classic + robots
        set_model_path,
        gazebo,
        rover1_rsp, rover2_rsp,
        tf_bridge,
        TimerAction(period=8.0, actions=[spawn_rover1]),
        TimerAction(period=10.0, actions=[spawn_rover2]),

        # Phase 2: Nav2 stacks
        TimerAction(period=18.0, actions=[rover1_nav2]),
        TimerAction(period=20.0, actions=[rover2_nav2]),

        # Phase 3: AMCL initial poses
        TimerAction(period=30.0, actions=[init_pose_r1]),
        TimerAction(period=31.0, actions=[init_pose_r2]),

        # Phase 4: Communication + RViz
        TimerAction(period=34.0, actions=[
            rover1_agent, rover2_agent, goal_dispatcher,
        ]),
        TimerAction(period=36.0, actions=[rviz]),
    ])
