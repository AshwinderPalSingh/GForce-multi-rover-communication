import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                             SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_path   = get_package_share_directory('hospital_robot_description')
    urdf_file  = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'hospital.world')

    rover1_description = ParameterValue(
        Command(['xacro ', urdf_file, ' robot_name:=rover1']),
        value_type=str
    )
    rover2_description = ParameterValue(
        Command(['xacro ', urdf_file, ' robot_name:=rover2']),
        value_type=str
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.join(pkg_path, '..')
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r ' + world_file,
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ══════════════════════════════════════════════════════════════════════
    #  ROVER 1 — spawn at (0.0, -2.9)
    #  EKF + RSP publish to global /tf. The multi_robot_tf_bridge (launched
    #  in multi_rover_nav2.launch.py) relays these to /rover1/tf for Nav2.
    # ══════════════════════════════════════════════════════════════════════

    rover1_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='rover1',
        parameters=[{
            'robot_description': rover1_description,
            'use_sim_time': True,
            'frame_prefix': 'rover1/',
        }],
        output='screen'
    )

    rover1_jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='rover1',
        parameters=[{
            'use_sim_time': True,
            'source_list': ['/rover1/joint_states'],
            'rate': 30
        }],
        output='screen'
    )

    rover1_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter',
        namespace='rover1',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'rover1/odom',
            'base_link_frame': 'rover1/base_link',
            'world_frame': 'rover1/odom',
            'publish_tf': True,
            'two_d_mode': True,
            'odom0': '/rover1/odom',
            'odom0_config': [True,  True,  False,
                              False, False, True,
                              False, False, False,
                              False, False, True,
                              False, False, False],
        }],
    )

    rover1_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rover1_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/rover1/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/rover1/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/rover1/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/rover1/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/rover1/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        output='screen'
    )

    rover1_cmd_relay = Node(
        package='hospital_robot_description',
        executable='cmd_vel_relay.py',
        name='rover1_cmd_vel_relay',
        output='screen',
        parameters=[{'rover_name': 'rover1'}],
    )

    spawn_rover1 = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', '/rover1/robot_description',
                    '-name',  'rover1',
                    '-x', '0.0',
                    '-y', '-2.9',
                    '-z', '0.15',
                ],
                output='screen'
            )
        ]
    )

    # ══════════════════════════════════════════════════════════════════════
    #  ROVER 2 — spawn at (3.0, -2.9)
    # ══════════════════════════════════════════════════════════════════════

    rover2_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='rover2',
        parameters=[{
            'robot_description': rover2_description,
            'use_sim_time': True,
            'frame_prefix': 'rover2/',
        }],
        output='screen'
    )

    rover2_jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='rover2',
        parameters=[{
            'use_sim_time': True,
            'source_list': ['/rover2/joint_states'],
            'rate': 30
        }],
        output='screen'
    )

    rover2_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter',
        namespace='rover2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'rover2/odom',
            'base_link_frame': 'rover2/base_link',
            'world_frame': 'rover2/odom',
            'publish_tf': True,
            'two_d_mode': True,
            'odom0': '/rover2/odom',
            'odom0_config': [True,  True,  False,
                              False, False, True,
                              False, False, False,
                              False, False, True,
                              False, False, False],
        }],
    )

    rover2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rover2_bridge',
        arguments=[
            '/rover2/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/rover2/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/rover2/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/model/rover2/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/rover2/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        output='screen'
    )

    rover2_cmd_relay = Node(
        package='hospital_robot_description',
        executable='cmd_vel_relay.py',
        name='rover2_cmd_vel_relay',
        output='screen',
        parameters=[{'rover_name': 'rover2'}],
    )

    spawn_rover2 = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', '/rover2/robot_description',
                    '-name',  'rover2',
                    '-x', '3.0',
                    '-y', '-2.9',
                    '-z', '0.15',
                ],
                output='screen'
            )
        ]
    )

    # ── TF Bridge — solves Nav2 namespace TF split ─────────────────────────
    tf_bridge = Node(
        package='hospital_robot_description',
        executable='multi_robot_tf_bridge.py',
        name='multi_robot_tf_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'rover_names': ['rover1', 'rover2'],
        }],
    )

    return LaunchDescription([
        set_ign_resource_path,
        gz_sim,
        # Rover 1
        rover1_rsp,
        rover1_jsp,
        rover1_ekf,
        rover1_bridge,
        rover1_cmd_relay,
        spawn_rover1,
        # Rover 2
        rover2_rsp,
        rover2_jsp,
        rover2_ekf,
        rover2_bridge,
        rover2_cmd_relay,
        spawn_rover2,
        # TF Bridge (starts immediately, relays as data arrives)
        tf_bridge,
    ])
