cmake_minimum_required(VERSION 3.10)
project(hospital_robot_description)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ── Generate custom messages for inter-rover communication ──
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverStatus.msg"
  "msg/TaskNegotiation.msg"
  "msg/GoalBroadcast.msg"
  DEPENDENCIES builtin_interfaces
)

# ── Install URDF, meshes, worlds, maps ──
install(DIRECTORY urdf meshes worlds maps
  DESTINATION share/${PROJECT_NAME}
)

# ── Install launch files ──
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ── Install config files ──
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ── Install executable scripts ──
install(PROGRAMS
  scripts/cmd_vel_relay.py
  scripts/recovery_helper.py
  scripts/pd_teleop.py
  scripts/odom_tf_publisher.py
  scripts/goal_assignment_manager.py
  scripts/multi_robot_tf_bridge.py
  scripts/rover_agent.py
  scripts/goal_dispatcher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()