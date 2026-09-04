# Published by base station on /fleet/goal_broadcast
# Announces a new goal to all rovers in the fleet

string      goal_id            # UUID for tracking this goal
float64     goal_x             # target position in map frame
float64     goal_y
float64     goal_yaw           # target orientation (radians)
builtin_interfaces/Time stamp
