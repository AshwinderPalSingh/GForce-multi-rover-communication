# Published by each rover at 1Hz on /fleet/rover_status
# Allows all rovers + base station to know each rover's state

string      rover_id           # "rover1", "rover2", etc.
uint8       state              # IDLE=0, NAVIGATING=1, BLOCKED=2, COMPLETED=3
float64     position_x         # current position in map frame
float64     position_y
float64     heading            # yaw in radians
float64     distance_to_goal   # current distance to active goal (INF if idle)
float64     battery_level      # 0.0-100.0 (for future real hardware use)
string      active_goal_id     # UUID of the goal being pursued ("" if idle)
builtin_interfaces/Time stamp

# State constants (for reference — use the uint8 values)
uint8 IDLE=0
uint8 NAVIGATING=1
uint8 BLOCKED=2
uint8 COMPLETED=3
