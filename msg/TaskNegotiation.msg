# Published for task handoff negotiation on /fleet/task_negotiation
# Rovers use this to claim, release, and hand off goals

string      sender_id          # rover publishing this message
string      target_id          # "" = broadcast to all, or specific rover id
uint8       action             # CLAIM=0, RELEASE=1, HANDOFF_REQUEST=2, HANDOFF_ACCEPT=3, HANDOFF_REJECT=4
string      goal_id            # UUID of the goal being negotiated
float64     goal_x             # goal position (so receivers know what goal)
float64     goal_y
float64     sender_distance    # sender's current distance to the goal
string      reason             # human-readable reason: "nearest", "blocked", "path_cleared"
builtin_interfaces/Time stamp

# Action constants
uint8 CLAIM=0
uint8 RELEASE=1
uint8 HANDOFF_REQUEST=2
uint8 HANDOFF_ACCEPT=3
uint8 HANDOFF_REJECT=4
