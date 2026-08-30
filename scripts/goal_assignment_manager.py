#!/usr/bin/env python3
"""
Goal Assignment Manager — Multi-Rover Intelligent Task Allocation

This node implements a smart goal assignment system for two (or more) rovers:

1. When a goal arrives on /goal_pose, it assigns the goal to the NEAREST rover.
2. It monitors the assigned rover's progress. If the rover gets stuck
   (moves < stuck_distance_threshold over stuck_consecutive_checks × check_interval),
   it cancels that rover and switches to the other rover.
3. While the backup rover is navigating, it periodically checks whether the
   originally-blocked rover's path has cleared. If the original rover is now
   significantly closer (distance < reswitch_distance_ratio × backup's distance),
   it re-switches back. Otherwise it stays with the current rover.

Topics:
  Subscribes:
    /goal_pose            (geometry_msgs/PoseStamped)  — incoming goal from user/RViz
    /rover1/odom          (nav_msgs/Odometry)          — rover1 position
    /rover2/odom          (nav_msgs/Odometry)          — rover2 position

  Action Clients:
    /rover1/navigate_to_pose   (NavigateToPose)
    /rover2/navigate_to_pose   (NavigateToPose)

  Service Clients:
    /rover1/compute_path_to_pose   (ComputePathToPose)  — for path feasibility check
    /rover2/compute_path_to_pose   (ComputePathToPose)  — for path feasibility check

Parameters:
    rover_names                 (string[])  — list of rover namespace names
    progress_check_interval     (double)    — seconds between progress checks
    stuck_distance_threshold    (double)    — meters; movement below this = stuck
    stuck_consecutive_checks    (int)       — number of consecutive stuck checks before switch
    reswitch_distance_ratio     (double)    — re-assign if blocked rover's dist < ratio × current's dist
    goal_tolerance              (double)    — meters; distance to goal to declare success
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, ComputePathToPose


class RoverState:
    """Tracks one rover's position and stuck counter."""
    def __init__(self, name):
        self.name = name
        self.position = Point()         # latest position from odom
        self.last_check_position = None  # position at previous progress check
        self.stuck_count = 0            # consecutive stuck checks
        self.goal_handle = None         # active Nav2 goal handle
        self.is_navigating = False


class GoalAssignmentManager(Node):

    # ── State Machine ──
    STATE_IDLE = 'IDLE'
    STATE_NAVIGATING = 'NAVIGATING'
    STATE_SWITCHING = 'SWITCHING'

    def __init__(self):
        super().__init__('goal_assignment_manager')
        self.cb_group = ReentrantCallbackGroup()

        # ── Parameters ──
        self.declare_parameter('rover_names', ['rover1', 'rover2'])
        self.declare_parameter('progress_check_interval', 3.0)
        self.declare_parameter('stuck_distance_threshold', 0.05)
        self.declare_parameter('stuck_consecutive_checks', 3)
        self.declare_parameter('reswitch_distance_ratio', 0.5)
        self.declare_parameter('goal_tolerance', 0.5)

        self.rover_names = self.get_parameter('rover_names').get_parameter_value().string_array_value
        self.check_interval = self.get_parameter('progress_check_interval').get_parameter_value().double_value
        self.stuck_threshold = self.get_parameter('stuck_distance_threshold').get_parameter_value().double_value
        self.stuck_checks_needed = self.get_parameter('stuck_consecutive_checks').get_parameter_value().integer_value
        self.reswitch_ratio = self.get_parameter('reswitch_distance_ratio').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        # ── Rover state tracking ──
        self.rovers = {}
        self.nav_clients = {}
        self.path_clients = {}

        for name in self.rover_names:
            self.rovers[name] = RoverState(name)

            # Odom subscriber
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, n=name: self._odom_callback(n, msg),
                10,
                callback_group=self.cb_group,
            )

            # Nav2 NavigateToPose action client
            client = ActionClient(
                self, NavigateToPose,
                f'/{name}/navigate_to_pose',
                callback_group=self.cb_group,
            )
            self.nav_clients[name] = client

            # Nav2 ComputePathToPose action client (for path feasibility checks)
            path_client = ActionClient(
                self, ComputePathToPose,
                f'/{name}/compute_path_to_pose',
                callback_group=self.cb_group,
            )
            self.path_clients[name] = path_client

        # ── Goal subscriber ──
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._goal_callback,
            10,
            callback_group=self.cb_group,
        )

        # ── State ──
        self.state = self.STATE_IDLE
        self.current_goal = None           # PoseStamped
        self.assigned_rover = None         # name of rover currently navigating
        self.originally_blocked_rover = None  # name of rover that was initially assigned but got stuck
        self.progress_timer = None

        self.get_logger().info(
            f'Goal Assignment Manager started with rovers: {self.rover_names}'
        )
        self.get_logger().info(
            f'Config: check_interval={self.check_interval}s, '
            f'stuck_threshold={self.stuck_threshold}m, '
            f'stuck_checks={self.stuck_checks_needed}, '
            f'reswitch_ratio={self.reswitch_ratio}'
        )

    # ══════════════════════════════════════════════════════════════════════
    #  CALLBACKS
    # ══════════════════════════════════════════════════════════════════════

    def _odom_callback(self, rover_name, msg):
        """Update rover position from odometry."""
        self.rovers[rover_name].position = msg.pose.pose.position

    def _goal_callback(self, msg):
        """New goal received — assign to nearest rover."""
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

        # Cancel any active navigation first
        if self.state != self.STATE_IDLE:
            self.get_logger().info('Cancelling previous goal before assigning new one')
            self._cancel_all()

        self.current_goal = msg
        self.originally_blocked_rover = None

        # Find nearest rover
        nearest = self._find_nearest_rover(msg.pose.position)
        self.get_logger().info(f'Nearest rover to goal: {nearest}')
        self._assign_goal(nearest)

    # ══════════════════════════════════════════════════════════════════════
    #  GOAL ASSIGNMENT
    # ══════════════════════════════════════════════════════════════════════

    def _find_nearest_rover(self, goal_position):
        """Return name of rover closest to the goal."""
        best_name = None
        best_dist = float('inf')
        for name, rover in self.rovers.items():
            d = self._distance(rover.position, goal_position)
            self.get_logger().info(f'  {name} distance to goal: {d:.2f}m')
            if d < best_dist:
                best_dist = d
                best_name = name
        return best_name

    def _assign_goal(self, rover_name):
        """Send NavigateToPose to the specified rover."""
        self.assigned_rover = rover_name
        rover = self.rovers[rover_name]

        # Reset stuck tracking
        for r in self.rovers.values():
            r.stuck_count = 0
            r.last_check_position = None

        rover.last_check_position = Point(
            x=rover.position.x,
            y=rover.position.y,
            z=rover.position.z,
        )

        # Build Nav2 goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self.current_goal

        client = self.nav_clients[rover_name]
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f'Nav2 action server not available for {rover_name}!'
            )
            return

        self.get_logger().info(f'Sending goal to {rover_name}...')
        send_future = client.send_goal_async(
            nav_goal,
            feedback_callback=lambda fb, n=rover_name: self._nav_feedback(n, fb),
        )
        send_future.add_done_callback(
            lambda future, n=rover_name: self._goal_accepted_callback(n, future)
        )

        self.state = self.STATE_NAVIGATING

        # Start progress monitoring timer
        if self.progress_timer:
            self.progress_timer.cancel()
        self.progress_timer = self.create_timer(
            self.check_interval, self._check_progress, callback_group=self.cb_group
        )

    def _goal_accepted_callback(self, rover_name, future):
        """Called when Nav2 accepts or rejects our goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'{rover_name}: Goal was REJECTED by Nav2')
            self.state = self.STATE_IDLE
            return

        self.get_logger().info(f'{rover_name}: Goal ACCEPTED by Nav2')
        self.rovers[rover_name].goal_handle = goal_handle
        self.rovers[rover_name].is_navigating = True

        # Register result callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, n=rover_name: self._nav_result_callback(n, f)
        )

    def _nav_feedback(self, rover_name, feedback_msg):
        """Nav2 feedback — we mainly rely on odom for position tracking."""
        pass

    def _nav_result_callback(self, rover_name, future):
        """Called when Nav2 navigation completes (success, failure, or cancel)."""
        result = future.result()
        status = result.status

        self.rovers[rover_name].is_navigating = False
        self.rovers[rover_name].goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'✅ {rover_name}: Goal REACHED successfully!'
            )
            self.state = self.STATE_IDLE
            self.current_goal = None
            self.assigned_rover = None
            self.originally_blocked_rover = None
            if self.progress_timer:
                self.progress_timer.cancel()
                self.progress_timer = None

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'{rover_name}: Navigation CANCELLED')
            # Don't reset state — switching logic handles this

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                f'❌ {rover_name}: Navigation ABORTED by Nav2 (path failure)'
            )
            # Treat abort as stuck — trigger switch
            if self.assigned_rover == rover_name and self.state == self.STATE_NAVIGATING:
                self._handle_stuck_rover(rover_name)

    # ══════════════════════════════════════════════════════════════════════
    #  PROGRESS MONITORING & STUCK DETECTION
    # ══════════════════════════════════════════════════════════════════════

    def _check_progress(self):
        """Periodic timer callback — detect stuck rover and check re-switch."""
        if self.state == self.STATE_IDLE or self.current_goal is None:
            return

        rover_name = self.assigned_rover
        if rover_name is None:
            return

        rover = self.rovers[rover_name]

        # Check if goal is already reached
        goal_dist = self._distance(rover.position, self.current_goal.pose.position)
        if goal_dist < self.goal_tolerance:
            self.get_logger().info(
                f'{rover_name} is within goal tolerance ({goal_dist:.2f}m). '
                f'Waiting for Nav2 to confirm...'
            )
            return

        # ── Stuck detection ──
        if rover.last_check_position is not None and rover.is_navigating:
            movement = self._distance(rover.position, rover.last_check_position)

            if movement < self.stuck_threshold:
                rover.stuck_count += 1
                self.get_logger().warn(
                    f'{rover_name}: LOW MOVEMENT ({movement:.3f}m) — '
                    f'stuck count: {rover.stuck_count}/{self.stuck_checks_needed}'
                )
                if rover.stuck_count >= self.stuck_checks_needed:
                    self._handle_stuck_rover(rover_name)
                    return
            else:
                rover.stuck_count = 0  # reset on meaningful movement

        # Update last position
        rover.last_check_position = Point(
            x=rover.position.x,
            y=rover.position.y,
            z=rover.position.z,
        )

        # ── Re-switch check (only while backup rover is navigating) ──
        if self.originally_blocked_rover is not None:
            self._check_reswitch()

    def _handle_stuck_rover(self, stuck_rover_name):
        """Switch navigation to the other rover."""
        other_rovers = [n for n in self.rover_names if n != stuck_rover_name]
        if not other_rovers:
            self.get_logger().error('No other rovers available to switch to!')
            return

        other_rover = other_rovers[0]  # For 2-rover system, there's only one alternative

        self.get_logger().warn(
            f'🔄 SWITCHING: {stuck_rover_name} is BLOCKED → '
            f'assigning goal to {other_rover}'
        )

        self.state = self.STATE_SWITCHING
        self.originally_blocked_rover = stuck_rover_name

        # Cancel stuck rover's navigation
        self._cancel_rover(stuck_rover_name)

        # Assign to the other rover
        self._assign_goal(other_rover)

    # ══════════════════════════════════════════════════════════════════════
    #  RE-SWITCH LOGIC
    # ══════════════════════════════════════════════════════════════════════

    def _check_reswitch(self):
        """
        While a backup rover navigates, check if the originally-blocked rover
        can now reach the goal AND is significantly closer (by reswitch_ratio).
        """
        if self.originally_blocked_rover is None:
            return
        if self.current_goal is None:
            return

        blocked = self.originally_blocked_rover
        current = self.assigned_rover

        if blocked == current:
            # Already back on original — no re-switch needed
            self.originally_blocked_rover = None
            return

        goal_pos = self.current_goal.pose.position
        dist_blocked = self._distance(self.rovers[blocked].position, goal_pos)
        dist_current = self._distance(self.rovers[current].position, goal_pos)

        # Only re-switch if blocked rover is significantly closer
        # (dist_blocked < ratio * dist_current)
        if dist_current > 0 and dist_blocked < self.reswitch_ratio * dist_current:
            self.get_logger().info(
                f'🔍 Re-switch check: {blocked} dist={dist_blocked:.2f}m vs '
                f'{current} dist={dist_current:.2f}m (ratio={dist_blocked/dist_current:.2f})'
            )

            # Try to compute a path for the blocked rover to verify it's no longer stuck
            self._try_reswitch_with_path_check(blocked, current)
        else:
            self.get_logger().debug(
                f'Re-switch skipped: {blocked}={dist_blocked:.2f}m, '
                f'{current}={dist_current:.2f}m — '
                f'ratio {dist_blocked/max(dist_current, 0.01):.2f} >= {self.reswitch_ratio}'
            )

    def _try_reswitch_with_path_check(self, blocked_rover, current_rover):
        """
        Use ComputePathToPose to verify the blocked rover actually has a
        clear path now before re-switching.
        """
        client = self.path_clients[blocked_rover]
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                f'ComputePathToPose not available for {blocked_rover}'
            )
            return

        goal = ComputePathToPose.Goal()
        goal.goal = self.current_goal

        # Set start pose to blocked rover's current position
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position = self.rovers[blocked_rover].position
        start.pose.orientation.w = 1.0
        goal.start = start

        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._path_check_result(f, blocked_rover, current_rover)
        )

    def _path_check_result(self, future, blocked_rover, current_rover):
        """Handle result of path feasibility check for re-switch."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info(
                    f'{blocked_rover}: Path check rejected — staying with {current_rover}'
                )
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self._path_check_complete(f, blocked_rover, current_rover)
            )
        except Exception as e:
            self.get_logger().warn(f'Path check failed: {e}')

    def _path_check_complete(self, future, blocked_rover, current_rover):
        """Path computation complete — if valid path exists, do the re-switch."""
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                path = result.result.path
                if len(path.poses) > 0:
                    self.get_logger().info(
                        f'✅ {blocked_rover} has a clear path '
                        f'({len(path.poses)} waypoints). RE-SWITCHING!'
                    )
                    self._execute_reswitch(blocked_rover, current_rover)
                else:
                    self.get_logger().info(
                        f'{blocked_rover}: Empty path returned — no re-switch'
                    )
            else:
                self.get_logger().info(
                    f'{blocked_rover}: Path computation failed — no re-switch'
                )
        except Exception as e:
            self.get_logger().warn(f'Path check result error: {e}')

    def _execute_reswitch(self, blocked_rover, current_rover):
        """Cancel current rover and re-assign to the originally blocked rover."""
        if self.assigned_rover != current_rover:
            return  # state changed since we started the path check

        self.get_logger().warn(
            f'🔄 RE-SWITCHING: {current_rover} → {blocked_rover} '
            f'(path cleared, distance advantage confirmed)'
        )

        # Cancel current rover
        self._cancel_rover(current_rover)

        # Clear the originally_blocked flag since we're going back to it
        self.originally_blocked_rover = None

        # Assign to the now-unblocked rover
        self._assign_goal(blocked_rover)

    # ══════════════════════════════════════════════════════════════════════
    #  UTILITIES
    # ══════════════════════════════════════════════════════════════════════

    def _cancel_rover(self, rover_name):
        """Cancel the navigation goal for a specific rover."""
        rover = self.rovers[rover_name]
        if rover.goal_handle is not None:
            self.get_logger().info(f'Cancelling {rover_name} navigation...')
            cancel_future = rover.goal_handle.cancel_goal_async()
            rover.goal_handle = None
            rover.is_navigating = False
            rover.stuck_count = 0

    def _cancel_all(self):
        """Cancel all active navigation goals."""
        for name in self.rover_names:
            self._cancel_rover(name)
        if self.progress_timer:
            self.progress_timer.cancel()
            self.progress_timer = None

    @staticmethod
    def _distance(p1, p2):
        """Euclidean distance between two Point or position objects."""
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return math.sqrt(dx * dx + dy * dy)


def main():
    rclpy.init()
    node = GoalAssignmentManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
