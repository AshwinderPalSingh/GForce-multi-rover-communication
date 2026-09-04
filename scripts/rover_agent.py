#!/usr/bin/env python3
"""
Rover Agent — Distributed Per-Rover Decision-Making Node

Each physical rover runs ONE instance of this node. It handles:
  1. Receiving goal broadcasts from base station
  2. Auction-based task claiming (nearest rover wins)
  3. Self-monitoring for stuck/blocked conditions
  4. Task handoff to peers when blocked
  5. Re-switch evaluation when path clears

Communication Protocol:
  - Each rover publishes its status on /fleet/rover_status (1 Hz)
  - Goal auctions happen via /fleet/task_negotiation
  - Rovers make decisions INDEPENDENTLY using peer status data

This architecture works identically in simulation and on real hardware
(multiple computers on WiFi) because ROS 2 DDS handles discovery.

Parameters:
    rover_id                    (string)  — this rover's unique ID
    auction_timeout             (double)  — seconds to wait for claims before deciding
    progress_check_interval     (double)  — seconds between movement checks
    stuck_distance_threshold    (double)  — meters; below this = stuck
    stuck_consecutive_checks    (int)     — consecutive stuck checks before BLOCKED
    reswitch_distance_ratio     (double)  — re-claim if distance < ratio × active rover's dist
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

import tf2_ros
from tf2_ros import TransformException

from hospital_robot_description.msg import (
    GoalBroadcast,
    TaskNegotiation,
    RoverStatus,
)


class RoverAgent(Node):
    """Autonomous rover agent with distributed task negotiation."""

    # ── States ──
    IDLE = 0
    NAVIGATING = 1
    BLOCKED = 2
    COMPLETED = 3

    def __init__(self):
        super().__init__('rover_agent')
        self.cb_group = ReentrantCallbackGroup()

        # ── Parameters ──
        self.declare_parameter('rover_id', 'rover1')
        self.declare_parameter('auction_timeout', 2.0)
        self.declare_parameter('progress_check_interval', 3.0)
        self.declare_parameter('stuck_distance_threshold', 0.05)
        self.declare_parameter('stuck_consecutive_checks', 3)
        self.declare_parameter('reswitch_distance_ratio', 0.5)
        self.declare_parameter('goal_tolerance', 0.5)
        # Retry guard: without these, two blocked rovers hand the same
        # unreachable goal back and forth forever, each immediately
        # re-claiming a goal it just failed (A blocked -> RELEASE -> B claims
        # -> B blocked -> RELEASE -> A claims -> ...). Nobody ever gives up
        # and the dispatcher never learns the goal is unreachable.
        self.declare_parameter('goal_retry_cooldown', 30.0)
        self.declare_parameter('max_goal_attempts', 2)

        self.rover_id = self.get_parameter('rover_id').get_parameter_value().string_value
        self.auction_timeout = self.get_parameter('auction_timeout').get_parameter_value().double_value
        self.check_interval = self.get_parameter('progress_check_interval').get_parameter_value().double_value
        self.stuck_threshold = self.get_parameter('stuck_distance_threshold').get_parameter_value().double_value
        self.stuck_checks_needed = self.get_parameter('stuck_consecutive_checks').get_parameter_value().integer_value
        self.reswitch_ratio = self.get_parameter('reswitch_distance_ratio').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.retry_cooldown = self.get_parameter('goal_retry_cooldown').get_parameter_value().double_value
        self.max_attempts = self.get_parameter('max_goal_attempts').get_parameter_value().integer_value

        # ── Own state ──
        self.state = self.IDLE
        self.position = Point()          # odom-frame (fallback)
        self.map_position = Point()      # map-frame (used for decisions)
        self.heading = 0.0
        self.last_check_position = None
        self.stuck_count = 0
        self.goal_handle = None         # active Nav2 goal handle
        self.map_position_valid = False  # True once first TF lookup succeeds
        # goal_id -> {'count': int, 'last': float seconds}. Drives the retry
        # guard in _may_claim() so this rover backs off goals it has failed.
        self.failed_goals = {}

        # ── Current goal ──
        self.active_goal_id = ''
        self.active_goal_x = 0.0
        self.active_goal_y = 0.0
        self.active_goal_yaw = 0.0

        # ── Peer state (received via /fleet/rover_status) ──
        self.peer_status = {}  # rover_id -> RoverStatus msg

        # ── Auction state ──
        self.pending_auction_goal_id = ''
        self.auction_claims = {}     # rover_id -> distance
        self.auction_timer = None

        # ═══════════════════════════════════════════════════════════════
        #  SUBSCRIBERS
        # ═══════════════════════════════════════════════════════════════

        # Own odometry
        self.create_subscription(
            Odometry, f'/{self.rover_id}/odom',
            self._odom_callback, 10,
            callback_group=self.cb_group)

        # Fleet communication
        self.create_subscription(
            GoalBroadcast, '/fleet/goal_broadcast',
            self._goal_broadcast_callback, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            TaskNegotiation, '/fleet/task_negotiation',
            self._negotiation_callback, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            RoverStatus, '/fleet/rover_status',
            self._peer_status_callback, 10,
            callback_group=self.cb_group)

        # ═══════════════════════════════════════════════════════════════
        #  PUBLISHERS
        # ═══════════════════════════════════════════════════════════════

        self.status_pub = self.create_publisher(
            RoverStatus, '/fleet/rover_status', 10)

        self.negotiation_pub = self.create_publisher(
            TaskNegotiation, '/fleet/task_negotiation', 10)

        # ═══════════════════════════════════════════════════════════════
        #  NAV2 ACTION CLIENTS (own namespace only)
        # ═══════════════════════════════════════════════════════════════

        self.nav_client = ActionClient(
            self, NavigateToPose,
            f'/{self.rover_id}/navigate_to_pose',
            callback_group=self.cb_group)

        self.path_client = ActionClient(
            self, ComputePathToPose,
            f'/{self.rover_id}/compute_path_to_pose',
            callback_group=self.cb_group)

        # ═══════════════════════════════════════════════════════════════
        #  TF2 — for map-frame position lookup
        # ═══════════════════════════════════════════════════════════════

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ═══════════════════════════════════════════════════════════════
        #  TIMERS
        # ═══════════════════════════════════════════════════════════════

        # Publish own status at 1 Hz (also updates map-frame position)
        self.create_timer(1.0, self._publish_status, callback_group=self.cb_group)

        # Progress monitoring (only active during navigation)
        self.progress_timer = None

        self.get_logger().info(
            f'🤖 Rover Agent [{self.rover_id}] started — '
            f'auction_timeout={self.auction_timeout}s, '
            f'stuck_checks={self.stuck_checks_needed}'
        )

    # ══════════════════════════════════════════════════════════════════
    #  ODOMETRY (fallback position source)
    # ══════════════════════════════════════════════════════════════════

    def _odom_callback(self, msg: Odometry):
        """Update odom-frame position (used as fallback if TF unavailable)."""
        self.position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.heading = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    # ══════════════════════════════════════════════════════════════════
    #  MAP-FRAME POSITION (via TF lookup)
    # ══════════════════════════════════════════════════════════════════

    def _update_map_position(self):
        """
        Look up map → base_link transform to get true map-frame position.
        This is what Nav2/AMCL uses and matches what RViz shows.
        Falls back to odom position if TF is not yet available.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                'map', f'{self.rover_id}/base_link',
                rclpy.time.Time())
            self.map_position.x = t.transform.translation.x
            self.map_position.y = t.transform.translation.y
            self.map_position.z = t.transform.translation.z
            q = t.transform.rotation
            self.heading = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self.map_position_valid = True
        except TransformException:
            # TF not ready yet (AMCL hasn't published map→odom)
            # Fall back to odom position
            self.map_position.x = self.position.x
            self.map_position.y = self.position.y

    def _get_position(self):
        """Return best known position (map-frame if available, else odom)."""
        if self.map_position_valid:
            return self.map_position
        return self.position

    # ══════════════════════════════════════════════════════════════════
    #  STATUS PUBLISHING
    # ══════════════════════════════════════════════════════════════════

    def _publish_status(self):
        """Broadcast own status to fleet at 1 Hz."""
        # Update map-frame position via TF
        self._update_map_position()
        pos = self._get_position()

        msg = RoverStatus()
        msg.rover_id = self.rover_id
        msg.state = self.state
        msg.position_x = pos.x
        msg.position_y = pos.y
        msg.heading = self.heading
        msg.active_goal_id = self.active_goal_id
        msg.battery_level = 100.0  # placeholder for real hardware
        msg.stamp = self.get_clock().now().to_msg()

        if self.active_goal_id:
            msg.distance_to_goal = self._distance_to_goal()
        else:
            msg.distance_to_goal = float('inf')

        self.status_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════
    #  GOAL BROADCAST → AUCTION
    # ══════════════════════════════════════════════════════════════════

    def _goal_broadcast_callback(self, msg: GoalBroadcast):
        """
        New goal broadcast from base station.
        Start an auction: compute own distance, publish CLAIM, wait for peers.
        """
        self.get_logger().info(
            f'📡 Received goal [{msg.goal_id}] at '
            f'({msg.goal_x:.2f}, {msg.goal_y:.2f})'
        )

        # If already navigating to a different goal, cancel first
        if self.state == self.NAVIGATING and self.active_goal_id != msg.goal_id:
            self.get_logger().info('Cancelling current goal for new one')
            self._cancel_navigation()

        # Compute own distance to this goal
        pos = self._get_position()
        my_dist = self._distance(
            pos,
            Point(x=msg.goal_x, y=msg.goal_y, z=0.0),
        )

        # Store pending auction
        self.pending_auction_goal_id = msg.goal_id
        self.auction_claims = {self.rover_id: my_dist}

        # Store goal details
        self.active_goal_x = msg.goal_x
        self.active_goal_y = msg.goal_y
        self.active_goal_yaw = msg.goal_yaw

        # Publish own CLAIM
        claim = TaskNegotiation()
        claim.sender_id = self.rover_id
        claim.target_id = ''  # broadcast
        claim.action = TaskNegotiation.CLAIM
        claim.goal_id = msg.goal_id
        claim.goal_x = msg.goal_x
        claim.goal_y = msg.goal_y
        claim.sender_distance = my_dist
        claim.reason = 'nearest'
        claim.stamp = self.get_clock().now().to_msg()
        self.negotiation_pub.publish(claim)

        self.get_logger().info(
            f'🏁 Published CLAIM for [{msg.goal_id}] — my distance: {my_dist:.2f}m'
        )

        # Start auction timer — resolve after timeout
        if self.auction_timer:
            self.auction_timer.cancel()
        self.auction_timer = self.create_timer(
            self.auction_timeout,
            self._resolve_auction,
            callback_group=self.cb_group,
        )

    # ══════════════════════════════════════════════════════════════════
    #  NEGOTIATION HANDLING
    # ══════════════════════════════════════════════════════════════════

    def _negotiation_callback(self, msg: TaskNegotiation):
        """Handle incoming task negotiation messages from peers."""

        # Ignore own messages
        if msg.sender_id == self.rover_id:
            return

        # ── CLAIM from a peer during auction ──
        if msg.action == TaskNegotiation.CLAIM:
            if msg.goal_id == self.pending_auction_goal_id:
                self.auction_claims[msg.sender_id] = msg.sender_distance
                self.get_logger().info(
                    f'📨 Peer [{msg.sender_id}] CLAIM dist={msg.sender_distance:.2f}m'
                )

        # ── RELEASE from active rover (they're blocked) ──
        elif msg.action == TaskNegotiation.RELEASE:
            if msg.goal_id == self.active_goal_id and self.state == self.IDLE:
                # Peer released the goal — I should try to take it
                self.get_logger().info(
                    f'📨 Peer [{msg.sender_id}] RELEASED goal [{msg.goal_id}] — '
                    f'reason: {msg.reason}. Attempting to claim...'
                )
                self._attempt_claim_released_goal(msg)

            elif msg.goal_id != self.active_goal_id and self.state == self.IDLE:
                # We weren't tracking this goal — claim it now
                self.active_goal_x = msg.goal_x
                self.active_goal_y = msg.goal_y
                self.get_logger().info(
                    f'📨 Peer [{msg.sender_id}] RELEASED goal [{msg.goal_id}] — '
                    f'reason: {msg.reason}. Claiming...'
                )
                self._attempt_claim_released_goal(msg)

        # ── HANDOFF_REQUEST from the originally blocked rover ──
        elif msg.action == TaskNegotiation.HANDOFF_REQUEST:
            if (msg.goal_id == self.active_goal_id and
                    self.state == self.NAVIGATING and
                    msg.target_id == self.rover_id):
                self._handle_handoff_request(msg)

        # ── HANDOFF_ACCEPT — peer accepted our handoff request ──
        elif msg.action == TaskNegotiation.HANDOFF_ACCEPT:
            if (msg.goal_id == self.active_goal_id and
                    msg.target_id == self.rover_id and
                    self.state == self.IDLE):
                # We asked for handoff and got accepted — start navigating.
                # Re-check the retry guard: the request may have been sent
                # before this rover's own failure was recorded.
                if not self._may_claim(msg.goal_id):
                    return
                self.get_logger().info(
                    f'✅ Handoff accepted by [{msg.sender_id}] — starting navigation'
                )
                self._start_navigation()

    def _may_claim(self, goal_id):
        """Retry guard: refuse goals this rover has recently failed.

        Without this, two blocked rovers ping-pong an unreachable goal
        forever. A rover backs off for goal_retry_cooldown seconds after a
        failure, and stops retrying entirely after max_goal_attempts. When
        every rover has given up the goal simply goes unclaimed, which the
        dispatcher can see (no rover reports NAVIGATING it).
        """
        rec = self.failed_goals.get(goal_id)
        if not rec:
            return True

        if rec['count'] >= self.max_attempts:
            self.get_logger().info(
                f'⛔ Not re-claiming [{goal_id}] — already failed '
                f'{rec["count"]}/{self.max_attempts} attempts'
            )
            return False

        elapsed = self.get_clock().now().nanoseconds / 1e9 - rec['last']
        if elapsed < self.retry_cooldown:
            self.get_logger().info(
                f'⏳ Not re-claiming [{goal_id}] yet — {elapsed:.0f}s since '
                f'failure, cooldown {self.retry_cooldown:.0f}s'
            )
            return False

        return True

    def _attempt_claim_released_goal(self, release_msg: TaskNegotiation):
        """Try to claim a goal that another rover released."""
        if not self._may_claim(release_msg.goal_id):
            return

        my_dist = self._distance(
            self._get_position(),
            Point(x=release_msg.goal_x, y=release_msg.goal_y, z=0.0),
        )

        self.active_goal_id = release_msg.goal_id
        self.active_goal_x = release_msg.goal_x
        self.active_goal_y = release_msg.goal_y

        # Publish claim
        claim = TaskNegotiation()
        claim.sender_id = self.rover_id
        claim.target_id = ''
        claim.action = TaskNegotiation.CLAIM
        claim.goal_id = release_msg.goal_id
        claim.goal_x = release_msg.goal_x
        claim.goal_y = release_msg.goal_y
        claim.sender_distance = my_dist
        claim.reason = 'peer_blocked'
        claim.stamp = self.get_clock().now().to_msg()
        self.negotiation_pub.publish(claim)

        # Start navigation immediately (in a 2-rover system, we're the only option)
        self._start_navigation()

    def _handle_handoff_request(self, msg: TaskNegotiation):
        """
        A blocked peer wants to take back the goal because their path cleared.
        Accept if they're significantly closer (reswitch_ratio).
        """
        my_dist = self._distance_to_goal()
        peer_dist = msg.sender_distance

        response = TaskNegotiation()
        response.sender_id = self.rover_id
        response.target_id = msg.sender_id
        response.goal_id = msg.goal_id
        response.goal_x = msg.goal_x
        response.goal_y = msg.goal_y
        response.sender_distance = my_dist
        response.stamp = self.get_clock().now().to_msg()

        if peer_dist < self.reswitch_ratio * my_dist:
            self.get_logger().info(
                f'🔄 HANDOFF ACCEPTED: [{msg.sender_id}] dist={peer_dist:.2f}m '
                f'vs my dist={my_dist:.2f}m (ratio={peer_dist/my_dist:.2f} < {self.reswitch_ratio})'
            )
            response.action = TaskNegotiation.HANDOFF_ACCEPT
            response.reason = 'closer'
            self.negotiation_pub.publish(response)

            # Cancel own navigation and go idle
            self._cancel_navigation()
            self.state = self.IDLE
            self.active_goal_id = ''
        else:
            self.get_logger().info(
                f'❌ HANDOFF REJECTED: [{msg.sender_id}] dist={peer_dist:.2f}m '
                f'vs my dist={my_dist:.2f}m — not close enough'
            )
            response.action = TaskNegotiation.HANDOFF_REJECT
            response.reason = 'not_close_enough'
            self.negotiation_pub.publish(response)

    # ══════════════════════════════════════════════════════════════════
    #  AUCTION RESOLUTION
    # ══════════════════════════════════════════════════════════════════

    def _resolve_auction(self):
        """
        Called after auction_timeout. All claims collected.
        Winner = rover with smallest distance. Tie-break: lexically smaller ID.
        """
        if self.auction_timer:
            self.auction_timer.cancel()
            self.auction_timer = None

        goal_id = self.pending_auction_goal_id
        if not goal_id:
            return

        # Find winner
        winner = None
        best_dist = float('inf')
        for rover_id, dist in sorted(self.auction_claims.items()):
            if dist < best_dist or (dist == best_dist and rover_id < winner):
                best_dist = dist
                winner = rover_id

        self.get_logger().info(
            f'🏆 Auction [{goal_id}] resolved: winner={winner} '
            f'(dist={best_dist:.2f}m) claims={self.auction_claims}'
        )

        if winner == self.rover_id:
            # I won — start navigating
            self.active_goal_id = goal_id
            self._start_navigation()
        else:
            # I lost — go idle, keep goal info for potential future handoff
            self.active_goal_id = goal_id  # track it for RELEASE handling
            self.state = self.IDLE
            self.get_logger().info(
                f'Standing by — [{winner}] is handling goal [{goal_id}]'
            )

        self.pending_auction_goal_id = ''
        self.auction_claims = {}

    # ══════════════════════════════════════════════════════════════════
    #  NAVIGATION
    # ══════════════════════════════════════════════════════════════════

    def _start_navigation(self):
        """Send NavigateToPose to own Nav2 stack."""
        self.state = self.NAVIGATING
        self.stuck_count = 0
        pos = self._get_position()
        self.last_check_position = Point(
            x=pos.x, y=pos.y, z=0.0)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.active_goal_x
        goal.pose.pose.position.y = self.active_goal_y
        goal.pose.pose.orientation.z = math.sin(self.active_goal_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.active_goal_yaw / 2.0)

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            self.state = self.IDLE
            return

        self.get_logger().info(
            f'🚀 Navigating to ({self.active_goal_x:.2f}, {self.active_goal_y:.2f})'
        )

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback,
        )
        future.add_done_callback(self._goal_accepted_callback)

        # Start progress timer
        if self.progress_timer:
            self.progress_timer.cancel()
        self.progress_timer = self.create_timer(
            self.check_interval, self._check_progress,
            callback_group=self.cb_group)

    def _goal_accepted_callback(self, future):
        """Nav2 accepted or rejected the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal REJECTED by Nav2')
            self.state = self.IDLE
            return

        self.get_logger().info('Goal ACCEPTED by Nav2')
        self.goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback(self, feedback_msg):
        """Nav2 feedback — position tracking done via odom."""
        pass

    def _nav_result_callback(self, future):
        """Nav2 navigation completed."""
        result = future.result()
        status = result.status

        self.goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✅ Goal [{self.active_goal_id}] REACHED!')
            # Clear any prior failure record so the goal isn't left blacklisted
            self.failed_goals.pop(self.active_goal_id, None)
            self.state = self.COMPLETED
            self._publish_status()  # immediately broadcast completion
            self.state = self.IDLE
            self.active_goal_id = ''
            if self.progress_timer:
                self.progress_timer.cancel()
                self.progress_timer = None

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation CANCELLED')
            # State is managed by whoever cancelled

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Navigation ABORTED by Nav2')
            if self.state == self.NAVIGATING:
                self._handle_blocked()

    # ══════════════════════════════════════════════════════════════════
    #  PROGRESS MONITORING & STUCK DETECTION
    # ══════════════════════════════════════════════════════════════════

    def _check_progress(self):
        """Periodic self-monitoring — detect if this rover is stuck."""
        if self.state != self.NAVIGATING:
            return

        # Check if goal reached
        goal_dist = self._distance_to_goal()
        if goal_dist < self.goal_tolerance:
            self.get_logger().info(f'Within goal tolerance ({goal_dist:.2f}m)')
            return

        # Stuck detection
        if self.last_check_position is not None:
            movement = self._distance(self._get_position(), self.last_check_position)

            if movement < self.stuck_threshold:
                self.stuck_count += 1
                self.get_logger().warn(
                    f'LOW MOVEMENT ({movement:.3f}m) — '
                    f'stuck count: {self.stuck_count}/{self.stuck_checks_needed}'
                )
                if self.stuck_count >= self.stuck_checks_needed:
                    self._handle_blocked()
                    return
            else:
                self.stuck_count = 0

        pos = self._get_position()
        self.last_check_position = Point(
            x=pos.x, y=pos.y, z=0.0)

        # ── Re-switch check: if we took over from a blocked peer, ──
        # ── check if that peer is now closer and has a clear path  ──
        self._check_peer_reswitch()

    def _handle_blocked(self):
        """
        This rover is blocked. Release the goal so a peer can take over.
        This is the KEY communication moment — actual inter-rover messaging.
        """
        self.get_logger().warn(
            f'🚫 BLOCKED on goal [{self.active_goal_id}] — '
            f'releasing to fleet'
        )

        self.state = self.BLOCKED
        self._cancel_navigation()

        # Record the failure so we don't immediately re-claim this same goal
        # when a peer releases it back to the fleet (see _may_claim).
        if self.active_goal_id:
            rec = self.failed_goals.setdefault(
                self.active_goal_id, {'count': 0, 'last': 0.0})
            rec['count'] += 1
            rec['last'] = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().warn(
                f'   attempt {rec["count"]}/{self.max_attempts} failed on '
                f'[{self.active_goal_id}]; backing off {self.retry_cooldown:.0f}s'
            )

        # Publish RELEASE message to fleet
        release = TaskNegotiation()
        release.sender_id = self.rover_id
        release.target_id = ''  # broadcast
        release.action = TaskNegotiation.RELEASE
        release.goal_id = self.active_goal_id
        release.goal_x = self.active_goal_x
        release.goal_y = self.active_goal_y
        release.sender_distance = self._distance_to_goal()
        release.reason = 'blocked'
        release.stamp = self.get_clock().now().to_msg()
        self.negotiation_pub.publish(release)

        # Go idle — keep tracking the goal for potential re-claim
        self.state = self.IDLE

    def _check_peer_reswitch(self):
        """
        While navigating, check if any idle peer (who was previously blocked)
        now has a significantly shorter distance. If so, propose handoff.
        """
        my_dist = self._distance_to_goal()

        for peer_id, peer in self.peer_status.items():
            if peer_id == self.rover_id:
                continue
            if peer.state != self.IDLE:
                continue
            if peer.active_goal_id != self.active_goal_id:
                continue

            peer_dist = self._distance(
                Point(x=peer.position_x, y=peer.position_y, z=0.0),
                Point(x=self.active_goal_x, y=self.active_goal_y, z=0.0),
            )

            if my_dist > 0 and peer_dist < self.reswitch_ratio * my_dist:
                self.get_logger().info(
                    f'🔍 Peer [{peer_id}] is significantly closer '
                    f'({peer_dist:.2f}m vs {my_dist:.2f}m) — '
                    f'checking if they want to reclaim...'
                )
                # The peer will detect this via their own monitoring
                # and send a HANDOFF_REQUEST if they have a clear path

    # ══════════════════════════════════════════════════════════════════
    #  PEER STATUS
    # ══════════════════════════════════════════════════════════════════

    def _peer_status_callback(self, msg: RoverStatus):
        """Store peer status for negotiation decisions."""
        if msg.rover_id == self.rover_id:
            return  # ignore own status
        self.peer_status[msg.rover_id] = msg

        # If we're IDLE and were previously blocked, check if we should
        # try to re-claim the goal via HANDOFF_REQUEST
        if (self.state == self.IDLE and
                self.active_goal_id and
                msg.active_goal_id == self.active_goal_id and
                msg.state == self.NAVIGATING):
            self._consider_handoff_request(msg)

    def _consider_handoff_request(self, active_peer: RoverStatus):
        """
        We're idle (probably were blocked), peer is navigating our goal.
        Check if we're now significantly closer and have a clear path.
        """
        # Same retry guard as the claim path. Without this the rover can
        # re-enter a goal it already failed by asking the active peer for a
        # handoff, which bypasses _attempt_claim_released_goal entirely and
        # re-creates the infinite ping-pong.
        if not self._may_claim(self.active_goal_id):
            return

        my_dist = self._distance_to_goal()
        peer_dist = active_peer.distance_to_goal

        if peer_dist > 0 and my_dist < self.reswitch_ratio * peer_dist:
            # Check if we have a viable path
            self._verify_path_and_request_handoff(active_peer.rover_id, my_dist)

    def _verify_path_and_request_handoff(self, peer_id, my_dist):
        """Use ComputePathToPose to verify we have a clear path before requesting handoff."""
        if not self.path_client.wait_for_server(timeout_sec=2.0):
            return

        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = 'map'
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = self.active_goal_x
        goal.goal.pose.position.y = self.active_goal_y
        goal.goal.pose.orientation.w = 1.0

        goal.start = PoseStamped()
        goal.start.header.frame_id = 'map'
        goal.start.header.stamp = self.get_clock().now().to_msg()
        pos = self._get_position()
        goal.start.pose.position.x = pos.x
        goal.start.pose.position.y = pos.y
        goal.start.pose.orientation.w = 1.0

        future = goal_handle_future = self.path_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._path_check_done(f, peer_id, my_dist))

    def _path_check_done(self, future, peer_id, my_dist):
        """Path computation done — if valid, send HANDOFF_REQUEST."""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self._path_result(f, peer_id, my_dist))
        except Exception:
            pass

    def _path_result(self, future, peer_id, my_dist):
        """Path result received — send handoff request if path exists."""
        try:
            result = future.result()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                return
            path = result.result.path
            if len(path.poses) == 0:
                return

            self.get_logger().info(
                f'✅ Path clear ({len(path.poses)} waypoints) — '
                f'requesting handoff from [{peer_id}]'
            )

            msg = TaskNegotiation()
            msg.sender_id = self.rover_id
            msg.target_id = peer_id
            msg.action = TaskNegotiation.HANDOFF_REQUEST
            msg.goal_id = self.active_goal_id
            msg.goal_x = self.active_goal_x
            msg.goal_y = self.active_goal_y
            msg.sender_distance = my_dist
            msg.reason = 'path_cleared'
            msg.stamp = self.get_clock().now().to_msg()
            self.negotiation_pub.publish(msg)

        except Exception:
            pass

    # ══════════════════════════════════════════════════════════════════
    #  UTILITIES
    # ══════════════════════════════════════════════════════════════════

    def _distance_to_goal(self):
        """Distance from own position to active goal."""
        if not self.active_goal_id:
            return float('inf')
        return self._distance(
            self._get_position(),
            Point(x=self.active_goal_x, y=self.active_goal_y, z=0.0),
        )

    def _cancel_navigation(self):
        """Cancel own Nav2 goal."""
        if self.goal_handle is not None:
            self.get_logger().info('Cancelling navigation...')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.stuck_count = 0
        if self.progress_timer:
            self.progress_timer.cancel()
            self.progress_timer = None

    @staticmethod
    def _distance(p1, p2):
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        return math.sqrt(dx * dx + dy * dy)


def main():
    rclpy.init()
    node = RoverAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel_navigation()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
