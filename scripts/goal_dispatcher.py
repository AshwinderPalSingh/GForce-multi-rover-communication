#!/usr/bin/env python3
"""
Goal Dispatcher — Lightweight Base Station Node

Runs on the base station (laptop). Its ONLY job is to:
  1. Listen for goals from RViz (/goal_pose)
  2. Assign a UUID and broadcast to all rovers via /fleet/goal_broadcast
  3. Monitor and log which rover claims each goal (observational only)

It does NOT make navigation decisions — those are made by each rover's
rover_agent independently via the auction protocol.

Topics:
  Subscribes:
    /goal_pose                (PoseStamped)       — from RViz
    /fleet/task_negotiation   (TaskNegotiation)   — observe claims
    /fleet/rover_status       (RoverStatus)       — observe fleet state
  Publishes:
    /fleet/goal_broadcast     (GoalBroadcast)     — broadcast to fleet
"""

import uuid
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from hospital_robot_description.msg import (
    GoalBroadcast,
    TaskNegotiation,
    RoverStatus,
)


class GoalDispatcher(Node):

    def __init__(self):
        super().__init__('goal_dispatcher')

        # ── Publishers ──
        self.goal_pub = self.create_publisher(
            GoalBroadcast, '/fleet/goal_broadcast', 10)

        # ── Subscribers ──
        self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_callback, 10)

        self.create_subscription(
            TaskNegotiation, '/fleet/task_negotiation',
            self._negotiation_callback, 10)

        self.create_subscription(
            RoverStatus, '/fleet/rover_status',
            self._status_callback, 10)

        # ── Fleet state (observational logging only) ──
        self.fleet_status = {}  # rover_id -> last RoverStatus
        self.active_goals = {}  # goal_id -> {goal_x, goal_y, claimed_by}

        self.get_logger().info('Goal Dispatcher started — waiting for goals')

    def _goal_callback(self, msg: PoseStamped):
        """New goal from RViz → broadcast to fleet with UUID."""
        goal_id = str(uuid.uuid4())[:8]  # short UUID for readability

        # Extract yaw from quaternion
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        broadcast = GoalBroadcast()
        broadcast.goal_id = goal_id
        broadcast.goal_x = msg.pose.position.x
        broadcast.goal_y = msg.pose.position.y
        broadcast.goal_yaw = yaw
        broadcast.stamp = self.get_clock().now().to_msg()

        self.goal_pub.publish(broadcast)

        self.active_goals[goal_id] = {
            'goal_x': msg.pose.position.x,
            'goal_y': msg.pose.position.y,
            'claimed_by': None,
        }

        self.get_logger().info(
            f'📡 BROADCAST goal [{goal_id}] at '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

    def _negotiation_callback(self, msg: TaskNegotiation):
        """Log task negotiation events for monitoring."""
        action_names = {
            TaskNegotiation.CLAIM: 'CLAIM',
            TaskNegotiation.RELEASE: 'RELEASE',
            TaskNegotiation.HANDOFF_REQUEST: 'HANDOFF_REQUEST',
            TaskNegotiation.HANDOFF_ACCEPT: 'HANDOFF_ACCEPT',
            TaskNegotiation.HANDOFF_REJECT: 'HANDOFF_REJECT',
        }
        action_str = action_names.get(msg.action, f'UNKNOWN({msg.action})')

        self.get_logger().info(
            f'📋 [{msg.sender_id}] {action_str} goal [{msg.goal_id}] '
            f'dist={msg.sender_distance:.2f}m reason="{msg.reason}"'
        )

        # Track claims
        if msg.action == TaskNegotiation.CLAIM:
            if msg.goal_id in self.active_goals:
                self.active_goals[msg.goal_id]['claimed_by'] = msg.sender_id

    def _status_callback(self, msg: RoverStatus):
        """Update fleet state for logging."""
        state_names = {0: 'IDLE', 1: 'NAVIGATING', 2: 'BLOCKED', 3: 'COMPLETED'}
        state_str = state_names.get(msg.state, 'UNKNOWN')

        # Only log on state change
        prev = self.fleet_status.get(msg.rover_id)
        if prev is None or prev.state != msg.state:
            self.get_logger().info(
                f'🤖 {msg.rover_id}: state={state_str} '
                f'pos=({msg.position_x:.2f}, {msg.position_y:.2f}) '
                f'dist_to_goal={msg.distance_to_goal:.2f}m'
            )

        self.fleet_status[msg.rover_id] = msg


def main():
    rclpy.init()
    node = GoalDispatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
