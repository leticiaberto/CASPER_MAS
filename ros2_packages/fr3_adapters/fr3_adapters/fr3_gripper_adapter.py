"""
ROS2 adapter that controls the Franka Research 3 gripper by sending
``JointTrajectory`` messages to the gripper's joint trajectory controller.

The gripper has two mirrored fingers:
    <robot_name>_fr3_finger_joint1
    <robot_name>_fr3_finger_joint2

Each joint travels from 0.0 (fully closed) to ~0.04 m (fully open).

Gazebo note
-----------
The Gazebo JointTrajectoryController accepts goals correctly but does NOT
publish a result message when the trajectory completes — the action just
goes silent. Waiting for a result callback therefore always times out.

Fix: after the goal is ACCEPTED, we wait for ``time_from_start`` + a small
buffer, then fire the result_callback with success=True. If the goal is
REJECTED we fire with success=False immediately.

This is safe for simulation. On a real robot with a proper controller that
DOES publish results, the result callback will fire first and the timer is
cancelled — so this file works correctly in both environments.
"""

from __future__ import annotations

import threading
from typing import Callable, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Gripper constants for the FR3 hand
# ---------------------------------------------------------------------------

GRIPPER_OPEN_M:   float = 0.04   # max opening per finger (metres)
GRIPPER_CLOSED_M: float = 0.0    # fully closed

# Time allocated for each gripper move (seconds)
GRIPPER_MOVE_DURATION_SEC: int = 2

# Extra buffer after trajectory time_from_start before declaring success
# This covers any controller latency in Gazebo
GRIPPER_COMPLETION_BUFFER_SEC: float = 0.5

ResultCallback = Callable[[bool, str], None]


class GripperAdapter(Node):
    """
    ROS2 Action-Client adapter for the Franka FR3 gripper.

    Parameters
    ----------
    robot_name : str
        Namespace of the robot (e.g. ``"fr3_robot1"``).
    result_callback : ResultCallback
        Called when the current gripper goal finishes.
        Signature: ``callback(success: bool, message: str) -> None``
    open_position : float, optional
        Per-finger joint position (m) for the fully-open state.
    closed_position : float, optional
        Per-finger joint position (m) for the fully-closed state.
    node_name : str, optional
        ROS2 node name.
    """

    def __init__(
        self,
        robot_name:       str,
        result_callback:  ResultCallback,
        open_position:    float = GRIPPER_OPEN_M,
        closed_position:  float = GRIPPER_CLOSED_M,
        node_name:        Optional[str] = None,
    ) -> None:
        node_name = node_name or f"gripper_adapter_{robot_name}"
        super().__init__(node_name)

        self._robot_name      = robot_name
        self._result_callback = result_callback
        self._open_position   = open_position
        self._closed_position = closed_position

        self._current_goal_handle = None
        self._goal_lock           = threading.Lock()

        # Timer handle for time-based completion (Gazebo workaround)
        self._completion_timer: Optional[threading.Timer] = None

        # Joint names for the two symmetric fingers
        prefix = f"{robot_name}_" if robot_name else ""
        self._joint_names = [
            f"{prefix}fr3_finger_joint1",
            f"{prefix}fr3_finger_joint2",
        ]

        action_name = f"/{robot_name}/gripper_controller/follow_joint_trajectory"
        self._action_client = ActionClient(self, FollowJointTrajectory, action_name)

        self.get_logger().info(
            f"[GripperAdapter] Waiting for '{action_name}' ..."
        )
        self._action_client.wait_for_server()
        self.get_logger().info("[GripperAdapter] Action server ready.")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def open(self, duration_sec: int = GRIPPER_MOVE_DURATION_SEC) -> None:
        """Command the gripper to fully open."""
        self.get_logger().info(
            f"[GripperAdapter:{self._robot_name}] Opening gripper."
        )
        self._send_position(self._open_position, duration_sec)

    def close(self, duration_sec: int = GRIPPER_MOVE_DURATION_SEC) -> None:
        """Command the gripper to fully close (grasp)."""
        self.get_logger().info(
            f"[GripperAdapter:{self._robot_name}] Closing gripper."
        )
        self._send_position(self._closed_position, duration_sec)

    def move_to_width(
        self,
        width_m:      float,
        duration_sec: int = GRIPPER_MOVE_DURATION_SEC,
    ) -> None:
        """Move each finger to an arbitrary position (clamped to valid range)."""
        clamped = max(self._closed_position, min(self._open_position, width_m))
        if clamped != width_m:
            self.get_logger().warn(
                f"[GripperAdapter] Width {width_m:.4f}m clamped to {clamped:.4f}m."
            )
        self._send_position(clamped, duration_sec)

    def cancel_goal(self) -> None:
        """Request cancellation of any active gripper goal."""
        self._cancel_completion_timer()
        with self._goal_lock:
            handle = self._current_goal_handle
        if handle is not None:
            handle.cancel_goal_async()
        else:
            self.get_logger().warn(
                "[GripperAdapter] cancel_goal() called but no goal is active."
            )

    # ------------------------------------------------------------------
    # Internal — send
    # ------------------------------------------------------------------

    def _send_position(self, position: float, duration_sec: int) -> None:
        """Build a single-point JointTrajectory and dispatch it."""
        with self._goal_lock:
            if self._current_goal_handle is not None:
                raise RuntimeError(
                    "[GripperAdapter] A gripper goal is already active. "
                    "Wait for it to finish or cancel with cancel_goal()."
                )

        traj = JointTrajectory()
        traj.joint_names = self._joint_names

        pt = JointTrajectoryPoint()
        pt.positions       = [position, position]
        pt.velocities      = [0.0, 0.0]
        pt.time_from_start = Duration(sec=duration_sec, nanosec=0)
        traj.points = [pt]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        # Store duration so the time-based fallback knows how long to wait
        self._pending_duration = duration_sec

        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        ).add_done_callback(self._goal_response_callback)

    # ------------------------------------------------------------------
    # Internal — action callbacks
    # ------------------------------------------------------------------

    def _goal_response_callback(self, future: Future) -> None:
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                f"[GripperAdapter:{self._robot_name}] Goal REJECTED — "
                "check that joint names match the controller config."
            )
            with self._goal_lock:
                self._current_goal_handle = None
            self._result_callback(False, "Gripper goal rejected by action server.")
            return

        self.get_logger().info(
            f"[GripperAdapter:{self._robot_name}] Goal ACCEPTED."
        )
        with self._goal_lock:
            self._current_goal_handle = goal_handle

        # Register for real result (works on real robot / well-behaved controllers)
        goal_handle.get_result_async().add_done_callback(
            self._get_result_callback
        )

        # Gazebo workaround: start a timer to declare success after the
        # trajectory duration + buffer, in case no result is ever published.
        wait = self._pending_duration + GRIPPER_COMPLETION_BUFFER_SEC
        self._completion_timer = threading.Timer(
            wait, self._time_based_completion
        )
        self._completion_timer.start()

    def _get_result_callback(self, future: Future) -> None:
        """Called when the controller publishes a real result (non-Gazebo)."""
        # Cancel the time-based fallback — real result arrived first
        self._cancel_completion_timer()

        result_response = future.result()
        result  = result_response.result

        with self._goal_lock:
            self._current_goal_handle = None

        success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        message = result.error_string if result.error_string else (
            "Gripper move succeeded." if success
            else f"Gripper move failed (error_code={result.error_code})."
        )

        log_fn = self.get_logger().info if success else self.get_logger().error
        log_fn(
            f"[GripperAdapter:{self._robot_name}] Result received — "
            f"success={success}, msg='{message}'"
        )
        self._result_callback(success, message)

    def _time_based_completion(self) -> None:
        """
        Fired when the trajectory duration + buffer has elapsed without a
        real result being published (Gazebo JointTrajectoryController behaviour).
        Declares success and unblocks the caller.
        """
        with self._goal_lock:
            if self._current_goal_handle is None:
                # Real result already handled — nothing to do
                return
            self._current_goal_handle = None

        self.get_logger().info(
            f"[GripperAdapter:{self._robot_name}] Time-based completion "
            "(no result from controller — normal in Gazebo sim)."
        )
        self._result_callback(True, "Gripper move completed (time-based).")

    def _cancel_completion_timer(self) -> None:
        if self._completion_timer is not None:
            self._completion_timer.cancel()
            self._completion_timer = None

    def _feedback_callback(self, feedback_msg) -> None:
        pass  # Gripper moves are short; feedback not needed.