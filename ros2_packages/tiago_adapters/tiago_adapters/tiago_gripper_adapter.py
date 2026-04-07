"""
ROS2 adapter that controls the Tiago PAL gripper by sending
JointTrajectory messages to the gripper's joint trajectory controller.

The PAL gripper has two parallel finger joints:
    gripper_left_finger_joint
    gripper_right_finger_joint

Each joint travels from ~0.002 m (closed) to ~0.044 m (fully open).
Joint names are NOT prefixed with the robot namespace — this is the PAL
convention, unlike Franka which prefixes all joint names.

Gazebo note
-----------
The Gazebo JointTrajectoryController accepts goals correctly but does NOT
publish a result message when the trajectory completes — the action goes
silent. Waiting for a result callback therefore always times out.

Fix: after the goal is ACCEPTED, we wait for time_from_start + a small
buffer, then fire result_callback(True). If the goal is REJECTED we fire
result_callback(False) immediately.

On a real robot with a controller that publishes results, the real result
arrives first and the timer is cancelled — so this works in both environments.
"""

from __future__ import annotations

import threading
from typing import Callable, Optional

import rclpy
import rclpy.parameter
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# PAL gripper constants
# ---------------------------------------------------------------------------

GRIPPER_OPEN_M:   float = 0.044   # max opening per finger (metres)
GRIPPER_CLOSED_M: float = 0.002   # closed (small positive to avoid singularity)

GRIPPER_MOVE_DURATION_SEC:      int   = 2
GRIPPER_COMPLETION_BUFFER_SEC: float  = 0.5

ResultCallback = Callable[[bool, str], None]

# Joint names — PAL convention: no robot-namespace prefix
_GRIPPER_JOINTS = [
    'gripper_left_finger_joint',
    'gripper_right_finger_joint',
]


class TiagoGripperAdapter(Node):
    """
    ROS2 Action-Client adapter for the Tiago PAL gripper.

    Parameters
    ----------
    robot_name : str
        Namespace of the robot (e.g. "tiago_robot1").
        Used only for the action topic path, not joint name prefixing.
    result_callback : ResultCallback
        Called when the current gripper goal finishes.
        Signature: callback(success: bool, message: str) -> None
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
        node_name = node_name or f"tiago_gripper_adapter_{robot_name}"
        super().__init__(
            node_name,
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.parameter.Parameter.Type.BOOL,
                    True,
                )
            ],
        )

        self._robot_name      = robot_name
        self._result_callback = result_callback
        self._open_position   = open_position
        self._closed_position = closed_position

        self._current_goal_handle = None
        self._goal_lock           = threading.Lock()
        self._completion_timer: Optional[threading.Timer] = None

        action_name = f"/{robot_name}/gripper_controller/follow_joint_trajectory"
        self._action_client = ActionClient(self, FollowJointTrajectory, action_name)

        self.get_logger().info(
            f"[TiagoGripperAdapter] Waiting for '{action_name}' ..."
        )
        self._action_client.wait_for_server()
        self.get_logger().info("[TiagoGripperAdapter] Gripper server ready.")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def open(self, duration_sec: int = GRIPPER_MOVE_DURATION_SEC) -> None:
        """Command the gripper to fully open."""
        self.get_logger().info(
            f"[TiagoGripperAdapter:{self._robot_name}] Opening gripper."
        )
        self._send_position(self._open_position, duration_sec)

    def close(self, duration_sec: int = GRIPPER_MOVE_DURATION_SEC) -> None:
        """Command the gripper to fully close (grasp)."""
        self.get_logger().info(
            f"[TiagoGripperAdapter:{self._robot_name}] Closing gripper."
        )
        self._send_position(self._closed_position, duration_sec)

    def cancel_goal(self) -> None:
        """Request cancellation of any active gripper goal."""
        self._cancel_completion_timer()
        with self._goal_lock:
            handle = self._current_goal_handle
        if handle is not None:
            handle.cancel_goal_async()
        else:
            self.get_logger().warn(
                "[TiagoGripperAdapter] cancel_goal() called but no goal is active."
            )

    # ------------------------------------------------------------------
    # Internal — send
    # ------------------------------------------------------------------

    def _send_position(self, position: float, duration_sec: int) -> None:
        with self._goal_lock:
            if self._current_goal_handle is not None:
                raise RuntimeError(
                    "[TiagoGripperAdapter] A gripper goal is already active. "
                    "Wait for it to finish or call cancel_goal() first."
                )

        traj = JointTrajectory()
        traj.joint_names = _GRIPPER_JOINTS

        pt = JointTrajectoryPoint()
        pt.positions       = [position, position]
        pt.velocities      = [0.0, 0.0]
        pt.time_from_start = Duration(sec=duration_sec, nanosec=0)
        traj.points = [pt]

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj
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
                f"[TiagoGripperAdapter:{self._robot_name}] Goal REJECTED."
            )
            with self._goal_lock:
                self._current_goal_handle = None
            self._result_callback(False, "Gripper goal rejected by action server.")
            return

        self.get_logger().info(
            f"[TiagoGripperAdapter:{self._robot_name}] Goal ACCEPTED."
        )
        with self._goal_lock:
            self._current_goal_handle = goal_handle

        goal_handle.get_result_async().add_done_callback(self._get_result_callback)

        # Gazebo workaround: declare success after duration + buffer
        # if no real result is ever published.
        wait = self._pending_duration + GRIPPER_COMPLETION_BUFFER_SEC
        self._completion_timer = threading.Timer(wait, self._time_based_completion)
        self._completion_timer.start()

    def _get_result_callback(self, future: Future) -> None:
        """Real result arrived (non-Gazebo controller). Cancel the timer."""
        self._cancel_completion_timer()

        result_response = future.result()
        result = result_response.result

        with self._goal_lock:
            self._current_goal_handle = None

        success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        message = result.error_string if result.error_string else (
            "Gripper move succeeded." if success
            else f"Gripper move failed (error_code={result.error_code})."
        )
        log_fn = self.get_logger().info if success else self.get_logger().error
        log_fn(
            f"[TiagoGripperAdapter:{self._robot_name}] Result: "
            f"success={success}, msg='{message}'"
        )
        self._result_callback(success, message)

    def _time_based_completion(self) -> None:
        """
        Fired when trajectory duration + buffer elapsed without a real result
        (expected in Gazebo's JointTrajectoryController).
        """
        with self._goal_lock:
            if self._current_goal_handle is None:
                return
            self._current_goal_handle = None

        self.get_logger().info(
            f"[TiagoGripperAdapter:{self._robot_name}] Time-based completion "
            "(no result from controller — normal in Gazebo)."
        )
        self._result_callback(True, "Gripper move completed (time-based).")

    def _cancel_completion_timer(self) -> None:
        if self._completion_timer is not None:
            self._completion_timer.cancel()
            self._completion_timer = None

    def _feedback_callback(self, feedback_msg) -> None:
        pass  # Gripper moves are short; feedback not needed.
