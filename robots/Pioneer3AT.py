"""
Pioneer3AT
==========
High-level agent for a Pioneer 3AT.

One instance is created per spawned Pioneer robot. It resolves navigation
targets the same way Tiago does — by named location, by Gazebo model name,
or by an explicit coordinate — and drives there using PioneerAdapter's
lightweight go-to-pose P-controller (rotate-to-bearing, drive, settle
heading). There's no path planner or costmap here on purpose: it's a
straight-line controller, which is enough for open Gazebo scenes and keeps
the whole stack dependency-free.

Pose source
-----------
Ground-truth world pose is read live from Gazebo Fortress via the gz-ros2
bridge topic ``/world/<world_name>/pose/info`` (same as Tiago) when
``world_name`` is supplied — this avoids odometry drift. If omitted, the
adapter falls back to wheel odometry from the bridged ``/<robot_name>/odom``
topic (see pioneer3at_adapters launch file).

Prerequisite: the gz-ros2 bridge must be running::

    ros2 run ros_gz_bridge parameter_bridge \\
      /world/<world_name>/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \\
      /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

Usage
-----
    pioneer = Pioneer3AT(
        robot_name = "pioneer3at_1",
        world_name = "backyard",
    )

    # Navigate by location name (from the locations dict):
    pioneer.navigate_to("DiningTable")

    # Navigate by Gazebo model name (resolved via ObjectToRobot):
    pioneer.navigate_to("prep_table_1")

    # Navigate by explicit world-frame coordinate (x, y) or (x, y, theta):
    pioneer.navigate_to((-0.77, -1.54))
    pioneer.navigate_to((-0.77, -1.54, 1.57))
"""

from __future__ import annotations

import threading
import time
from typing import List, Optional, Tuple, Union

import rclpy
from rclpy.executors import MultiThreadedExecutor

from src.entities.Agent import Agent
from pioneer3at_adapters.PioneerAdapter import PioneerAdapter, RobotMode
from robot_common.object_world_to_robot import ObjectToRobot

from locations import get_locations

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

XY         = Tuple[float, float]
XYTheta    = Tuple[float, float, float]
NameOrXY   = Union[str, XY, XYTheta]


class Pioneer3AT(Agent):
    """
    Parameters
    ----------
    robot_name : str
        Agent identifier and Gazebo / ROS2 namespace (e.g. "pioneer3at_1").
        Must match the ``-name`` the robot was spawned with.
    world_name : str
        Gazebo world name that matches the gz-ros2-bridge command, e.g.
        "backyard". Must match ``<world name="…">`` in the .sdf AND the
        bridge topic /world/<world_name>/pose/info.
    skill_weights, contexts, role, teamsize :
        Passed through to the Agent base class.
    pose_timeout : float
        Seconds to wait for the first pose message on startup. Default 15.
    nav_timeout : float
        Default seconds to wait for a navigation goal to complete
        (overridable per-call). Default 120.
    """

    def __init__(
        self,
        robot_name:     str,
        world_name:     str,
        constraints:    dict            = None,
        skill_weights:  dict            = None,
        contexts:       List[str]       = None,
        role:           str             = "member",
        teamsize:       int             = 1,
        run_id:         str             = "test",
        use_sim:        bool            = True,
        workspace:      List[str]       = None,
        party_duration: float           = 3600.0,
        guests:         int             = 0,
        pose_timeout:   float           = 15.0,
        nav_timeout:    float           = 120.0,
    ) -> None:

        super().__init__(
            robot_name,
            constraints or {},
            workspace or [],
            skill_weights or {},
            contexts or [],
            role,
            teamsize,
            party_duration,
            guests,
            run_id,
        )

        mode = RobotMode.SIMULATION if use_sim else RobotMode.REAL_WORLD

        self._robot_name  = robot_name
        self._world_name  = world_name
        self._locations   = get_locations(world_name)
        self._nav_timeout = nav_timeout

        self._nav_done_event = threading.Event()
        self._nav_result: Tuple[bool, str] = (False, "No nav goal sent yet.")

        # ── ROS2 initialisation ───────────────────────────────────────
        if not rclpy.ok():
            rclpy.init()

        # world_name activates ground-truth pose (/world/<world_name>/pose/info);
        # no spawn-pose parameter is needed regardless of where the robot
        # was placed in the world.
        self._adapter = PioneerAdapter(
            robot_name      = robot_name,
            mode            = mode,
            result_callback = self._on_nav_result,
            world_name      = world_name,
        )

        # Shares the adapter node so all subscriptions live on the same
        # executor. Both OTR and the adapter subscribe to
        # /world/<world_name>/pose/info; ROS2 delivers to each callback
        # independently — no conflict.
        self._otr = ObjectToRobot(
            node         = self._adapter,
            robot_name   = robot_name,
            world_name   = world_name,
            pose_timeout = pose_timeout,
        )

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._adapter)
        self._spin_thread = threading.Thread(
            target = self._executor.spin,
            daemon = True,
            name   = f"pioneer_{robot_name}_spin",
        )
        self._spin_thread.start()

        print(f"[Pioneer3AT] Waiting for pose from /world/{world_name}/pose/info ...")
        if self._adapter._pose_ready.wait(timeout=pose_timeout):
            src = "ground-truth" if self._adapter._use_ground_truth else "odometry"
            x, y, th = self._adapter._get_robot_pose()
            print(f"[Pioneer3AT] Pose active ({src}): ({x:.3f}, {y:.3f}, {th:.3f} rad).")
        else:
            print(f"[Pioneer3AT] WARNING: no pose received within {pose_timeout:.0f}s. "
                  "Check the gz-ros2 bridge is running.")

        self._adapter.get_logger().info(
            f"[Pioneer3AT] Agent '{robot_name}' ready. world='{world_name}'"
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def navigate_to(
        self,
        target:  NameOrXY,
        timeout: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """
        Drive the robot to a named location, a Gazebo model name, or an
        explicit world-frame coordinate.

        Parameters
        ----------
        target : str or (x, y) or (x, y, theta)
            * ``str`` — key in the ``locations`` dict (e.g. "DiningTable"),
              or a Gazebo model name resolved via ObjectToRobot. Named
              locations are tried first; if not found there, the name is
              looked up in Gazebo.
            * ``(x, y)`` — explicit world-frame position; final heading
              is left unconstrained.
            * ``(x, y, theta)`` — explicit world-frame position + a final
              heading (rad) to settle into on arrival.
        timeout : float, optional
            Override the instance-level ``nav_timeout``.

        Returns
        -------
        (success, message)
        """
        # ── Resolve target (x, y[, theta]) ──────────────────────────────
        theta: Optional[float] = None

        if isinstance(target, str):
            if target in self._locations:
                loc = self._locations[target]
                tx, ty = loc["x"], loc["y"]
                theta = loc.get("theta")
                self._adapter.get_logger().info(
                    f"[Pioneer3AT] navigate_to: '{target}' → ({tx:.3f}, {ty:.3f}) [locations dict]"
                )
            else:
                result = self._otr.get_pose(target)
                if "error" in result:
                    return False, f"Cannot resolve navigate target '{target}': {result['error']}"
                pos = result["world_pose"].position
                tx, ty = pos.x, pos.y
                self._adapter.get_logger().info(
                    f"[Pioneer3AT] navigate_to: '{target}' → ({tx:.3f}, {ty:.3f}) [Gazebo OTR]"
                )
        elif len(target) == 3:
            tx, ty, theta = target
            self._adapter.get_logger().info(
                f"[Pioneer3AT] navigate_to: XYTheta=({tx:.3f}, {ty:.3f}, {theta:.3f})"
            )
        else:
            tx, ty = target
            self._adapter.get_logger().info(
                f"[Pioneer3AT] navigate_to: XY=({tx:.3f}, {ty:.3f})"
            )

        # ── Dispatch via PioneerAdapter ──────────────────────────────────
        self._nav_done_event.clear()
        deadline = timeout if timeout is not None else self._nav_timeout

        accepted = self._adapter.navigate_to(tx, ty, theta=theta, timeout=deadline)
        if not accepted:
            msg = "[Pioneer3AT] Adapter rejected goal (already busy)."
            self._adapter.get_logger().error(msg)
            return False, msg

        finished = self._nav_done_event.wait(timeout=deadline + 5.0)
        if not finished:
            self._adapter.cancel()
            msg = f"[Pioneer3AT] Navigation timed out after {deadline:.0f}s."
            self._adapter.get_logger().error(msg)
            return False, msg

        return self._nav_result

    def shutdown(self) -> None:
        """Cleanly shut down ROS2 executor and node."""
        self._adapter.get_logger().info("[Pioneer3AT] Shutting down ...")
        self._executor.shutdown(timeout_sec=3.0)
        self._spin_thread.join(timeout=3.0)
        try:
            self._adapter.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        print("[Pioneer3AT] Shutdown complete.")

    # ------------------------------------------------------------------
    # Agent interface (called by base class task dispatch)
    # ------------------------------------------------------------------

    def _execute_task_specific(self, task: dict) -> None:
        """
        Supports the same subtask-sequence convention as Tiago's dispatcher,
        limited to what a mobile-only robot can do:

            {"action": "navigate_to", "target_name": "..."}       or
            {"action": "navigate_to", "target_xyz": (x, y[, th])}
            {"action": "sleep", "time": seconds}
        """
        action = task.get("action")
        if action == "navigate_to":
            target  = task.get("target_name") or task.get("target_xyz")
            timeout = task.get("timeout")
            if target is None:
                print("[Pioneer3AT] navigate_to subtask missing 'target_name' or 'target_xyz'.")
                return
            ok, msg = self.navigate_to(target, timeout=timeout)
            logger = self._adapter.get_logger()
            if ok:
                logger.info(f"[Pioneer3AT] navigate result: {msg}")
            else:
                logger.error(f"[Pioneer3AT] navigate result: {msg}")
        elif action == "transport_to":
            target  = task.get("target_name") or task.get("target_xyz")
            timeout = task.get("timeout")
            if target is None:
                print("[Pioneer3AT] transport_to subtask missing 'target_name' or 'target_xyz'.")
                return
            ok, msg = self.navigate_to(target, timeout=timeout)
            logger = self._adapter.get_logger()
            if ok:
                logger.info(f"[Pioneer3AT] transport result: {msg}")
            else:
                logger.error(f"[Pioneer3AT] transport result: {msg}")
        else:
            print(f"[Pioneer3AT] Unknown task action: '{action}'")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _on_nav_result(self, success: bool, message: str) -> None:
        """Called by PioneerAdapter when a navigation goal finishes."""
        self._nav_result = (success, message)
        self._nav_done_event.set()
