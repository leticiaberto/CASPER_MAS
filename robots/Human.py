"""
Human
=====
High-level agent for a Gazebo / Ignition human actor driven by
actor_controller.py (std_msgs/String topic-based command protocol).

Wraps HumanAdapter the same way Tiago wraps TiagoAdapter:
  - owns the ROS2 executor and spin thread
  - exposes clean blocking and non-blocking public methods
  - fires result_callback(success, message) on completion

Architecture
------------
                ┌─────────────────────────────┐
                │  Human                       │
                │                             │
                │  HumanAdapter  (ROS2 node)  │
                │    ├─ pub /<name>/actor_command  (std_msgs/String JSON)
                │    └─ sub /<name>/actor_result   (std_msgs/String JSON)
                └─────────────────────────────┘

Usage
-----
    human = Human(actor_name="host")

    # Blocking — returns (success, message) when the actor arrives
    ok, msg = human.goto(x=-0.19, y=-6.76, final_yaw=1.57)

    # Non-blocking — pass a callback, returns immediately
    human.goto(x=2.0, y=1.0, final_yaw=0.0,
               callback=lambda ok, msg: print(ok, msg))

    human.stop()
    human.test()
    human.finish()

    human.shutdown()
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor

from human_adapters.HumanAdapter import HumanAdapter
from src.entities.Agent import Agent

locations = {
    "DiningTable": {
        "x": -0.77,
        "y": -1.54,
        "yaw": 3.14
    },

    "House": {
        "x": -0.19,
        "y": -6.76,
        "yaw": 1.57
    },

    "GroupOfGuests": {
        "x": 3.25,
        "y": -2.82,
        "yaw": 0.0
    },

    "MainPrepTable": {
        "x": 0.95,
        "y": 4.30,
        "yaw": 1.57
    },

    "GrillPrepTable": {
        "x": 4.14,
        "y": 4.24,
        "yaw": 0.8
    }
}

# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

ResultCallback = Callable[[bool, str], None]


# ---------------------------------------------------------------------------
# Human
# ---------------------------------------------------------------------------

class Human(Agent):
    """
    High-level handle for a Gazebo human actor.

    Parameters
    ----------
    actor_name : str
        Namespace of the actor (must match the actor_controller launch arg).
        E.g. "host", "w1", "visitor1".
    result_timeout : float
        Seconds to wait for a result before declaring a timeout.
        Default: 120 s — covers slow or long-range moves.
    node_name : str, optional
        Override the internal ROS2 node name.
    """

    def __init__(
        self,
        actor_name:     str,
        skill_weights,
        contexts,
        role,
        teamsize:     int,
        result_timeout: float         = 120.0,
        node_name:     Optional[str] = None,
        use_sim       = True,
        workspace:    str           = None,
        party_duration:       float         = 3600.0, #1h default
    ) -> None:
        constraints = {
            "can_move":               True,
            "can_manipulate":         True,
            "max_size_object_cm":     10,
            "max_payload_kg":         3,
            "max_reach_cm":           85,
            "can_transport_objects":  True,
            "workspace":              workspace,
        }

        super().__init__(actor_name, constraints, skill_weights, contexts, role, teamsize, party_duration)

        self._actor_name = actor_name

        # ── ROS2 initialisation ───────────────────────────────────────
        if not rclpy.ok():
            rclpy.init()

        # Completion signal shared between the adapter callback and blocking callers
        self._goal_done_event           = threading.Event()
        self._last_result: Tuple[bool, str] = (False, "No command sent yet.")

        # ── HumanAdapter node ─────────────────────────────────────────
        self._adapter = HumanAdapter(
            actor_name      = actor_name,
            result_callback = self._on_result,
            result_timeout  = float(result_timeout),
            node_name       = node_name,
        )

        # ── Executor + spin thread ────────────────────────────────────
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._adapter)

        self._spin_thread = threading.Thread(
            target = self._executor.spin,
            daemon = True,
            name   = f"human_{actor_name}_spin",
        )
        self._spin_thread.start()

        # Give the publisher/subscriber a moment to register before any
        # caller tries to send a command.
        time.sleep(0.5)

        self._adapter.get_logger().info(
            f"[Human] Ready — actor='{actor_name}'."
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def get_destionation_coordinates(self, location_name: str) -> Tuple[float, float, float]:
        """
        Get the coordinates of a named location.

        Parameters
        ----------
        location_name : str
            Name of the location, e.g. "DiningTable", "House", etc.

        Returns
        -------
        (x, y, yaw) : Tuple[float, float, float]
            Coordinates of the location in the world frame.
            Raises KeyError if the location name is not found.
        """
        if location_name not in locations:
            raise KeyError(f"Location '{location_name}' not found.")
        loc = locations[location_name]
        return loc["x"], loc["y"], loc["yaw"]

    def goto(
        self,
        location   = None,
        x               = None,
        y               = None,
        final_yaw       = None,
        callback:  Optional[ResultCallback] = None,
    ) -> Tuple[bool, str]:
        """
        Move the actor to (x, y) then rotate to face final_yaw.

        Parameters
        ----------
        x, y : float
            World-frame target position (metres).
        final_yaw : float
            Heading to face after arriving (radians, world frame).
            Default: 0.0 (facing +X).
        callback : callable, optional
            If provided the call returns immediately and callback(success, message)
            is fired when the actor arrives.
            If omitted the call blocks until the actor arrives (or times out).

        Returns
        -------
        (success, message) when blocking; (True, "accepted") when non-blocking.
        """
         # Option 1: location key
        if location is not None:
            x, y, final_yaw = self.get_destionation_coordinates(location)
        
        # Option 2: direct coordinates
        else:
            if x is None or y is None or final_yaw is None:
                raise ValueError(
                    "You must provide either a valid location "
                    "or x, y, yaw coordinates."
                )

        return self._send("goto", callback, x=x, y=y, final_yaw=final_yaw)

    def stop(
        self,
        callback: Optional[ResultCallback] = None,
    ) -> Tuple[bool, str]:
        """
        Interrupt any in-progress movement immediately.

        The actor stays at its current position.
        Blocking by default; pass a callback for fire-and-forget.
        """
        return self._send("stop", callback)

    def test(
        self,
        callback: Optional[ResultCallback] = None,
    ) -> Tuple[bool, str]:
        """
        Run the built-in square-loop trajectory (one full lap).

        Blocking by default; pass a callback for fire-and-forget.
        """
        return self._send("test", callback)

    def finish(
        self,
        callback: Optional[ResultCallback] = None,
    ) -> Tuple[bool, str]:
        """
        Ask the actor to complete its current action then shut down its
        controller node.

        Blocking by default; pass a callback for fire-and-forget.
        """
        return self._send("finish", callback)

    @property
    def is_busy(self) -> bool:
        """True while a command is in flight."""
        return self._adapter.is_busy

    # ------------------------------------------------------------------
    # Agent interface (task-graph dispatch)
    # ------------------------------------------------------------------

    def _execute_task_specific(self, task: dict) -> None:
        print(f"[Human] Executing task: {task}")
        action = task.get("action") if isinstance(task, dict) else task

        if action == "PickRice":
            ok, msg = self.goto(location="House")
        elif action == "CookRice":
            ok, msg = self.goto(location="House")
        elif action == "ServeMainDish":
            ok, msg = self.goto(location="House")
            ok, msg = self.goto(location="DiningTable")
        elif action == "ServeSides":
            ok, msg = self.goto(location="MainPrepTable")
            ok, msg = self.goto(location="DiningTable")
        elif action == "ServeSalad":
            ok, msg = self.goto(location="MainPrepTable")
            ok, msg = self.goto(location="DiningTable")
        elif action == "ServeGrilledFood":
            ok, msg = self.goto(location="GrillPrepTable")
            ok, msg = self.goto(location="DiningTable")
        elif action == "WelcomeGuests":
            ok, msg = self.goto(location="House")
            ok, msg = self.goto(location="GroupOfGuests")
        elif action == "PutTheDishesAway":
            ok, msg = self.goto(location="House")
        else:
            print(f"[Human] Unknown task action: {action}")

    def shutdown(self) -> None:
        """Cleanly stop the executor and destroy the ROS2 node."""
        self._adapter.get_logger().info("[Human] Shutting down ...")
        self._executor.shutdown(timeout_sec=3.0)
        try:
            self._adapter.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        print(f"[Human] '{self._actor_name}' shutdown complete.")

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _send(
        self,
        command:  str,
        callback: Optional[ResultCallback],
        **kwargs,
    ) -> Tuple[bool, str]:
        """
        Dispatch a command through the adapter.

        Non-blocking path: wire a one-shot wrapper around the user callback
        and return immediately.

        Blocking path: clear the event, let the adapter's internal callback
        (_on_result) set it, then wait.
        """
        if callback is not None:
            # Non-blocking — install a temporary callback then return
            self._install_callback(command, callback, **kwargs)
            return True, "accepted"

        # Blocking
        self._goal_done_event.clear()
        accepted = self._dispatch(command, **kwargs)
        if not accepted:
            msg = f"[Human] Adapter busy — '{command}' rejected."
            self._adapter.get_logger().warn(msg)
            return False, msg

        self._goal_done_event.wait()
        return self._last_result

    def _dispatch(self, command: str, **kwargs) -> bool:
        """Call the matching HumanAdapter method and return accepted bool."""
        if command == "goto":
            return self._adapter.goto(**kwargs)
        if command == "stop":
            return self._adapter.stop()
        if command == "test":
            return self._adapter.test()
        if command == "finish":
            return self._adapter.finish()
        return False

    def _install_callback(
        self,
        command:  str,
        callback: ResultCallback,
        **kwargs,
    ) -> None:
        """
        Temporarily swap in a one-shot callback, dispatch, then restore.
        Runs in a thread so the caller is not blocked.
        """
        original_callback = self._adapter._result_callback

        def one_shot(success: bool, message: str) -> None:
            self._adapter._result_callback = original_callback
            self._on_result(success, message)   # keep internal state fresh
            callback(success, message)

        self._adapter._result_callback = one_shot
        accepted = self._dispatch(command, **kwargs)
        if not accepted:
            # Restore immediately if rejected
            self._adapter._result_callback = original_callback
            callback(False, "Adapter busy — command rejected.")

    def _on_result(self, success: bool, message: str) -> None:
        """Internal callback — stores result and wakes any blocking caller."""
        self._last_result = (success, message)
        self._goal_done_event.set()