"""
ROS2 adapter for a commanded human actor in Gazebo / Ignition.

Mirrors the FrankaAdapter interface: non-blocking public methods,
a result_callback(success, message) fired on completion, and a
standalone __main__ entry point.

The underlying actor_controller node uses std_msgs/String topics:

    Publishes to:   /<actor_name>/actor_command   (std_msgs/String, JSON)
    Subscribes to:  /<actor_name>/actor_result    (std_msgs/String, JSON)

Supported commands
------------------
    human.goto(x, y, final_yaw)   – move to (x, y) then face final_yaw
    human.stop()                  – interrupt any in-progress movement
    human.test()                  – run the built-in square trajectory
    human.finish()                – complete current action then shut actor down

Usage
-----
    human = HumanAdapter(
        actor_name="host",
        result_callback=on_done,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(human)
    threading.Thread(target=executor.spin, daemon=True).start()

    time.sleep(0.5)  # let subscriber register

    human.goto(x=-0.19, y=-6.76, final_yaw=1.57)

Standalone
----------
    python3 HumanAdapter.py --actor host --x -0.19 --y -6.76 --yaw 1.57
    python3 HumanAdapter.py --actor host --command stop
    python3 HumanAdapter.py --actor host --command test
    python3 HumanAdapter.py --actor host --command finish
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from typing import Callable, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

ResultCallback = Callable[[bool, str], None]


# ---------------------------------------------------------------------------
# HumanAdapter
# ---------------------------------------------------------------------------

class HumanAdapter(Node):
    """
    ROS2 adapter for a Gazebo human actor driven by actor_controller.py.

    Parameters
    ----------
    actor_name : str
        Namespace of the actor (e.g. "host", "w1"). Must match the
        actor_controller node's namespace.
    result_callback : ResultCallback
        Called when a command finishes (success or failure).
        Signature: callback(success: bool, message: str) -> None
    result_timeout : float, optional
        Seconds to wait for a result before declaring a timeout.
        Default: 60.0 s.  Increase for long-range goto moves.
    node_name : str, optional
        Override the ROS2 node name.
    """

    def __init__(
        self,
        actor_name:      str,
        result_callback: ResultCallback,
        result_timeout:  float         = 60.0,
    ) -> None:
        super().__init__(actor_name)

        self._actor_name      = actor_name
        self._result_callback = result_callback
        self._result_timeout  = result_timeout

        # Task state
        self._busy      = False
        self._busy_lock = threading.Lock()

        # Used to wake the waiting thread when a result arrives
        self._result_event: threading.Event = threading.Event()
        self._pending_result: Optional[dict] = None
        self._pending_command: Optional[str] = None

        # Publisher — send commands to actor_controller
        self._cmd_pub = self.create_publisher(
            String,
            f"/{actor_name}/actor_command",
            10,
        )

        # Subscriber — receive results from actor_controller
        self._result_sub = self.create_subscription(
            String,
            f"/{actor_name}/actor_result",
            self._on_result,
            10,
        )

        self.get_logger().info(
            f"[HumanAdapter] Ready — actor='{actor_name}'. "
            f"Publishing to /{actor_name}/actor_command, "
            f"listening on /{actor_name}/actor_result."
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def goto(
        self,
        x:         float,
        y:         float,
        final_yaw: float = 0.0,
    ) -> bool:
        """
        Move the actor to (x, y) then rotate to face final_yaw.

        Non-blocking: returns True if the command was accepted (started),
        False if the adapter is already busy.
        result_callback(success, message) is called when the actor arrives.
        """
        payload = {"command": "goto", "x": x, "y": y, "final_yaw": final_yaw}
        return self._dispatch(payload)

    def stop(self) -> bool:
        """
        Interrupt any in-progress movement immediately.

        Returns True as soon as the stop signal is sent; result_callback
        is fired once the controller confirms the halt.
        """
        return self._dispatch({"command": "stop"})

    def test(self) -> bool:
        """
        Run the built-in square-loop trajectory (one full lap).

        Non-blocking; result_callback fires when the lap finishes.
        """
        return self._dispatch({"command": "test"})

    def finish(self) -> bool:
        """
        Ask the actor to complete its current action then shut down.

        result_callback fires once the controller acknowledges.
        """
        return self._dispatch({"command": "finish"})

    @property
    def is_busy(self) -> bool:
        """True while a command is in flight."""
        with self._busy_lock:
            return self._busy

    # ------------------------------------------------------------------
    # Internal — dispatch
    # ------------------------------------------------------------------

    def _dispatch(self, payload: dict) -> bool:
        """
        Publish a command JSON and start a watcher thread that waits for
        the matching result and fires result_callback.
        """
        cmd = payload["command"]

        with self._busy_lock:
            if self._busy:
                self.get_logger().warn(
                    f"[HumanAdapter] Rejected '{cmd}' — adapter is already busy."
                )
                return False
            self._busy            = True
            self._pending_command = cmd
            self._pending_result  = None
            self._result_event.clear()

        self.get_logger().info(f"[HumanAdapter] Sending command: {payload}")
        self._cmd_pub.publish(String(data=json.dumps(payload)))

        threading.Thread(
            target=self._wait_for_result,
            args=(cmd,),
            daemon=True,
        ).start()

        return True

    def _wait_for_result(self, cmd: str) -> None:
        """Block until a result arrives for *cmd*, then fire the callback."""
        arrived = self._result_event.wait(timeout=self._result_timeout)

        with self._busy_lock:
            self._busy = False

        if not arrived:
            msg = (
                f"Timeout ({self._result_timeout}s) waiting for result "
                f"of '{cmd}'."
            )
            self.get_logger().error(f"[HumanAdapter] ✗ {msg}")
            self._result_callback(False, msg)
            return

        result  = self._pending_result or {}
        success = bool(result.get("success", False))
        message = str(result.get("message", ""))

        icon   = "✓" if success else "✗"
        log_fn = self.get_logger().info if success else self.get_logger().error
        log_fn(f"[HumanAdapter] {icon} {message}")

        self._result_callback(success, message)

    # ------------------------------------------------------------------
    # Internal — result subscriber
    # ------------------------------------------------------------------

    def _on_result(self, msg: String) -> None:
        """
        Receive a JSON result from actor_controller and wake the watcher
        thread if the result belongs to the pending command.
        """
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(
                f"[HumanAdapter] Could not parse result JSON: '{msg.data}'"
            )
            return

        incoming_cmd = result.get("command", "")

        # Only wake the watcher if this result matches what we sent
        with self._busy_lock:
            if incoming_cmd != self._pending_command:
                self.get_logger().debug(
                    f"[HumanAdapter] Ignoring result for '{incoming_cmd}' "
                    f"(waiting for '{self._pending_command}')."
                )
                return
            self._pending_result = result

        self._result_event.set()


# ---------------------------------------------------------------------------
# Standalone test
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standalone test for HumanAdapter."
    )
    parser.add_argument(
        "--actor", default="host", metavar="NAME",
        help="Actor namespace as set in the launch file (default: host).",
    )
    parser.add_argument(
        "--command", choices=["goto", "stop", "test", "finish"],
        default="goto",
        help="Command to send (default: goto).",
    )
    parser.add_argument("--x",   type=float, default=-0.19,
                        help="Target X for goto (default: -0.19).")
    parser.add_argument("--y",   type=float, default=-6.76,
                        help="Target Y for goto (default: -6.76).")
    parser.add_argument("--yaw", type=float, default=1.57,
                        help="Final yaw (rad) for goto (default: 1.57).")
    parser.add_argument(
        "--timeout", type=float, default=60.0,
        help="Seconds to wait for the actor to finish (default: 60).",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()

    done_event = threading.Event()
    result_box: list = []

    def on_result(success: bool, message: str) -> None:
        result_box.append((success, message))
        done_event.set()

    human = HumanAdapter(
        actor_name      = args.actor,
        result_callback = on_result,
        result_timeout  = args.timeout,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(human)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Small delay so the subscriber is registered before we publish
    time.sleep(0.5)

    try:
        if args.command == "goto":
            accepted = human.goto(x=args.x, y=args.y, final_yaw=args.yaw)
        elif args.command == "stop":
            accepted = human.stop()
        elif args.command == "test":
            accepted = human.test()
        elif args.command == "finish":
            accepted = human.finish()
        else:
            accepted = False

        if not accepted:
            print("[standalone] Task rejected — adapter is busy.")
        else:
            print(
                f"[standalone] Command '{args.command}' sent. "
                f"Waiting up to {args.timeout}s ..."
            )
            finished = done_event.wait(timeout=args.timeout + 5.0)
            if not finished:
                print("[standalone] ✗ Timed out waiting for done_event.")
            else:
                success, message = result_box[0]
                print(f"[standalone] {'✓' if success else '✗'} {message}")

    except KeyboardInterrupt:
        print("\n[standalone] Interrupted — sending stop.")
        human.stop()
        time.sleep(1.0)
    finally:
        executor.shutdown()
        human.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
