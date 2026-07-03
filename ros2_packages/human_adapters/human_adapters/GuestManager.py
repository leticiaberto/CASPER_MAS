"""
GuestManager
============
Orchestrates guest arrivals in sync with the host's WelcomeGuests action.

Rendezvous protocol (two sync points per guest)
------------------------------------------------
  Two sync points — sequential movement to protect Gazebo performance
    1. GuestManager publishes "ready" once adapter is live → host greets guest
    2. Host walks to group, then publishes "walk" → guest walks to group
    Only one actor moves at a time, keeping real-time factor healthy.

Topic map
---------
  sub  /<host>/actor_state    String JSON   ← host publishes WelcomeGuests/loop
  pub  /<host>/guest_sync     String JSON   → host reads to unblock waits
  HumanAdapter per guest:
       pub  /<guest>/actor_command
       sub  /<guest>/actor_result

Usage
-----
    manager = GuestManager(
        host_name      = "host",
        guest_configs  = [
            {"actor_type": "FemaleVisitor", "color": "blue",
             "entry_x": -0.19, "entry_y": -6.76, "entry_yaw": 1.57},
            ...
        ],
        # group_positions defaults to DEFAULT_GROUP_POSITIONS (6 spots)
        world_name      = "backyard",
    )
    manager.spin_forever()   # blocks; Ctrl-C to stop

Standalone
----------
    python3 GuestManager.py --host host --world backyard
"""

from __future__ import annotations

import argparse
import yaml
import json
import subprocess
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from human_adapters.HumanAdapter import HumanAdapter

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

DEFAULT_GUEST_CONFIGS: List[Dict] = [
    {"actor_type": "FemaleVisitor",
     "entry_x": -0.19, "entry_y": -6.76, "entry_yaw": 1.57},
    {"actor_type": "WalkingActor",  
     "entry_x": -0.19, "entry_y": -6.76, "entry_yaw": 1.57},
    {"actor_type": "CasualFemale",  "color": "orange",
     "entry_x": -0.19, "entry_y": -6.76, "entry_yaw": 1.57},
    {"actor_type": "FemaleVisitor", "color": "yellow",
     "entry_x": -0.19, "entry_y": -6.76, "entry_yaw": 1.57},
     {"actor_type": "CasualFemale",  
     "entry_x": -0.19, "entry_y": -6.76, "entry_yaw": 1.57},
    {"actor_type": "WalkingActor",  "color": "orange",
     "entry_x": -0.19, "entry_y": -6.76, "entry_yaw": 1.57},
]

DEFAULT_GROUP_POSITIONS: List[Dict] = [
    {"x":  4.78, "y": -3.31, "yaw": -3.14},  # guest 1
    {"x":  4.0,  "y": -2.36, "yaw": -1.57},  # guest 2
    {"x":  3.57, "y": -3.62, "yaw":  0.8 },  # guest 3
    {"x": -1.94, "y":  4.66, "yaw": -0.42},  # guest 4
    {"x": -1.30, "y":  4.11, "yaw":  2.55},  # guest 5
    {"x": -3.37, "y": -2.92, "yaw":  2.73},  # guest 6
]


# ---------------------------------------------------------------------------
# GuestManager
# ---------------------------------------------------------------------------

class GuestManager(Node):
    """
    ROS2 node that reacts to /<host>/actor_state and drives each guest
    through a two-point rendezvous with the host.

    Parameters
    ----------
    host_name : str
        ROS2 namespace of the host Human actor (e.g. ``"host"``).
    guest_configs : list[dict]
        One entry per guest, in arrival order.
        Required keys: actor_type, entry_x, entry_y, entry_yaw. color is optional.
    group_positions : list[dict]
        Party group coordinates (x, y, yaw).
    world_name : str
        Gazebo world name passed to actor.launch.py.
    launch_pkg : str
        ROS2 package containing actor.launch.py.
    controller_ready_delay : float
        Extra seconds to wait after launch before HumanAdapter connects.
        Default 12 s covers the 5 s + 10 s TimerActions inside the launch.
    result_timeout : float
        Per-goto timeout forwarded to each guest's HumanAdapter.
    sync_timeout : float
        How long the host will wait at each sync point before giving up.
        Default 300 s (5 min) — enough for slow spawn + walk.
    """

    def __init__(
        self,
        host_name:               str,
        guest_configs:           List[Dict]   = None,
        group_positions:         List[Dict]   = None,
        world_name:              str          = "backyard",
        launch_pkg:              str          = "human_adapters",
        controller_ready_delay:  float        = 12.0,
        result_timeout:          float        = 120.0,
        sync_timeout:            float        = 300.0,
        node_name:               Optional[str] = None,
    ) -> None:
        node_name = node_name or f"guest_manager_{host_name}"
        super().__init__(node_name)

        self._host_name              = host_name
        self._guest_configs          = guest_configs  or DEFAULT_GUEST_CONFIGS
        self._group_positions        = group_positions or DEFAULT_GROUP_POSITIONS
        self._world_name             = world_name
        self._launch_pkg             = launch_pkg
        self._controller_ready_delay = controller_ready_delay
        self._result_timeout         = result_timeout
        self._sync_timeout           = sync_timeout

        # Executor — set by spin_forever so pipeline threads can add nodes
        self._executor: Optional[MultiThreadedExecutor] = None

        # Spawn counter — atomically incremented before each launch
        self._guests_spawned = 0
        self._spawn_lock     = threading.Lock()

        # Keep adapters alive for the lifetime of the manager
        self._guest_adapters: List[HumanAdapter] = []
        self._adapter_lock   = threading.Lock()

        # ── Subscribe to host state ───────────────────────────────────
        self._state_sub = self.create_subscription(
            String,
            f"/{host_name}/actor_state",
            self._on_host_state,
            10,
        )

        # ── Publish sync signals back to host ─────────────────────────
        self._sync_pub = self.create_publisher(
            String,
            f"/{host_name}/guest_sync",
            10,
        )

        self.get_logger().info(
            f"[GuestManager] Ready — watching /{host_name}/actor_state, "
            f"publishing to /{host_name}/guest_sync. "
            f"{len(self._guest_configs)} guest(s) configured."
        )

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    def spin_forever(self) -> None:
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        try:
            self._executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown(self._executor)

    def shutdown(self, executor: Optional[MultiThreadedExecutor] = None) -> None:
        self.get_logger().info("[GuestManager] Shutting down …")
        ex = executor or self._executor
        if ex:
            ex.shutdown(timeout_sec=3.0)
        with self._adapter_lock:
            for adapter in self._guest_adapters:
                try:
                    adapter.destroy_node()
                except Exception:
                    pass
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # State subscriber
    # ------------------------------------------------------------------

    def _on_host_state(self, msg: String) -> None:
        """React to WelcomeGuests/loop — spawn the next guest in a thread."""
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        # Party is over — walk every already-spawned guest back home.
        if state.get("action") == "FarewellGuests" and state.get("phase") == "start":
            threading.Thread(
                target=self._farewell_pipeline, daemon=True, name="farewell_pipeline"
            ).start()
            return

        if state.get("action") != "WelcomeGuests" or state.get("phase") != "loop":
            return

        with self._spawn_lock:
            idx = self._guests_spawned
            if idx >= len(self._guest_configs):
                self.get_logger().info("[GuestManager] All guests spawned.")
                return
            self._guests_spawned += 1   # claim this slot before releasing lock

        cfg        = self._guest_configs[idx]
        guest_name = f"guest_{idx + 1}"

        self.get_logger().info(
            f"[GuestManager] Guest #{idx + 1} triggered → spawning '{guest_name}' "
            f"({cfg['actor_type']}, {cfg.get('color', 'default')})"
        )

        threading.Thread(
            target = self._pipeline,
            args   = (guest_name, cfg, idx),
            daemon = True,
            name   = f"guest_{idx + 1}_pipeline",
        ).start()

    # ------------------------------------------------------------------
    # Full pipeline for one guest
    # ------------------------------------------------------------------

    def _pipeline(self, guest_name: str, cfg: Dict, idx: int) -> None:
        """
        1. Launch actor in Gazebo (spawns at door coordinates)
        2. Wait for controller + build HumanAdapter
        3. Publish SYNC "ready"       → host unblocks, greets, then walks to group
        4. Drive guest to group (fires immediately, in parallel with host)
        """

        # ── 1. Launch ─────────────────────────────────────────────────
        if not self._launch_actor(guest_name, cfg):
            self.get_logger().error(
                f"[GuestManager] Launch failed for '{guest_name}' — aborting."
            )
            return

        # ── 2. Wait for controller then build adapter ──────────────────
        self.get_logger().info(
            f"[GuestManager] Waiting {self._controller_ready_delay}s for "
            f"'{guest_name}' controller …"
        )
        time.sleep(self._controller_ready_delay)

        done_event = threading.Event()
        result_box: list = []

        def on_result(success: bool, message: str) -> None:
            result_box.append((success, message))
            done_event.set()

        adapter = HumanAdapter(
            actor_name      = guest_name,
            result_callback = on_result,
            result_timeout  = self._result_timeout,
        )

        with self._adapter_lock:
            self._guest_adapters.append(adapter)

        # Spin the adapter — attach to the running executor if available
        if self._executor is not None:
            try:
                self._executor.add_node(adapter)
            except Exception as exc:
                self.get_logger().warning(
                    f"[GuestManager] Could not add adapter to executor ({exc}); "
                    "falling back to dedicated spin thread."
                )
                self._spin_adapter_in_thread(adapter)
        else:
            self.get_logger().warning(
                "[GuestManager] Executor not ready yet — spinning adapter in dedicated thread."
            )
            self._spin_adapter_in_thread(adapter)

        time.sleep(0.5)   # let pub/sub register

        # ── 3. SYNC — guest spawned at door, adapter is live ─────────
        # Publish "ready" so the host unblocks from its door wait.
        self._publish_sync("ready", idx + 1)

        # ── 4. WAIT for host "walk" signal before moving to group ─────
        # The host greets the guest (TIME_WELCOMING_GUESTS), then publishes
        # "walk" on guest_sync.  Only then does the guest start moving.
        # This also prevents two actors pathfinding simultaneously, which
        # tanks Gazebo real-time factor.
        self.get_logger().info(
            f"[GuestManager] '{guest_name}' waiting for host 'walk' signal …"
        )
        walk_event = threading.Event()

        def _on_walk(msg: String) -> None:
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            if data.get("phase") == "walk" and data.get("guest") == idx + 1:
                walk_event.set()

        walk_sub = self.create_subscription(
            String,
            f"/{self._host_name}/guest_sync",
            _on_walk,
            10,
        )

        arrived = walk_event.wait(timeout=self._sync_timeout)
        self.destroy_subscription(walk_sub)

        if not arrived:
            self.get_logger().error(
                f"[GuestManager] '{guest_name}' timed out waiting for 'walk' signal."
            )
            return

        # ── 5. Host finishes first, then guest walks to group ─────────
        # Sequential movement keeps Gazebo real-time factor healthy.
        group_idx = min(idx, len(self._group_positions) - 1)
        group     = self._group_positions[group_idx]

        self.get_logger().info(
            f"[GuestManager] '{guest_name}' → position {group_idx + 1} "
            f"({group['x']}, {group['y']})"
        )

        self._blocking_goto(
            adapter    = adapter,
            x          = group["x"],
            y          = group["y"],
            yaw        = group["yaw"],
            label      = f"{guest_name}/group_{group_idx + 1}",
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_sync(self, phase: str, guest_number: int) -> None:
        """Publish a sync signal that Human._wait_for_guest_sync() reads."""
        payload = {"phase": phase, "guest": guest_number}
        self._sync_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"[GuestManager] SYNC → phase='{phase}' guest={guest_number}"
        )

    def _blocking_goto(
        self,
        adapter:    HumanAdapter,
        x: float, y: float, yaw: float,
        label: str,
    ) -> bool:
        """
        Drive *adapter* to (x, y, yaw) and block until it finishes.

        Retargets adapter._result_callback for this call — the adapter
        is reused across multiple commands over its lifetime (arrival,
        then later being walked home), and each call needs its own
        done_event/result_box rather than the one from a previous call.
        """
        done_event = threading.Event()
        result_box: list = []

        def on_result(success: bool, message: str) -> None:
            result_box.append((success, message))
            done_event.set()

        adapter._result_callback = on_result

        accepted = adapter.goto(x=x, y=y, final_yaw=yaw)
        if not accepted:
            self.get_logger().warn(
                f"[GuestManager] goto({label}) rejected — adapter busy."
            )
            return False
        finished = done_event.wait(timeout=self._result_timeout + 5.0)
        if not finished:
            self.get_logger().error(f"[GuestManager] Timeout at '{label}'.")
            return False
        success, message = result_box[0]
        icon = "✓" if success else "✗"
        self.get_logger().info(f"[GuestManager] {label} {icon}  {message}")
        return success

    def _farewell_pipeline(self) -> None:
        """
        Party is over — walk every already-spawned guest back to their
        entry coordinates, then shut their controller down. Sequential,
        same as the arrival pipeline, to protect Gazebo's real-time factor.
        """
        with self._adapter_lock:
            adapters = list(self._guest_adapters)

        self.get_logger().info(
            f"[GuestManager] Party over — sending {len(adapters)} guest(s) home."
        )

        for i, adapter in enumerate(adapters):
            cfg = self._guest_configs[i] if i < len(self._guest_configs) else {}
            x   = cfg.get("entry_x", 0.0)
            y   = cfg.get("entry_y", 0.0)
            yaw = cfg.get("entry_yaw", 0.0)

            self._blocking_goto(adapter, x, y, yaw, label=f"guest_{i + 1}/home")
            adapter.finish()  # let the controller shut itself down once idle

        self.get_logger().info("[GuestManager] All guests sent home.")

    def _launch_actor(self, actor_name: str, cfg: Dict) -> bool:
        cmd = [
            "ros2", "launch", self._launch_pkg, "actor.launch.py",
            f"actor_name:={actor_name}",
            f"world_name:={self._world_name}",
            f"actor_type:={cfg['actor_type']}",
            f"x:={cfg.get('entry_x', 0.0)}",
            f"y:={cfg.get('entry_y', 0.0)}",
            f"yaw:={cfg.get('entry_yaw', 0.0)}",
        ]
        color = cfg.get("color", "")
        if color:
            cmd.append(f"color:={color}")
        self.get_logger().info(f"[GuestManager] $ {' '.join(cmd)}")
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            time.sleep(1.0)
            ret = proc.poll()
            if ret is not None and ret != 0:
                out = proc.stdout.read().decode(errors="replace")
                self.get_logger().error(
                    f"[GuestManager] Launch exited (code {ret}):\n{out}"
                )
                return False
            return True
        except FileNotFoundError:
            self.get_logger().error(
                "[GuestManager] 'ros2' not found — source your workspace."
            )
            return False
        except Exception as exc:
            self.get_logger().error(f"[GuestManager] Launch error: {exc}")
            return False

    @staticmethod
    def _spin_adapter_in_thread(adapter: HumanAdapter) -> None:
        ex = MultiThreadedExecutor()
        ex.add_node(adapter)
        threading.Thread(
            target=ex.spin, daemon=True,
            name=f"spin_{adapter._actor_name}"
        ).start()


# ---------------------------------------------------------------------------
# Standalone
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="GuestManager standalone.")
    p.add_argument("--host",    default="host",      help="Host namespace.")
    p.add_argument("--world",   default="backyard",  help="Gazebo world.")
    p.add_argument("--timeout", type=float, default=120.0)
    p.add_argument("--exp",     default="exp1.yaml", help="Experiment config file.")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    # Kill any stale actor_controller processes from previous runs
    subprocess.run(["pkill", "-f", "actor_controller"], capture_output=True)
    time.sleep(1.0)

    with open("configs/exps/" + args.exp + ".yaml") as f:
        exp_cfg = yaml.safe_load(f)

    guests = exp_cfg.get("guests", 0)
    print(f"[GuestManager] Loaded '{args.exp}' — {guests} guest(s).")

    if not rclpy.ok():
        rclpy.init()

    manager = GuestManager(
        host_name       = args.host,
        world_name      = args.world,
        result_timeout  = args.timeout,
        guest_configs   = DEFAULT_GUEST_CONFIGS[:guests],
    )
    print(f"[GuestManager] Watching /{args.host}/actor_state — Ctrl-C to stop.")
    manager.spin_forever()


if __name__ == "__main__":
    main()
