#!/usr/bin/env python3
"""
object_to_robot.py
------------------
A reusable ROS2 library class that any robot node can instantiate to get
Gazebo Fortress object poses expressed in that robot's own coordinate frame.

Typical usage inside a robot node
----------------------------------
    from object_to_robot import ObjectToRobot

    class MyRobotNode(Node):
        def __init__(self):
            super().__init__("my_robot_node")

            self.obj_transformer = ObjectToRobot(
                node        = self,          # pass your own node
                robot_name  = "fr3_robot1",  # Gazebo model name of this robot
                world_name  = "empty",       # Gazebo world name from your .sdf
            )

        def do_something(self):
            results = self.obj_transformer.get_poses(["box1", "cylinder1"])

            for obj_name, data in results.items():
                if "error" not in data:
                    pos = data["robot_pose"].position
                    print(f"{obj_name} is at x={pos.x:.3f} y={pos.y:.3f} z={pos.z:.3f}")

Robot name → base TF frame resolution
--------------------------------------
The class resolves the robot's base frame automatically from its name:

  1. Explicit override via `base_frame` constructor argument.
  2. Prefix match against ROBOT_BASE_FRAME_MAP (e.g. "fr3*" → "<name>/fr3_link0").
  3. Live TF tree probe using common frame name candidates.

Extend ROBOT_BASE_FRAME_MAP below to support additional robot types.

Assumptions
-----------
- The gz-ros2-bridge is running for /world/<world>/pose/info.
    The gz-ros2-bridge is running for /world/<world>/pose/info.
        Bridge launch (run this before your robot / before this script):
            ros2 run ros_gz_bridge parameter_bridge \
                /world/<WORLD_NAME>/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
                /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

            Replace <WORLD_NAME> with the name of your Gazebo world (e.g. "empty").
            You can find it in your .sdf file: <world name="...">
- Each robot's TF frames are namespaced as "<robot_name>/<frame>".
- The world TF frame is "world" (override via `world_frame` argument).
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers PoseStamped <-> TF2

from geometry_msgs.msg import PoseStamped, Pose
from tf2_msgs.msg import TFMessage

try:
    from transforms3d.euler import quat2euler
    _HAS_T3D = True
except ImportError:
    _HAS_T3D = False


# ─────────────────────────────────────────────────────────────────────────────
#  Robot type registry
#  Keys   : lowercase prefix of the Gazebo model / robot name
#  Values : format string – {name} is replaced with the full robot_name
#
#  Add your own robot types here.
# ─────────────────────────────────────────────────────────────────────────────
ROBOT_BASE_FRAME_MAP: dict[str, str] = {
    # Franka Research 3
    # When spawned with a robot name prefix, Gazebo/ros2_control flattens the
    # namespace with underscores: "fr3_robot1" -> "fr3_robot1_fr3_link0"
    "fr3":       "{name}_fr3_link0",
    "panda":     "{name}_panda_link0",      # Franka Panda (same convention)
    "tiago":     "{name}/base_footprint",   # PAL Tiago / Tiago++
    "ur":        "{name}/base_link",        # Universal Robots UR3/5/10/…
    "turtlebot": "{name}/base_footprint",   # TurtleBot 3 / 4
    "tb":        "{name}/base_footprint",
    "husky":     "{name}/base_link",        # Clearpath Husky
    "jackal":    "{name}/base_link",        # Clearpath Jackal
    "spot":      "{name}/body",             # Boston Dynamics Spot
    "robot":     "{name}/base_link",        # generic "robot" prefix
}

# Probed in order when no registry entry matches.
# Both slash-namespaced (ROS2 convention) and underscore-namespaced
# (Gazebo/URDF flattened) variants are tried for each candidate.
_FALLBACK_FRAMES: list[str] = [
    "{name}/base_link",
    "{name}_base_link",
    "{name}/base_footprint",
    "{name}_base_footprint",
    "{name}/base",
    "{name}_base",
    "base_link",
    "base_footprint",
]


# ─────────────────────────────────────────────────────────────────────────────
#  ObjectToRobot
# ─────────────────────────────────────────────────────────────────────────────
class ObjectToRobot:
    """
    Provides world-object → robot-frame pose transforms for a single robot.

    Parameters
    ----------
    node : rclpy.node.Node
        The ROS2 node that owns this instance (used for subscriptions,
        logging, clock and spinning).
    robot_name : str
        The Gazebo model name of *this* robot (e.g. "fr3_robot1").
        Used both to filter the pose/info topic and to resolve the TF frame.
    world_name : str
        The Gazebo world name as declared in your .sdf  (<world name="…">).
    base_frame : str, optional
        Explicit TF base frame for this robot. When omitted the frame is
        resolved automatically from robot_name via ROBOT_BASE_FRAME_MAP
        or by probing the live TF tree.
    world_frame : str
        TF frame that represents the world origin (default: "world").
    tf_timeout : float
        Seconds to wait for the TF tree to populate on startup (default: 5.0).
    pose_timeout : float
        Seconds to wait for the first pose/info message on startup (default: 5.0).
    """

    def __init__(
        self,
        node:         Node,
        robot_name:   str,
        world_name:   str,
        base_frame:   Optional[str] = None,
        world_frame:  str           = "world",
        tf_timeout:   float         = 5.0,
        pose_timeout: float         = 5.0,
    ) -> None:
        self._node         = node
        self.robot_name    = robot_name
        self.world_name    = world_name
        self.world_frame   = world_frame
        self._tf_timeout   = Duration(seconds=tf_timeout)
        self._pose_timeout = pose_timeout

        # Populated after the TF tree is available
        self._base_frame: Optional[str] = base_frame

        # { model_name: Pose }  — world poses, updated by pose/info subscription
        # Includes both the requested objects AND this robot itself.
        self._latest_poses: dict[str, Pose] = {}
        # Set to the frame_id from the first pose/info message received
        self._pose_info_world_frame: Optional[str] = None

        # ── TF2 — used only to read the frame tree for base-frame discovery
        #         (NOT used for the actual world→robot transform)
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node)

        # ── Subscribe to Gazebo Fortress pose/info ────────────────────────
        # BEST_EFFORT QoS is mandatory – the gz-ros2-bridge uses it.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pose_topic = f"/world/{world_name}/pose/info"
        node.create_subscription(
            TFMessage,
            self._pose_topic,
            self._pose_cb,
            qos_profile=qos,
        )
        self._log_info(f"ObjectToRobot ready | robot='{robot_name}' world='{world_name}'")

    # ─────────────────────────────────────────────────────────────────────
    #  Public API
    # ─────────────────────────────────────────────────────────────────────

    @property
    def base_frame(self) -> Optional[str]:
        """The resolved TF base frame for this robot, or None if not yet known."""
        return self._base_frame

    def get_poses(self, object_names: list[str]) -> dict[str, dict]:
        """
        Return the pose of each object expressed in this robot's base frame.

        Blocks briefly on first call to wait for pose/info messages and the
        TF tree. Subsequent calls return immediately using cached data.

        Parameters
        ----------
        object_names : list[str]
            Gazebo model names to query (e.g. ["box1", "cylinder1"]).

        Returns
        -------
        dict keyed by object name:
        {
          "box1": {
              "world_pose" : geometry_msgs/Pose,   # pose in world frame
              "robot_pose" : geometry_msgs/Pose,   # pose in this robot's frame
              "euler_deg"  : (roll, pitch, yaw),   # degrees (needs transforms3d)
          },
          "missing": { "error": "pose_not_found" },
          "bad_tf":  { "error": "tf_failed", "world_pose": Pose },
          "no_base": { "error": "base_frame_unknown" },
        }
        """
        self._ensure_ready(object_names)

        if self._base_frame is None:
            self._log_error(
                f"Cannot resolve base frame for robot '{self.robot_name}'. "
                "Set it explicitly via the base_frame argument, or add an entry "
                "to ROBOT_BASE_FRAME_MAP."
            )
            return {name: {"error": "base_frame_unknown"} for name in object_names}

        results: dict[str, dict] = {}
        for name in object_names:
            results[name] = self._transform_one(name)

        return results

    def get_pose(self, object_name: str) -> dict:
        """Convenience wrapper for a single object. Returns one result dict."""
        return self.get_poses([object_name])[object_name]

    # ─────────────────────────────────────────────────────────────────────
    #  Internals
    # ─────────────────────────────────────────────────────────────────────

    def _transform_one(self, object_name: str) -> dict:
        # ── Object world pose ─────────────────────────────────────────────
        if object_name not in self._latest_poses:
            self._log_warning(
                f"No pose for '{object_name}'. "
                "Is it spawned? Does the name match the Gazebo model exactly?"
            )
            return {"error": "pose_not_found"}
        obj_world_pose = self._latest_poses[object_name]

        # ── Robot world pose ──────────────────────────────────────────────
        # The robot's own pose comes from the same pose/info topic.
        # TF2 is NOT used here because the robot's TF tree is typically
        # disconnected from the Gazebo world frame (e.g. rooted at "odom").
        if self.robot_name not in self._latest_poses:
            self._log_warning(
                f"No world pose for robot '{self.robot_name}' in pose/info. "
                "Cannot compute relative transform."
            )
            return {"error": "robot_pose_not_found"}
        robot_world_pose = self._latest_poses[self.robot_name]

        # ── Compute object pose in robot frame ────────────────────────────
        robot_pose = self._relative_pose(obj_world_pose, robot_world_pose)

        entry: dict = {
            "world_pose": obj_world_pose,
            "robot_pose": robot_pose,
        }
        if _HAS_T3D:
            q = robot_pose.orientation
            roll, pitch, yaw = quat2euler([q.w, q.x, q.y, q.z], axes="sxyz")
            entry["euler_deg"] = (
                round(roll  * 57.2958, 3),
                round(pitch * 57.2958, 3),
                round(yaw   * 57.2958, 3),
            )

        self._log_result(object_name, entry)
        return entry

    # ── Pose/info callback ────────────────────────────────────────────────
    def _pose_cb(self, msg: TFMessage) -> None:
        """Cache world poses for all top-level Gazebo models, including this robot."""
        for tfs in msg.transforms:
            # Capture the authoritative world frame from the message header.
            if self._pose_info_world_frame is None and tfs.header.frame_id:
                self._pose_info_world_frame = tfs.header.frame_id

            child = tfs.child_frame_id
            if "::" in child:
                continue  # skip link-level entries (e.g. "robot::base_link")
            t = tfs.transform
            p = Pose()
            p.position.x    = t.translation.x
            p.position.y    = t.translation.y
            p.position.z    = t.translation.z
            p.orientation.x = t.rotation.x
            p.orientation.y = t.rotation.y
            p.orientation.z = t.rotation.z
            p.orientation.w = t.rotation.w
            self._latest_poses[child] = p

    # ── Startup checks ────────────────────────────────────────────────────
    def _ensure_ready(self, object_names: list[str]) -> None:
        """On first call: wait for pose messages and resolve base/world frames."""
        if not self._latest_poses:
            self._wait_for_poses(object_names)
        if self._base_frame is None:
            self._resolve_base_frame()

    def _wait_for_poses(self, object_names: list[str]) -> None:
        self._log_info(f"Waiting for pose/info from '{self._pose_topic}' …")
        deadline = time.time() + self._pose_timeout
        while time.time() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            # First wait for the topic to arrive at all (any message).
            # Checking only for requested objects can time out prematurely when
            # the objects arrive slightly later than the first message burst.
            if self._pose_info_world_frame is not None:
                # Topic is live — now check for the specific objects
                if any(n in self._latest_poses for n in object_names):
                    self._log_info(
                        f"Poses available: {[n for n in object_names if n in self._latest_poses]} ✓"
                    )
                    return
                # Topic is live but objects not seen yet — keep spinning briefly
        # Final check after deadline
        if self._pose_info_world_frame is None:
            self._log_warning(
                f"No messages received on '{self._pose_topic}' after {self._pose_timeout} s. "
                f"Check the gz-ros2-bridge is running:\n"
                f"  ros2 run ros_gz_bridge parameter_bridge \\\n"
                f"    {self._pose_topic}"
                f"@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
            )
        else:
            missing = [n for n in object_names if n not in self._latest_poses]
            if missing:
                self._log_warning(
                    f"pose/info topic is live but objects not seen: {missing}. "
                    f"Are they spawned? Known objects so far: {list(self._latest_poses.keys())}"
                )

    def _resolve_base_frame(self) -> None:
        """Resolve and cache this robot's base TF frame and world TF frame."""
        # Wait for TF tree to populate
        self._log_info("Waiting for TF tree …")
        deadline = time.time() + 10.0
        while time.time() < deadline:
            if self._tf_buffer.all_frames_as_string():
                break
            rclpy.spin_once(self._node, timeout_sec=0.2)

        all_frames = self._tf_buffer.all_frames_as_string()
        self._log_info(f"Available TF frames:\n{all_frames}")

        # ── Resolve base frame ────────────────────────────────────────────
        # Step 1: registry prefix match — but only if the frame actually exists
        lower = self.robot_name.lower()
        for prefix, template in ROBOT_BASE_FRAME_MAP.items():
            if lower.startswith(prefix):
                candidate = template.format(name=self.robot_name)
                if candidate in all_frames:
                    self._base_frame = candidate
                    self._log_info(
                        f"Base frame resolved via registry: '{candidate}' "
                        f"(matched prefix '{prefix}')"
                    )
                    break
                else:
                    self._log_warning(
                        f"Registry suggested '{candidate}' for prefix '{prefix}' "
                        f"but that frame is not in the TF tree. Falling through to probe."
                    )

        # Step 2: probe live TF tree with common candidates (if registry missed)
        if self._base_frame is None:
            for template in _FALLBACK_FRAMES:
                candidate = template.format(name=self.robot_name)
                if candidate in all_frames:
                    self._base_frame = candidate
                    self._log_info(f"Base frame resolved via TF probe: '{candidate}'")
                    break

        if self._base_frame is None:
            self._log_error(
                f"Could not resolve base frame for robot '{self.robot_name}'.\n"
                f"  Neither registry nor fallback candidates matched any TF frame.\n"
                f"  Pass base_frame= explicitly, e.g.:\n"
                f"    ObjectToRobot(node=self, robot_name='{self.robot_name}', "
                f"world_name=..., base_frame='<your_frame>')\n"
                f"  Or add an entry to ROBOT_BASE_FRAME_MAP.\n"
                f"  Available TF frames:\n{all_frames}"
            )
            return

        # ── Resolve world frame ───────────────────────────────────────────
        # The most reliable source for the world frame is the header.frame_id
        # of the pose/info message itself — Gazebo sets this to the actual
        # simulation world frame name. A robot's local TF tree may be rooted
        # at "odom" or "map" (nav stack frames), which are NOT the world frame.
        if self._pose_info_world_frame is not None:
            if self._pose_info_world_frame != self.world_frame:
                self._log_info(
                    f"World frame updated from pose/info header: "
                    f"'{self.world_frame}' → '{self._pose_info_world_frame}'"
                )
                self.world_frame = self._pose_info_world_frame
        else:
            self._log_warning(
                f"pose/info not yet received; keeping world_frame='{self.world_frame}'. "
                f"If TF transforms fail, ensure the bridge is running."
            )
        self._log_info(f"World frame: '{self.world_frame}'")

    # ── Pose math ─────────────────────────────────────────────────────────
    @staticmethod
    def _pose_to_matrix(pose: Pose) -> np.ndarray:
        """Convert a geometry_msgs/Pose to a 4×4 homogeneous transform matrix."""
        p = pose.position
        q = pose.orientation
        # Rotation matrix from quaternion (x, y, z, w)
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = [p.x, p.y, p.z]
        return T

    @staticmethod
    def _matrix_to_pose(T: np.ndarray) -> Pose:
        """Convert a 4×4 homogeneous transform matrix back to a Pose."""
        pose = Pose()
        pose.position.x = float(T[0, 3])
        pose.position.y = float(T[1, 3])
        pose.position.z = float(T[2, 3])
        # Extract quaternion from rotation matrix
        R = T[:3, :3]
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        pose.orientation.x = float(x)
        pose.orientation.y = float(y)
        pose.orientation.z = float(z)
        pose.orientation.w = float(w)
        return pose

    @staticmethod
    def _relative_pose(obj_world: Pose, robot_world: Pose) -> Pose:
        """
        Return the pose of obj_world expressed in the robot's coordinate frame.

        T_robot_to_object = T_world_to_robot⁻¹  ×  T_world_to_object
        """
        T_obj   = ObjectToRobot._pose_to_matrix(obj_world)
        T_robot = ObjectToRobot._pose_to_matrix(robot_world)
        T_rel   = np.linalg.inv(T_robot) @ T_obj
        return ObjectToRobot._matrix_to_pose(T_rel)

    # ── Logging helpers ───────────────────────────────────────────────────
    # ROS2 Humble raises ValueError('Logger severity cannot be changed between
    # calls') when the same Python call-site (identified by file+line number)
    # emits different log severities across invocations.  The only reliable fix
    # is one dedicated method per severity so each call-site is always the same
    # severity.  Never use a generic dispatcher with getattr() on Humble.

    def _log_info(self, msg: str) -> None:
        self._node.get_logger().info(f"[ObjectToRobot|{self.robot_name}] {msg}")

    def _log_warning(self, msg: str) -> None:
        self._node.get_logger().warning(f"[ObjectToRobot|{self.robot_name}] {msg}")

    def _log_error(self, msg: str) -> None:
        self._node.get_logger().error(f"[ObjectToRobot|{self.robot_name}] {msg}")

    def _log_result(self, object_name: str, entry: dict) -> None:
        p = entry["robot_pose"].position
        o = entry["robot_pose"].orientation
        msg = (
            f"'{object_name}' in '{self._base_frame}':\n"
            f"    position    x={p.x:+.4f}  y={p.y:+.4f}  z={p.z:+.4f}\n"
            f"    orientation x={o.x:+.4f}  y={o.y:+.4f}  z={o.z:+.4f}  w={o.w:+.4f}"
        )
        if "euler_deg" in entry:
            r, pi_, y = entry["euler_deg"]
            msg += f"\n    euler (RPY) roll={r}°  pitch={pi_}°  yaw={y}°"
        self._log_info(msg)


# ─────────────────────────────────────────────────────────────────────────────
#  Minimal demo node  (only runs when executed directly, not when imported)
# ─────────────────────────────────────────────────────────────────────────────
class _DemoRobotNode(Node):
    """
    Stand-alone demo: instantiates ObjectToRobot for one robot, queries once,
    then shuts down.  Run with:
        python3 object_to_robot.py --robot fr3_robot1 --world empty \
                                   --objects box1 cylinder1
    """

    def __init__(self, robot_name: str, world_name: str, object_names: list[str]):
        super().__init__(f"demo_{robot_name}")
        self._object_names = object_names

        self.transformer = ObjectToRobot(
            node       = self,
            robot_name = robot_name,
            world_name = world_name,
        )

        # Query once after a short spin to let subscriptions settle
        self.create_timer(0.5, self._query_once)

    def _query_once(self) -> None:
        results = self.transformer.get_poses(self._object_names)
        for obj, data in results.items():
            if "error" in data:
                self.get_logger().warning(f"{obj}: {data['error']}")
        raise SystemExit  # stop spinning after one query


def _demo_main() -> None:
    import argparse
    p = argparse.ArgumentParser(description="ObjectToRobot demo")
    p.add_argument("--robot",   "-r", default="fr3_robot1")
    p.add_argument("--world",   "-W", default="empty")
    p.add_argument("--objects", "-o", nargs="+", default=["box1"])
    args = p.parse_args()

    rclpy.init()
    node = _DemoRobotNode(args.robot, args.world, args.objects)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    _demo_main()