#!/usr/bin/env python3
"""
Drive the Tiago robot to pick one named Gazebo object and place it on another,
using TiagoAdapter directly (no Tiago wrapper class required).

The robot's pose is read automatically from Gazebo Fortress via the gz-ros2
bridge (/world/<world>/pose/info) — no spawn-pose parameter is needed.

Usage
-----
    # Pick tomato_2 and place it on bench_r2
    python3 robots/Tests/tiago_pick_and_place.py tomato_2 bench_r2

    # Multiple pairs in sequence
    python3 robots/Tests/tiago_pick_and_place.py tomato_1 bowl_1 tomato_2 bowl_1

    # Override robot / world name
    python3 robots/Tests/tiago_pick_and_place.py tomato_2 bench_r2 \\
        --robot tiago_robot1 --world backyard

    # Disable Gazebo ground-truth (fall back to wheel odometry — not recommended)
    python3 robots/Tests/tiago_pick_and_place.py tomato_2 bench_r2 --world ''

Prerequisite
------------
The gz-ros2 bridge must be running before this script:

    ros2 run ros_gz_bridge parameter_bridge \\
      /world/backyard/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \\
      /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from typing import Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
import ros2_path_setup  # noqa: F401

from utils import ROSUtils

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import rclpy.parameter

from TiagoAdapter import TiagoAdapter, RobotMode
from object_world_to_robot import ObjectToRobot
from sdf_surface_resolver import SdfSurfaceResolver

# ---------------------------------------------------------------------------
# Pick z calibration
# ---------------------------------------------------------------------------
# In Gazebo the world_pose.z of a model is its SDF frame origin.  Many
# object models (bottles, cups, boxes) define that origin at their *base*
# rather than their geometric centre.  If we target that z directly the
# gripper descends to the object's bottom face.
#
# _PICK_Z_LIFT shifts the grasp target upward so the gripper centres on the
# object body.  Tune this value for your specific object set:
#   • Small spheres / tomatoes (r ≈ 4 cm)  →  0.04 m
#   • Cylinders / cups (h ≈ 12 cm)         →  0.06 m  (default)
#   • Tall bottles   (h ≈ 20 cm)           →  0.10 m
_PICK_Z_LIFT: float = 0.06

def _resolve_xyz(
    otr:        ObjectToRobot,
    surface:    SdfSurfaceResolver,
    name:       str,
    use_center: bool = False,
) -> Optional[tuple]:
    """
    Return (x, y, z) in the world frame for the named Gazebo object.

    Parameters
    ----------
    use_center : bool
        True  → return the object's Gazebo world_pose z, i.e. its geometric
                 centre.  Use this for the **pick** target so the gripper
                 grasps the object at mid-height with a top-down approach.
        False → return the top-of-surface z from SdfSurfaceResolver, i.e.
                 the height at which a placed object would rest on the
                 destination surface.  Use this for the **place** target.
    """
    result = otr.get_pose(name)
    if "error" in result:
        print(f"[pick_and_place] ERROR: Cannot resolve pose of '{name}': "
              f"{result['error']}")
        return None

    if use_center:
        # Object centre directly from Gazebo — the natural grasp point for a
        # top-down approach.  Add _PICK_Z_LIFT to compensate for models whose
        # SDF origin is at the base rather than the geometric centre.
        pos = result["world_pose"].position
        xyz = (pos.x, pos.y, pos.z + _PICK_Z_LIFT)
        print(f"[pick_and_place]   '{name}' world XYZ (centre+lift): "
              f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})")
        return xyz

    # Surface-adjusted z for placing: object should rest on top of the surface.
    xyz = surface.place_xyz(result, name)
    if xyz is None:
        pos = result["world_pose"].position
        xyz = (pos.x, pos.y, pos.z)

    print(f"[pick_and_place]   '{name}' world XYZ (surface): "
          f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})")
    return xyz


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Pick-and-place using TiagoAdapter directly.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python3 robots/Tests/tiago_pick_and_place.py tomato_2 bench_r2\n"
            "  python3 robots/Tests/tiago_pick_and_place.py tomato_1 bowl_1 tomato_2 bowl_1\n"
        ),
    )
    parser.add_argument(
        "pairs", nargs="+", metavar="NAME",
        help="Alternating pick/place object names (must be even count).",
    )
    parser.add_argument("--robot",   default="tiago_robot1",
                        help="Gazebo robot model name (default: tiago_robot1)")
    parser.add_argument(
        "--world", default="backyard",
        help=(
            "Gazebo world name (default: backyard). "
            "Used for ground-truth pose (/world/<n>/pose/info) and SDF loading. "
            "Pass '' to disable ground-truth and use odometry only."
        ),
    )
    parser.add_argument("--timeout", type=float, default=600.0,
                        help="Per-task timeout in seconds (default: 600)")
    args = parser.parse_args()

    # Validate pairs
    if len(args.pairs) % 2 != 0:
        parser.error(
            f"Expected an even number of names (pick/place pairs), "
            f"got {len(args.pairs)}: {args.pairs}"
        )
    pairs      = [(args.pairs[i], args.pairs[i + 1])
                  for i in range(0, len(args.pairs), 2)]
    world_name = args.world or None   # empty string -> None -> odometry mode

    # ---- Resolve paths --------------------------------------------------
    sdf_path = models_dir = None
    try:
        sdf_path   = ROSUtils._get_sdf_path(world_name)
        models_dir = ROSUtils._get_models_dir()
    except FileNotFoundError as exc:
        print(f"[pick_and_place] WARNING: {exc}")
        print("[pick_and_place] Continuing without SDF (no table-aware nav).")

    # ---- Banner ---------------------------------------------------------
    print()
    print("=" * 60)
    print("  tiago_pick_and_place.py")
    print("=" * 60)
    print(f"  Robot    : {args.robot}")
    print(f"  World    : {world_name or '(odometry mode)'}")
    print(f"  Pose src : {'Gazebo ground truth (auto)' if world_name else 'wheel odometry'}")
    print(f"  SDF      : {sdf_path or 'not found'}")
    print(f"  Tasks    : {len(pairs)}")
    for i, (pick, place) in enumerate(pairs):
        print(f"    [{i+1}] pick '{pick}'  ->  place on '{place}'")
    print("=" * 60)
    print()

    # ---- ROS2 init ------------------------------------------------------
    rclpy.init()

    # ObjectToRobot shares a separate node so its subscriptions are isolated.
    otr_node = Node(
        f"otr_{args.robot}",
        parameter_overrides=[
            rclpy.parameter.Parameter(
                "use_sim_time",
                rclpy.parameter.Parameter.Type.BOOL,
                True,
            )
        ],
    )

    otr = ObjectToRobot(
        node       = otr_node,
        robot_name = args.robot,
        world_name = world_name or "backyard",
    )

    surface = SdfSurfaceResolver(sdf_path or "", verbose=False)

    done_event  = threading.Event()
    result_box: list = []

    def on_result(success: bool, message: str) -> None:
        result_box.append((success, message))
        done_event.set()

    adapter = TiagoAdapter(
        robot_name      = args.robot,
        mode            = RobotMode.SIMULATION,
        result_callback = on_result,
        world_sdf_path  = sdf_path,
        models_base_dir = models_dir,
        table_standoff  = 0.10,
        world_name      = world_name,   # activates GT pose — no spawn param needed
    )

    executor = MultiThreadedExecutor()
    executor.add_node(otr_node)
    executor.add_node(adapter)
    executor.add_node(adapter._gripper)
    threading.Thread(target=executor.spin, daemon=True).start()

    # ---- Wait for ground-truth pose -------------------------------------
    src = (f"/world/{world_name}/pose/info" if world_name
           else f"/{args.robot}/mobile_base_controller/odom")
    print(f"[pick_and_place] Waiting for pose from {src} ...")
    if not adapter._pose_ready.wait(timeout=15.0):
        print("[pick_and_place] FAIL: No pose after 15 s.")
        if world_name:
            print("[pick_and_place] Is the gz-ros2 bridge running?")
            print(f"[pick_and_place]   ros2 run ros_gz_bridge parameter_bridge \\")
            print(f"[pick_and_place]     {src}"
                  f"@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \\")
            print(f"[pick_and_place]     /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")
        rclpy.shutdown()
        sys.exit(1)

    using_gt = adapter._use_ground_truth
    if using_gt:
        import math
        wx, wy, wth = adapter._get_robot_pose()
        print(f"[pick_and_place] Ground-truth active: "
              f"({wx:.3f}, {wy:.3f}, {math.degrees(wth):.1f}deg)")
    else:
        print("[pick_and_place] WARNING: Odometry active — check gz-ros2 bridge.")
    print()

    # ---- Execute tasks --------------------------------------------------
    all_ok = True
    for task_idx, (pick_name, place_name) in enumerate(pairs):
        print(f"[pick_and_place] ── Task {task_idx+1}/{len(pairs)}: "
              f"pick '{pick_name}' -> place on '{place_name}' ──")

        # pick_xyz  : object centre (Gazebo world_pose z) — top-down grasp at mid-height.
        # place_xyz : top of the destination surface — object deposited on the surface.
        pick_xyz  = _resolve_xyz(otr, surface, pick_name,  use_center=True)
        place_xyz = _resolve_xyz(otr, surface, place_name, use_center=False)

        if pick_xyz is None or place_xyz is None:
            print(f"[pick_and_place] SKIP: Could not resolve pose for task {task_idx+1}.")
            all_ok = False
            continue

        # Run pick and place
        done_event.clear()
        result_box.clear()

        t0       = time.perf_counter()
        accepted = adapter.pick_and_place(pick_xyz=pick_xyz, place_xyz=place_xyz)

        if not accepted:
            print(f"[pick_and_place] FAIL: Adapter rejected task {task_idx+1} "
                  f"(is it already busy?).")
            all_ok = False
            continue

        print(f"[pick_and_place] Task running — timeout {args.timeout:.0f} s ...")
        finished = done_event.wait(timeout=args.timeout)
        elapsed  = time.perf_counter() - t0

        if not finished:
            print(f"[pick_and_place] FAIL: Task {task_idx+1} timed out "
                  f"after {args.timeout:.0f} s.")
            adapter.cancel()
            all_ok = False
            continue

        success, message = result_box[0]
        status = "OK" if success else "FAIL"
        print(f"[pick_and_place] [{status}] Task {task_idx+1} in {elapsed:.1f} s: "
              f"{message}")
        print()
        if not success:
            all_ok = False

    # ---- Shutdown -------------------------------------------------------
    print(f"[pick_and_place] All tasks done. Overall: "
          f"{'SUCCESS' if all_ok else 'SOME TASKS FAILED'}")

    try:
        adapter.cancel()
    except Exception:
        pass
    time.sleep(0.5)
    try:
        executor.shutdown(timeout_sec=2.0)
        for node in (adapter._gripper, adapter, otr_node):
            try:
                node.destroy_node()
            except Exception:
                pass
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass

    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
