#!/usr/bin/env python3
"""
Standalone navigation-only test for TiagoNavigator.

Usage
-----
    python3 robots/Tests/tiago_navigation.py
    python3 robots/Tests/tiago_navigation.py --target -5.0 5.20 0.910
    python3 robots/Tests/tiago_navigation.py --no-sdf
    python3 robots/Tests/tiago_navigation.py --poll-hz 1.0
    python3 robots/Tests/tiago_navigation.py --mode drive_to
    python3 robots/Tests/tiago_navigation.py --world-name ''   # fall back to odometry

The robot's pose is read automatically from Gazebo Fortress via
/world/<world_name>/pose/info.

NOTE on XY / heading metrics
    XY err vs approach - distance from final position to the APPROACH POSE.
                         Success = < 0.15 m.  The arm reaches the target from there.
    XY err vs target   - distance to the raw object; never reaches 0 and is
                         NOT the success criterion.
    Heading error      - angle between final heading and direction toward target.
                         Success = < 5 deg.
    The 'd_approach' column in the live log shows the true closing distance.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from typing import Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
import ros2_path_setup  # noqa: F401

from utils import ROSUtils

import rclpy
from rclpy.executors import MultiThreadedExecutor

from tiago_navigator import TiagoNavigator, _N_APPROACH_ANGLES

NavPose = Tuple[float, float, float]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _fmt(x: float, y: float, th: Optional[float] = None) -> str:
    if th is not None:
        return f"x={x:+.3f}  y={y:+.3f}  th={math.degrees(th):+.1f}deg"
    return f"x={x:+.3f}  y={y:+.3f}"


def _wrap(a: float, b: float) -> float:
    """Signed angular difference (a - b) wrapped to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


# ---------------------------------------------------------------------------
# Pre-flight internals dump
# Returns the chosen approach pose (world frame) for the final report.
# ---------------------------------------------------------------------------

def _dump_internals(
    nav:        TiagoNavigator,
    target_xyz: Tuple[float, float, float],
) -> Optional[NavPose]:
    tx, ty, tz  = target_xyz
    robot_world = nav.get_pose()
    rx, ry, rth = robot_world

    print()
    print("-" * 62)
    print("  NAVIGATOR INTERNALS DUMP")
    print("-" * 62)
    using_gt = nav._use_ground_truth
    print(f"  Pose source      : {'Gazebo ground truth (no drift)' if using_gt else 'Wheel odometry (DRIFT WARNING)'}")
    print(f"  Robot world pose : {_fmt(rx, ry, rth)}")
    print(f"  Target world     : ({tx:.3f}, {ty:.3f}, {tz:.3f})")

    reachable = nav._is_reachable(target_xyz, robot_world)
    print(f"\n  Already reachable from current pose: {reachable}")

    # ---- Table resolver -------------------------------------------------
    if nav._table_resolver is not None:
        tables = nav._table_resolver.known_tables()
        print(f"\n  Table resolver: {len(tables)} tables:")
        for t in tables:
            fp = nav._table_resolver.get_footprint(t)
            if fp:
                print(f"    {t:<25}  centre=({fp.centre_x:.2f}, {fp.centre_y:.2f})"
                      f"  half=({fp.half_x:.2f}, {fp.half_y:.2f})")
        found = nav._table_resolver.find_table_for_object((tx, ty))
        print(f"\n  find_table_for_object({tx:.2f}, {ty:.2f}) -> '{found}'")
        if found:
            sp = nav._table_resolver.approach_pose_for_object(
                object_xy=(tx, ty), robot_xy=(rx, ry), table_name=found,
            )
            if sp:
                print(f"  Table standoff pose : ({sp[0]:.3f}, {sp[1]:.3f},"
                      f" {math.degrees(sp[2]):.1f}deg)")
                print(f"  Standoff reachable? : {nav._is_reachable(target_xyz, sp)}")
            else:
                print("  Table standoff pose : None")
    else:
        print("\n  Table resolver : NOT loaded (--no-sdf)")

    # ---- Angle-sweep candidates ----------------------------------------
    print(f"\n  Angle-sweep (preferred_reach={nav._preferred_reach:.2f} m):")
    base_angle = math.atan2(ry - ty, rx - tx)
    for i in range(_N_APPROACH_ANGLES):
        angle     = base_angle + i * (2.0 * math.pi / _N_APPROACH_ANGLES)
        nav_x     = tx + nav._preferred_reach * math.cos(angle)
        nav_y     = ty + nav._preferred_reach * math.sin(angle)
        nav_theta = math.atan2(ty - nav_y, tx - nav_x)
        ok        = nav._is_reachable(target_xyz, (nav_x, nav_y, nav_theta))
        d         = math.hypot(nav_x - rx, nav_y - ry)
        dh        = math.atan2(nav_y - ry, nav_x - rx) if d > 1e-3 else nav_theta
        fa        = abs(_wrap(nav_theta, dh))
        cost      = 10.0 * fa + d
        mark      = "OK" if ok else "--"
        print(f"    [{i:2d}] {mark}  ({nav_x:+.2f}, {nav_y:+.2f},"
              f" {math.degrees(nav_theta):+.1f}deg)"
              f"  d={d:.2f}  align={math.degrees(fa):.1f}deg  cost={cost:.2f}")

    # ---- Final chosen approach pose ------------------------------------
    nav_pose = nav._compute_approach_pose(target_xyz, robot_world)
    if nav_pose is None:
        print("\n  _compute_approach_pose -> None (already reachable)")
        nav_pose = robot_world
    else:
        npx, npy, npth = nav_pose
        arm_reach = math.hypot(tx - npx, ty - npy)
        print(f"\n  CHOSEN approach pose (world) : ({npx:.3f}, {npy:.3f},"
              f" {math.degrees(npth):.1f}deg)")
        print(f"  Distance robot->approach     : {math.hypot(npx-rx, npy-ry):.3f} m")
        print(f"  Distance approach->target    : {arm_reach:.3f} m  (arm reach needed)")

    print("-" * 62)
    print()
    return nav_pose


# ---------------------------------------------------------------------------
# Live pose reporter thread
# ---------------------------------------------------------------------------

def _make_reporter(
    nav:           TiagoNavigator,
    target_xyz:    Tuple[float, float, float],
    approach_pose: Optional[NavPose],
    done_event:    threading.Event,
    poll_hz:       float,
) -> threading.Thread:
    tx, ty, _ = target_xyz
    ap_x = approach_pose[0] if approach_pose else tx
    ap_y = approach_pose[1] if approach_pose else ty
    interval = 1.0 / max(0.1, poll_hz)
    t0 = time.perf_counter()

    def _run() -> None:
        while not done_event.wait(timeout=interval):
            wx, wy, wth = nav.get_pose()
            ox, oy, oth = nav._get_pose()
            d_target    = math.hypot(tx - wx, ty - wy)
            d_approach  = math.hypot(ap_x - wx, ap_y - wy)
            bearing     = math.atan2(ty - wy, tx - wx)
            hdg_err     = math.degrees(abs(_wrap(bearing, wth)))
            elapsed     = time.perf_counter() - t0
            print(
                f"[{elapsed:5.1f}s]"
                f"  world ({wx:+.3f}, {wy:+.3f}, {math.degrees(wth):+.1f}deg)"
                f"  odom ({ox:+.3f}, {oy:+.3f}, {math.degrees(oth):+.1f}deg)"
                f"  d_target={d_target:.3f}m"
                f"  d_approach={d_approach:.3f}m"
                f"  hdg_err={hdg_err:.1f}deg"
            )

    return threading.Thread(target=_run, daemon=True)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Standalone TiagoNavigator test -- no arm, no gripper."
    )
    parser.add_argument("--robot", default="tiago_robot1")
    parser.add_argument(
        "--target", nargs=3, type=float, metavar=("X", "Y", "Z"),
        default=[-5.0, 5.20, 0.910],
        help="World-frame target XYZ (default: -5.0 5.20 0.910)",
    )
    parser.add_argument(
        "--world-name", default="backyard", dest="world_name",
        help=(
            "Gazebo world name (default: backyard). "
            "Used to subscribe to /world/<name>/pose/info for ground-truth pose. "
            "Pass --world-name '' to fall back to wheel odometry."
        ),
    )
    parser.add_argument(
        "--mode",
        choices=["navigate_to", "drive_to", "drive_to_reach"],
        default="navigate_to",
    )
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--poll-hz", type=float, default=0.5, dest="poll_hz")
    parser.add_argument("--no-sdf",  action="store_true",
                        help="Skip table resolver; use angle-sweep only")
    parser.add_argument("--no-dump", action="store_true",
                        help="Skip pre-flight internals dump")
    args = parser.parse_args()

    tx, ty, tz = args.target
    target_xyz = (tx, ty, tz)
    world_name = args.world_name or None   # empty string → None → odom mode

    # ---- Banner ---------------------------------------------------------
    print()
    print("=" * 62)
    print("  tiago_navigation.py -- TiagoNavigator standalone test")
    print("=" * 62)
    print(f"  Robot      : {args.robot}")
    print(f"  Mode       : {args.mode}")
    print(f"  Target     : ({tx:.3f}, {ty:.3f}, {tz:.3f})  [world frame]")
    print(f"  Table SDF  : {'disabled (--no-sdf)' if args.no_sdf else 'enabled'}")
    if world_name:
        print(f"  Pose src   : Gazebo ground truth (/world/{world_name}/pose/info)")
    else:
        print(f"  Pose src   : Wheel odometry (DRIFT WARNING -- pass --world-name)")
    print("=" * 62)
    print()

    # ---- ROS2 init ------------------------------------------------------
    rclpy.init()

    sdf_path = models_dir = None
    if not args.no_sdf:
        try:
            sdf_path   = ROSUtils._get_sdf_path(world_name)
            models_dir = ROSUtils._get_models_dir()
            print(f"[test] SDF path   : {sdf_path}")
            print(f"[test] Models dir : {models_dir}")
        except FileNotFoundError as exc:
            print(f"[test] WARNING: {exc}")
            print("[test] Falling back to angle-sweep only.\n")

    navigator = TiagoNavigator(
        robot_name      = args.robot,
        world_sdf_path  = sdf_path,
        models_base_dir = models_dir,
        table_standoff  = 0.10,
        world_name      = world_name,   # activates GT pose
    )

    executor = MultiThreadedExecutor()
    executor.add_node(navigator)
    threading.Thread(target=executor.spin, daemon=True).start()

    # ---- Wait for first pose (GT preferred, falls back to odom) --------
    src_label = (f"Gazebo GT (/world/{world_name}/pose/info)"
                 if world_name else "wheel odometry")
    print(f"[test] Waiting for first pose from {src_label} ...")
    if not navigator.wait_for_pose(timeout=15.0):
        print("[test] FAIL: No pose after 15 s.")
        if world_name:
            print(f"[test]   Is the gz-ros2 bridge running for world '{world_name}'?")
            print(f"[test]   Start it with:")
            print(f"[test]   ros2 run ros_gz_bridge parameter_bridge \\")
            print(f"[test]     /world/{world_name}/pose/info"
                  f"@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \\")
            print(f"[test]     /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock")
        rclpy.shutdown()
        return

    wx, wy, wth = navigator.get_pose()
    using_gt    = navigator._use_ground_truth
    print(f"[test] First pose received  [{'GROUND TRUTH' if using_gt else 'ODOMETRY -- DRIFT WARNING'}]:")
    print(f"         world : {_fmt(wx, wy, wth)}")
    if not using_gt and world_name:
        print("[test] WARNING: GT subscription present but no message yet.")
        print("[test]          Continuing on odometry -- results may drift.")

    # ---- Pre-flight dump (also captures approach pose) ------------------
    approach_pose: Optional[NavPose] = None
    if not args.no_dump:
        approach_pose = _dump_internals(navigator, target_xyz)
    else:
        rp = navigator.get_pose()
        approach_pose = navigator._compute_approach_pose(target_xyz, rp) or rp

    # ---- Live reporter --------------------------------------------------
    done_event = threading.Event()
    reporter   = _make_reporter(navigator, target_xyz, approach_pose,
                                done_event, args.poll_hz)
    reporter.start()

    # ---- Launch navigation ----------------------------------------------
    result = [None, None]   # [success, message]

    def on_done(success: bool, message: str) -> None:
        result[0] = success
        result[1] = message
        done_event.set()

    t0 = time.perf_counter()
    print(f"\n[test] Starting navigation (mode='{args.mode}') ...\n")

    if args.mode == "navigate_to":
        accepted = navigator.navigate_to(tx, ty, on_done, target_z=tz)

    elif args.mode == "drive_to_reach":
        accepted = navigator.drive_to_reach(target_xyz, on_done)

    else:   # drive_to
        nav_pose = approach_pose or navigator.get_pose()
        npx, npy, npth = nav_pose
        print(f"[test] drive_to ({npx:.3f}, {npy:.3f},"
              f" {math.degrees(npth):.1f}deg)  nav_target_xy=({tx:.3f}, {ty:.3f})")
        accepted = navigator.drive_to(npx, npy, npth, on_done, nav_target_xy=(tx, ty))

    if not accepted:
        print("[test] FAIL: Navigator rejected goal (already busy?).")
        done_event.set()

    # ---- Wait -----------------------------------------------------------
    finished = done_event.wait(timeout=args.timeout)
    elapsed  = time.perf_counter() - t0
    done_event.set()
    time.sleep(0.3)

    # ---- Final report ---------------------------------------------------
    print()
    print("-" * 62)

    if not finished:
        print(f"[test] FAIL: Timed out after {args.timeout:.0f} s.")
        navigator.cancel()

    elif not result[0]:
        print(f"[test] FAIL: Navigation failed -- {result[1]}")

    else:
        fx, fy, fth = navigator.get_pose()

        ap_x  = approach_pose[0] if approach_pose else tx
        ap_y  = approach_pose[1] if approach_pose else ty
        ap_th = approach_pose[2] if approach_pose else 0.0

        # XY error vs the APPROACH POSE (what the robot actually aimed for)
        xy_err_ap  = math.hypot(ap_x - fx, ap_y - fy)
        # Distance to raw target (informational; never 0)
        xy_err_tgt = math.hypot(tx - fx, ty - fy)
        arm_dist   = math.hypot(tx - ap_x, ty - ap_y)

        # Heading error: final heading vs direction toward TARGET
        bearing = math.atan2(ty - fy, tx - fx)
        hdg_err = math.degrees(abs(_wrap(bearing, fth)))

        xy_ok  = xy_err_ap < 0.15
        hdg_ok = hdg_err   < 5.0

        print(f"[test] SUCCESS: Navigated in {elapsed:.1f} s")
        print()
        print(f"  Final pose    (world) : {_fmt(fx, fy, fth)}")
        print()
        print(f"  Approach pose (world) : {_fmt(ap_x, ap_y, ap_th)}")
        print(f"  Target        (world) : ({tx:.3f}, {ty:.3f}, {tz:.3f})")
        print()
        print(f"  XY err vs approach : {xy_err_ap:.3f} m  "
              f"tol 0.15 m  {'PASS' if xy_ok  else 'FAIL'}")
        print(f"  XY err vs target   : {xy_err_tgt:.3f} m  "
              f"(approach is {arm_dist:.2f} m from target -- expected)")
        print(f"  Heading error      : {hdg_err:.1f}deg  "
              f"tol 5deg    {'PASS' if hdg_ok else 'FAIL'}")

        if not xy_ok:
            print()
            print("  [XY FAIL] Robot did not stop within 0.15 m of the approach pose.")
            print("  Check 'd_approach' in the live log above. If it plateaued above")
            print("  0.15 m the unicycle controller overshot.")
            print("  Try reducing _K_LINEAR in tiago_navigator.py.")

        if not hdg_ok:
            print()
            print("  [HDG FAIL] Final heading does not face the target.")
            print("  Phase 3 may not have received nav_target_xy.")
            print("  Re-run with --mode drive_to and check the log.")

    print("-" * 62)
    print()

    # ---- Cleanup --------------------------------------------------------
    try:
        navigator.cancel()
    except Exception:
        pass
    time.sleep(0.5)
    try:
        executor.shutdown(timeout_sec=2.0)
        navigator.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
