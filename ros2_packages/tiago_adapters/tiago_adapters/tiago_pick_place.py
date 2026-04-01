#!/usr/bin/env python3
"""
tiago_pick_and_place.py
-----------------------
Pick-and-place state machine for Tiago (arm + pal-gripper).
No MoveIt — motion is sent directly to ros2_control action servers.
Self-contained: no external utility module needed.

State machine
-------------
  HOME          → tuck arm, open gripper
  NAV_TO_PICK   → drive base toward object
  PRE_GRASP     → arm above object
  GRASP         → arm lowered onto object
  CLOSE         → close gripper
  LIFT          → raise to carry pose
  NAV_TO_PLACE  → drive to drop-off location
  PLACE         → lower arm to place surface
  OPEN          → open gripper
  RETRACT       → tuck arm back to home
  DONE

Interfaces
----------
  /{ns}/arm_controller/follow_joint_trajectory      (FollowJointTrajectory)
  /{ns}/gripper_controller/follow_joint_trajectory  (FollowJointTrajectory)
  /{ns}/mobile_base_controller/cmd_vel_unstamped    (geometry_msgs/Twist)
    ↑ PAL uses cmd_vel_unstamped, not cmd_vel

Drive timing
------------
  Uses time.perf_counter (wall clock) — NOT the ROS clock.
  With use_sim_time=True the ROS clock only advances while the executor
  is spinning. Blocking with time.sleep + get_clock().now() deadlocks.

Usage
-----
  ros2 run robots_adapters tiago_pick_and_place \\
      --ros-args -p robot_namespace:=tiago_robot1
"""

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Joint names — PAL Tiago with pal-gripper
# ---------------------------------------------------------------------------
ARM_JOINTS = [
    'arm_1_joint',
    'arm_2_joint',
    'arm_3_joint',
    'arm_4_joint',
    'arm_5_joint',
    'arm_6_joint',
    'arm_7_joint',
]

# pal-gripper: two parallel finger joints
GRIPPER_JOINTS = [
    'gripper_left_finger_joint',
    'gripper_right_finger_joint',
]

# ---------------------------------------------------------------------------
# Named arm poses (radians) — tune to match your Gazebo world geometry
# ---------------------------------------------------------------------------
POSES = {
    # Safe tucked-in carry pose
    'home':      [0.20, -1.34, -0.20,  1.94, -1.57,  1.37,  0.0],

    # Arm extended forward, gripper above object
    'pre_grasp': [0.20, -0.50,  0.0,   1.20, -1.57,  1.00,  0.0],

    # Arm lowered to contact the object
    'grasp':     [0.20, -0.30,  0.0,   0.90, -1.57,  0.90,  0.0],

    # Raised enough to clear obstacles while navigating
    'carry':     [0.20, -1.00, -0.20,  1.60, -1.57,  1.20,  0.0],

    # Arm extended to place the object on the target surface
    'place':     [0.20, -0.40,  0.0,   1.10, -1.57,  1.00,  0.0],
}

# pal-gripper finger travel (metres)
GRIPPER_OPEN   = [0.044, 0.044]
GRIPPER_CLOSED = [0.002, 0.002]


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class TiagoPickAndPlace(Node):

    def __init__(self):
        super().__init__('tiago_pick_and_place')

        self._ns = self.declare_parameter(
            'robot_namespace', 'tiago_robot1'
        ).value

        # ── Base publisher ────────────────────────────────────────────
        # PAL uses cmd_vel_unstamped (not cmd_vel).
        # Namespaced: safe to run multiple Tiago instances in parallel.
        self._cmd_vel = self.create_publisher(
            Twist,
            f'/{self._ns}/mobile_base_controller/cmd_vel_unstamped',
            10,
        )

        # ── Arm action client ─────────────────────────────────────────
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{self._ns}/arm_controller/follow_joint_trajectory',
        )

        # ── Gripper action client ─────────────────────────────────────
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{self._ns}/gripper_controller/follow_joint_trajectory',
        )

        self.get_logger().info(f'[{self._ns}] Waiting for action servers…')
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info(f'[{self._ns}] Servers ready — starting sequence')

        self._run_sequence()

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _run_sequence(self):

        self._state('HOME')
        self._arm('home',       3.0)
        self._gripper(GRIPPER_OPEN, 2.0)

        self._state('NAV_TO_PICK')
        self._drive(linear=0.3, angular=0.0, seconds=3.0)
        self._stop()

        self._state('PRE_GRASP')
        self._arm('pre_grasp',  3.0)

        self._state('GRASP')
        self._arm('grasp',      2.0)

        self._state('CLOSE')
        self._gripper(GRIPPER_CLOSED, 2.0)

        self._state('LIFT')
        self._arm('carry',      2.5)

        self._state('NAV_TO_PLACE')
        self._drive(linear=-0.3, angular=0.0, seconds=2.0)   # back up
        self._stop()
        self._drive(linear=0.0,  angular=0.5, seconds=3.14)  # rotate ~180°
        self._stop()
        self._drive(linear=0.3,  angular=0.0, seconds=3.0)   # advance to place spot
        self._stop()

        self._state('PLACE')
        self._arm('place',      3.0)

        self._state('OPEN')
        self._gripper(GRIPPER_OPEN, 2.0)

        self._state('RETRACT')
        self._arm('home',       3.0)

        self._state('DONE')
        self.get_logger().info(f'[{self._ns}] Pick-and-place complete ✓')

    # ------------------------------------------------------------------
    # Motion helpers
    # ------------------------------------------------------------------
    def _arm(self, pose_name: str, duration_sec: float):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._make_trajectory(
            ARM_JOINTS, POSES[pose_name], duration_sec
        )
        self._send_and_wait(self._arm_client, goal)

    def _gripper(self, positions: list, duration_sec: float):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._make_trajectory(
            GRIPPER_JOINTS, positions, duration_sec
        )
        self._send_and_wait(self._gripper_client, goal)

    def _drive(self, linear: float, angular: float, seconds: float,
               rate_hz: float = 20.0):
        """
        Publish a constant Twist for `seconds` of WALL-CLOCK time.
        Uses time.perf_counter — not the ROS clock — so it works correctly
        under use_sim_time=True.
        """
        twist = Twist()
        twist.linear.x  = float(linear)
        twist.angular.z = float(angular)
        deadline = time.perf_counter() + seconds
        while time.perf_counter() < deadline:
            self._cmd_vel.publish(twist)
            time.sleep(1.0 / rate_hz)

    def _stop(self, pause_sec: float = 0.3):
        self._cmd_vel.publish(Twist())
        time.sleep(pause_sec)

    # ------------------------------------------------------------------
    # Low-level utilities (self-contained)
    # ------------------------------------------------------------------
    def _make_trajectory(self, joint_names: list, positions: list,
                         duration_sec: float) -> JointTrajectory:
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)

        pt = JointTrajectoryPoint()
        pt.positions  = list(positions)
        pt.velocities = [0.0] * len(positions)

        secs     = int(duration_sec)
        nanosecs = int((duration_sec - secs) * 1e9)
        pt.time_from_start = Duration(sec=secs, nanosec=nanosecs)

        traj.points = [pt]
        return traj

    def _send_and_wait(self, client: ActionClient, goal):
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'[{self._ns}] Goal rejected by {client._action_name}'
            )
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

    def _state(self, name: str):
        self.get_logger().info(f'[{self._ns}] ── STATE: {name} ──')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TiagoPickAndPlace()
    rclpy.shutdown()


if __name__ == '__main__':
    main()