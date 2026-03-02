import sys

class FrankaMoveItAdapter:
    """
    Dual-mode Franka adapter.

    agent_agnostic=True  -> dry-run (no ROS / MoveIt)
    agent_agnostic=False -> ROS + MoveIt control
    """

    def __init__(self, use_sim: bool, agent_agnostic: bool):
        self.use_sim = use_sim
        self.agent_agnostic = agent_agnostic

        self.ros_initialized = False
        self.arm = None
        self.gripper = None

        print(f"[FrankaMoveItAdapter] constructed (use_sim={self.use_sim}, "
              f"agent_agnostic={self.agent_agnostic})")

    # -----------------------------
    # Lifecycle management
    # -----------------------------
    def initialize(self):
        """
        Initialize ROS + MoveIt resources.
        Called explicitly by Robot.py
        """
        if self.agent_agnostic:
            print("[FrankaMoveItAdapter] initialize(): dry-run mode")
            return

        try:
            import rclpy
            from moveit_commander import (
                MoveGroupCommander,
                roscpp_initialize,
            )

            # Initialize ROS / MoveIt
            if not rclpy.ok():
                rclpy.init()

            roscpp_initialize(sys.argv)

            self.arm = MoveGroupCommander("panda_arm")
            self.gripper = MoveGroupCommander("hand")

            # Safety tuning
            if self.use_sim:
                self.arm.set_max_velocity_scaling_factor(0.3)
            else:
                self.arm.set_max_velocity_scaling_factor(0.05)
                self.arm.set_max_acceleration_scaling_factor(0.05)

            self.ros_initialized = True
            print("[FrankaMoveItAdapter] ROS + MoveIt initialized")

        except Exception as e:
            print(f"[FrankaMoveItAdapter] Initialization failed → "
                  f"fallback to dry-run: {e}")
            self.agent_agnostic = True
            self.ros_initialized = False

    def shutdown(self):
        """
        Clean shutdown of MoveIt and ROS.
        """
        if self.agent_agnostic or not self.ros_initialized:
            return

        try:
            from moveit_commander import roscpp_shutdown
            import rclpy

            roscpp_shutdown()

            if rclpy.ok():
                rclpy.shutdown()

            self.arm = None
            self.gripper = None
            self.ros_initialized = False

            print("[FrankaMoveItAdapter] ROS + MoveIt shutdown complete")

        except Exception as e:
            print(f"[FrankaMoveItAdapter] Shutdown error: {e}")

    # -----------------------------
    # Dual-mode actions
    # -----------------------------
    def move_to_pose(self, xyz):
        if self.agent_agnostic:
            print(f"[DRY-RUN] Franka move_to_pose {xyz}")
            return

        from geometry_msgs.msg import Pose

        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x, pose.position.y, pose.position.z = xyz

        self.arm.set_pose_target(pose)
        self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()

    def pick_and_place(self, pick, place):
        """franka.pick_and_place(
            pick=[0.5, 0.0, 0.2],
            place=[0.3, -0.2, 0.2]
        )"""
        if self.agent_agnostic:
            print(f"[DRY-RUN] Franka pick {pick} -> place {place}")
            return

        # Open gripper
        self.gripper.go([0.04, 0.04])

        # Approach pick
        self.move_to_pose([pick[0], pick[1], pick[2] + 0.15])
        self.move_to_pose(pick)

        # Close gripper
        self.gripper.go([0.0, 0.0])

        # Retreat
        self.move_to_pose([pick[0], pick[1], pick[2] + 0.15])

        # Move to place
        self.move_to_pose([place[0], place[1], place[2] + 0.15])
        self.move_to_pose(place)

        # Release
        self.gripper.go([0.04, 0.04])

    def grasp(self, obj):
        if self.agent_agnostic:
            print(f"[DRY-RUN] Franka grasp {obj}")
            return

        self.gripper.go([0.0, 0.0])
        print(f"[ROS] Franka grasping {obj}")