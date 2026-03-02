import os

class PepperAdapter:
    """
    Dual-mode Pepper adapter.

    agent_agnostic=True  -> dry-run (no ROS)
    agent_agnostic=False -> ROS-based control
    """

    def __init__(self, use_sim: bool, agent_agnostic: bool = False):
        self.use_sim = use_sim
        self.agent_agnostic = agent_agnostic

        self.ros_initialized = False
        self.node = None

        if self.agent_agnostic:
            print("[PepperAdapter] ROS-agnostic mode selected")

    # -----------------------------
    # Lifecycle management
    # -----------------------------
    def initialize(self):
        """
        Initialize ROS resources if needed.
        Called explicitly by Robot.py
        """
        if self.agent_agnostic:
            print("[PepperAdapter] initialize(): dry-run mode")
            return

        try:
            import rclpy
            from rclpy.node import Node

            if not rclpy.ok():
                rclpy.init()

            self.node = Node("pepper_adapter")
            self.ros_initialized = True

            print(f"[PepperAdapter] ROS initialized (USE_SIM={self.use_sim})")

        except Exception as e:
            print(f"[PepperAdapter] ROS init failed → fallback to agent-agnostic: {e}")
            self.agent_agnostic = True
            self.ros_initialized = False

    def shutdown(self):
        """
        Clean shutdown of ROS resources.
        """
        if self.agent_agnostic or not self.ros_initialized:
            return

        try:
            import rclpy

            if self.node is not None:
                self.node.destroy_node()
                self.node = None

            if rclpy.ok():
                rclpy.shutdown()

            self.ros_initialized = False
            print("[PepperAdapter] ROS shutdown complete")

        except Exception as e:
            print(f"[PepperAdapter] Error during shutdown: {e}")

    # -----------------------------
    # Dual-mode actions
    # -----------------------------
    def move_to_pose(self, xyz):
        if self.agent_agnostic:
            print(f"[DRY-RUN] Pepper move_to_pose {xyz}")
            return

        print(f"[ROS] Pepper moving to {xyz} (sim={self.use_sim})")
        # Example:
        # self.node.publish(...)

    def pick_and_place(self, pick, place):
        if self.agent_agnostic:
            print(f"[DRY-RUN] Pepper pick {pick} -> place {place}")
            return

        print(f"[ROS] Pepper pick {pick} -> place {place} (sim={self.use_sim})")
        # Implement ROS actions or SDK calls

    def grasp(self, obj):
        if self.agent_agnostic:
            print(f"[DRY-RUN] Pepper grasp {obj}")
            return

        print(f"[ROS] Pepper grasping {obj} (sim={self.use_sim})")
        # Pepper grasp logic here