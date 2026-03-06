import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy

from robots_adapters.ros_context import ROSContextManager

class FrankaAdapter:
    def __init__(self, robot_id, use_sim=True, mock=False):
        self.robot_id = robot_id
        self.namespace = f"/{robot_id}"
        self.node_name = f"{robot_id}_moveit_adapter"
        self.use_sim = use_sim
        self.mock = mock

        self.node: Node = None
        self.moveit: MoveItPy = None
        self.planning_component = None
        self.current_handle = None

        print(f"[FrankaAdapter] id={robot_id} sim={use_sim} mock={mock}")

    def initialize(self):
        if self.mock:
            print("[FrankaAdapter] MOCK mode — no ROS started")
            return

        ROSContextManager.acquire()
        self.node = Node(self.node_name, namespace=self.namespace)
        self.moveit = MoveItPy(node=self.node)
        self.planning_component = self.moveit.get_planning_component("panda_arm")
        print(f"[FrankaAdapter] MoveItPy ready for {self.robot_id}")

    def shutdown(self):
        if self.mock:
            return
        if self.node:
            self.node.destroy_node()
        ROSContextManager.release()

    # -------------------------------
    # Motion with async handle
    # -------------------------------
    def move_to_pose(self, xyz, wait=True):
        if self.mock:
            print(f"[MOCK {self.robot_id}] Move to {xyz}")
            time.sleep(1)
            return True

        pose = PoseStamped()
        pose.header.frame_id = "panda_link0"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
        pose.pose.orientation.w = 1.0

        self.planning_component.set_goal_state(pose_stamped_msg=pose, pose_link="panda_link8")
        plan = self.planning_component.plan()
        if not plan:
            print(f"[{self.robot_id}] Planning failed")
            return False

        self.current_handle = self.planning_component.execute_async(plan)
        if wait:
            while not self.current_handle.done():
                time.sleep(0.05)
            return self.current_handle.result()
        return self.current_handle

    def stop(self):
        if self.current_handle and not self.current_handle.done():
            self.current_handle.cancel()
            print(f"[{self.robot_id}] Motion preempted")

# -------------------------------
# Optional ROS Node entry point
# -------------------------------
def main(args=None):
    adapter = FrankaAdapter(robot_id="franka_1", use_sim=True, mock=False)
    adapter.initialize()
    adapter.move_to_pose([0.4, 0.2, 0.3])
    adapter.shutdown()