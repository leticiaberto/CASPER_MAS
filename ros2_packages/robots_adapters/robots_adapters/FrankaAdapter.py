import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from robots_adapters.ros_context import ROSContextManager

class FrankaAdapter:
    def __init__(self, robot_id, use_sim=True, mock=False):
        self.robot_id = robot_id
        self.namespace = f"/{robot_id}"  # namespace per robot
        self.node_name = f"{robot_id}_adapter"

        self.use_sim = use_sim
        self.mock = mock

        self.node: Node = None
        self.move_client: ActionClient = None
        self.goal_handle = None

        print(f"[FrankaAdapter] id={robot_id} sim={use_sim} mock={mock}")

    # -----------------------------
    # Initialization
    # -----------------------------
    def initialize(self):
        if self.mock:
            print(f"[{self.robot_id}] MOCK mode — no ROS node started")
            return

        # Acquire shared ROS context
        ROSContextManager.acquire()

        # Create node in the robot namespace
        self.node = Node(self.node_name, namespace=self.namespace)
        ROSContextManager.add_node(self.node)

        # Connect to MoveIt action server under the namespace
        move_action_name = f"{self.namespace}/move_action"
        self.move_client = ActionClient(self.node, MoveGroup, move_action_name)

        self.node.get_logger().info(f"[{self.robot_id}] Waiting for MoveGroup server at '{move_action_name}'...")
        self.move_client.wait_for_server()
        self.node.get_logger().info(f"[{self.robot_id}] Connected to MoveGroup server")

    # -----------------------------
    # Shutdown
    # -----------------------------
    def shutdown(self):
        if self.mock:
            return

        if self.node:
            ROSContextManager.remove_node(self.node)
            self.node.destroy_node()
        ROSContextManager.release()

    # -----------------------------
    # Motion commands
    # -----------------------------
    def move_to_pose(self, xyz, wait=True):
        if self.mock:
            print(f"[MOCK {self.robot_id}] Move to {xyz}")
            time.sleep(1)
            return True

        # Build goal message
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = "panda_arm"
        req.allowed_planning_time = 5.0

        # Position constraint
        constraint = Constraints()
        pos_constraint = PositionConstraint()
        pos_constraint.link_name = "panda_link8"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]

        pose = Pose()
        pose.position.x = xyz[0]
        pose.position.y = xyz[1]
        pose.position.z = xyz[2]
        pose.orientation.w = 1.0

        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(pose)
        pos_constraint.weight = 1.0

        constraint.position_constraints.append(pos_constraint)
        req.goal_constraints.append(constraint)
        goal_msg.request = req

        self.node.get_logger().info(f"[{self.robot_id}] Sending MoveGroup goal: {xyz}")

        # Send goal asynchronously
        send_goal_future = self.move_client.send_goal_async(goal_msg)

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        self.goal_handle = send_goal_future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().error(f"[{self.robot_id}] Goal rejected")
            return False

        # Get result future
        result_future = self.goal_handle.get_result_async()

        if wait:
            # Non-blocking spin loop for this node only
            rclpy.spin_until_future_complete(self.node, result_future)
            result = result_future.result().result
            return result

        return result_future

    # -----------------------------
    # Stop / cancel motion
    # -----------------------------
    def stop(self):
        if self.goal_handle and not self.goal_handle.done():
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future)
            print(f"[{self.robot_id}] Motion cancelled")


# -----------------------------
# Optional standalone test
# -----------------------------
def main(args=None):
    franka1 = FrankaAdapter("franka_1", use_sim=True, mock=False)
    franka1.initialize()
    # Move both robots concurrently
    franka1.move_to_pose([0.4, 0.1, 0.4], wait=True)
    # Add small sleep to observe mock logs
    time.sleep(1)
    franka1.stop()
    franka1.shutdown()