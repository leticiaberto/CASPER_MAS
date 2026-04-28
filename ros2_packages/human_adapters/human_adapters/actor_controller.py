# actor_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time


class ActorController(Node):
    def __init__(self):
        super().__init__('actor_controller')

        self.declare_parameter('actor_name', 'human_1')
        self.declare_parameter('world_name', 'backyard')

        self.pub = self.create_publisher(Pose, 'cmd_pose', 10)

    def wait_for_subscriber(self, timeout=30.0):
        """Wait until bridge is connected AND enough time for spawn."""
        self.get_logger().info('Waiting for bridge subscriber...')
        start = time.time()
        while self.pub.get_subscription_count() == 0:
            if time.time() - start > timeout:
                self.get_logger().error('Timed out waiting for bridge!')
                return False
            time.sleep(0.5)
        
        # Extra wait for actor to actually be spawned in Gazebo
        self.get_logger().info('Bridge connected, waiting for actor spawn...')
        time.sleep(6.0)  # must be > TimerAction period (5.0s)
        self.get_logger().info('Starting movement.')
        return True

    def move_to(self, x, y, z=0.0):
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        self.pub.publish(msg)
        self.get_logger().info(f'Moving to ({x}, {y})')

    def run_trajectory(self):
        if not self.wait_for_subscriber():
            return

        waypoints = [(2.0, 0.0), (2.0, 3.0), (0.0, 3.0), (0.0, 0.0)]

        for (x, y) in waypoints:
            self.move_to(x, y)
            self.get_logger().info('Travelling...')
            time.sleep(4.0)
            self.get_logger().info('Pausing at waypoint...')
            time.sleep(2.0)


def main():
    rclpy.init()
    node = ActorController()
    node.run_trajectory()
    rclpy.shutdown()