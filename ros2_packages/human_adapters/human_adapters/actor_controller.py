import rclpy
from rclpy.node import Node
import subprocess
import time
import math


class ActorController(Node):
    def __init__(self):
        super().__init__('actor_controller')

        self.declare_parameter('actor_name', 'human_1')
        self.declare_parameter('world_name', 'backyard')

        self.actor_name = self.get_parameter('actor_name').get_parameter_value().string_value
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value

        self.current_x = 0.0
        self.current_y = 0.0
        self.z = 0.0
        self.current_yaw = 0.0  # track current yaw for smooth turning

        self.get_logger().info(f'Actor: {self.actor_name}, World: {self.world_name}')

    def yaw_to_quaternion(self, yaw):
        """Convert yaw (radians) to quaternion (x, y, z, w)."""
        w = math.cos(yaw / 2.0)
        z = math.sin(yaw / 2.0)
        return 0.0, 0.0, z, w

    def set_pose(self, x, y, yaw):
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        req = (
            f'name: "{self.actor_name}", '
            f'position: {{x: {float(x)}, y: {float(y)}, z: {self.z}}}, '
            f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        )
        subprocess.run([
            'ign', 'service',
            '-s', f'/world/{self.world_name}/set_pose',
            '--reqtype', 'ignition.msgs.Pose',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000',
            '--req', req
        ])

    def angle_diff(self, target, current):
        """Shortest angular difference, handles wrap-around."""
        diff = (target - current + math.pi) % (2 * math.pi) - math.pi
        return diff

    def move_to(self, target_x, target_y, speed=0.5, step_delay=0.05, turn_speed=2.0):
        """
        Smoothly move to target with orientation facing direction of travel.
        turn_speed: radians per second for turning
        """
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.01:
            return

        # Target yaw: atan2 gives angle from x-axis
        target_yaw = math.atan2(dy, dx)

        self.get_logger().info(
            f'Moving to ({target_x}, {target_y}), '
            f'distance={distance:.2f}m, '
            f'yaw={math.degrees(target_yaw):.1f}deg'
        )

        # --- Phase 1: Turn to face target ---
        yaw_diff = self.angle_diff(target_yaw, self.current_yaw)
        turn_steps = int(abs(yaw_diff) / (turn_speed * step_delay))

        for i in range(turn_steps):
            t = (i + 1) / max(turn_steps, 1)
            yaw = self.current_yaw + yaw_diff * t
            self.set_pose(self.current_x, self.current_y, yaw)
            time.sleep(step_delay)

        self.current_yaw = target_yaw

        # --- Phase 2: Walk forward ---
        step_dist = speed * step_delay
        num_steps = int(distance / step_dist)

        for i in range(num_steps):
            t = (i + 1) / num_steps
            x = self.current_x + dx * t
            y = self.current_y + dy * t
            self.set_pose(x, y, self.current_yaw)
            time.sleep(step_delay)

        self.set_pose(target_x, target_y, self.current_yaw)
        self.current_x = target_x
        self.current_y = target_y

    def run_trajectory(self):
        time.sleep(3.0)

        waypoints = [(2.0, 0.0), (2.0, 3.0), (0.0, 3.0), (0.0, 0.0)]

        while True:
            for (x, y) in waypoints:
                self.move_to(x, y, speed=0.5, turn_speed=2.0)
                self.get_logger().info(f'Reached ({x}, {y}), pausing...')
                time.sleep(2.0)


def main():
    rclpy.init()
    node = ActorController()
    node.run_trajectory()
    rclpy.shutdown()