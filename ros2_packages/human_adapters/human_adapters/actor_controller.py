"""
ActorController — command-driven human actor for Gazebo / Ignition.

Uses only standard message types (no custom .srv needed).

Topics
------
  Subscribed:
    /<actor_name>/actor_command   (std_msgs/String)
      JSON payload:
        {"command": "goto", "x": 3.0, "y": 2.0, "final_yaw": 1.57}
        {"command": "stop"}
        {"command": "test"}
        {"command": "finish"}

  Published:
    /<actor_name>/actor_result    (std_msgs/String)
      JSON payload published once each command completes:
        {"success": true,  "command": "goto",   "message": "Reached (3.00, 2.00), facing 90.0°."}
        {"success": false, "command": "goto",   "message": "goto interrupted by stop."}
        {"success": true,  "command": "stop",   "message": "Actor stopped."}
        {"success": true,  "command": "finish", "message": "Shutting down."}

Usage examples
--------------
# Move to a position and face a direction
ros2 topic pub --once /w1/actor_command std_msgs/msg/String \
  'data: "{\"command\": \"goto\", \"x\": 3.0, \"y\": 2.0, \"final_yaw\": 1.57}"'

# Stop immediately
ros2 topic pub --once /w1/actor_command std_msgs/msg/String \
  'data: "{\"command\": \"stop\"}"'

# Run the built-in test trajectory
ros2 topic pub --once /w1/actor_command std_msgs/msg/String \
  'data: "{\"command\": \"test\"}"'

# Finish current action then shut down
ros2 topic pub --once /w1/actor_command std_msgs/msg/String \
  'data: "{\"command\": \"finish\"}"'

# Monitor results
ros2 topic echo /w1/actor_result
"""

import json
import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ActorController(Node):
    def __init__(self):
        super().__init__('actor_controller')

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('actor_name',  'human_1')
        self.declare_parameter('world_name',  'backyard')
        self.declare_parameter('initial_x',   0.0)
        self.declare_parameter('initial_y',   0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.actor_name  = self.get_parameter('actor_name').get_parameter_value().string_value
        self.world_name  = self.get_parameter('world_name').get_parameter_value().string_value
        self.current_x   = self.get_parameter('initial_x').get_parameter_value().double_value
        self.current_y   = self.get_parameter('initial_y').get_parameter_value().double_value
        self.current_yaw = self.get_parameter('initial_yaw').get_parameter_value().double_value
        self.z           = 0.0

        # Model faces -Y when SDF yaw=0 → add π/2 when calling set_pose
        self.MODEL_FORWARD_OFFSET = math.pi / 2

        self.get_logger().info(
            f'ActorController ready — actor={self.actor_name}, world={self.world_name}, '
            f'pose=({self.current_x:.2f}, {self.current_y:.2f}, '
            f'yaw={math.degrees(self.current_yaw):.1f}°)'
        )

        # ------------------------------------------------------------------ #
        # State
        # ------------------------------------------------------------------ #
        self._lock           = threading.Lock()
        self._stop_requested = False
        self._finish_after   = False
        self._busy           = False

        # ------------------------------------------------------------------ #
        # Topics
        # ------------------------------------------------------------------ #
        self._result_pub = self.create_publisher(String, 'actor_result', 10)

        self._cmd_sub = self.create_subscription(
            String,
            'actor_command',
            self._on_command,
            10,
        )
        self.get_logger().info(
            "Subscribed to '~/actor_command'. Publishing results on '~/actor_result'."
        )

    # ===================================================================== #
    # Command subscriber callback
    # ===================================================================== #
    def _on_command(self, msg: String):
        raw = msg.data.strip()
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().error(f"Could not parse command JSON: '{raw}'")
            self._publish_result(False, 'parse_error', f"Invalid JSON: '{raw}'")
            return

        cmd = payload.get('command', '').strip().lower()
        self.get_logger().info(f"Received command: '{cmd}'  payload={payload}")

        # ---- Non-blocking commands ---- #
        if cmd == 'stop':
            with self._lock:
                self._stop_requested = True
            self.get_logger().info('Stop requested.')
            self._publish_result(True, 'stop', 'Actor stopped.')
            return

        if cmd == 'finish':
            with self._lock:
                self._finish_after   = True
                self._stop_requested = True
            self.get_logger().info('Finish requested.')
            self._publish_result(True, 'finish',
                                 'Finish acknowledged — will shut down when idle.')
            return

        # ---- Blocking commands run in a background thread ---- #
        if cmd == 'goto':
            x         = float(payload.get('x', 0.0))
            y         = float(payload.get('y', 0.0))
            final_yaw = float(payload.get('final_yaw', 0.0))
            self._dispatch(self._action_goto, cmd, x=x, y=y, final_yaw=final_yaw)
            return

        if cmd == 'test':
            self._dispatch(self._action_test, cmd)
            return

        self.get_logger().warn(f"Unknown command: '{cmd}'")
        self._publish_result(False, cmd,
                             f"Unknown command '{cmd}'. "
                             "Valid: goto | stop | test | finish")

    # ===================================================================== #
    # Thread dispatcher
    # ===================================================================== #
    def _dispatch(self, action_fn, cmd, **kwargs):
        with self._lock:
            if self._busy:
                self.get_logger().warn(
                    f"Ignoring '{cmd}' — actor is already executing a command."
                )
                self._publish_result(False, cmd,
                                     'Actor is already executing a command.')
                return
            self._busy           = True
            self._stop_requested = False

        def run():
            result = {'success': True, 'message': 'Done.'}
            try:
                action_fn(result, **kwargs)
            except Exception as exc:  # noqa: BLE001
                result['success'] = False
                result['message'] = f'Exception: {exc}'
            finally:
                with self._lock:
                    self._busy = False

            self._publish_result(result['success'], cmd, result['message'])

            # Check finish flag now that we're idle
            with self._lock:
                should_finish = self._finish_after
            if should_finish:
                self.get_logger().info('Finish flag set — shutting down.')
                threading.Thread(target=self._shutdown, daemon=True).start()

        threading.Thread(target=run, daemon=True).start()

    def _shutdown(self):
        time.sleep(0.2)
        rclpy.shutdown()

    # ===================================================================== #
    # Actions
    # ===================================================================== #
    def _action_goto(self, result, x, y, final_yaw):
        self.get_logger().info(
            f'goto → ({x:.2f}, {y:.2f}), final_yaw={math.degrees(final_yaw):.1f}°'
        )

        self.move_to(x, y, speed=2.0, turn_speed=2.0)
        if self._is_stopped():
            result['success'] = False
            result['message'] = 'goto interrupted by stop.'
            return

        self._rotate_to(final_yaw, turn_speed=2.0, step_delay=0.05)
        if self._is_stopped():
            result['success'] = False
            result['message'] = 'goto (final rotation) interrupted by stop.'
            return

        result['success'] = True
        result['message'] = (
            f'Reached ({x:.2f}, {y:.2f}), facing {math.degrees(final_yaw):.1f}°.'
        )

    def _action_test(self, result):
        self.get_logger().info('test → running square trajectory')
        time.sleep(3.0)

        waypoints = [(2.0, 0.0), (2.0, 3.0), (0.0, 3.0), (0.0, 0.0)]
        for wx, wy in waypoints:
            if self._is_stopped():
                result['success'] = False
                result['message'] = 'test trajectory interrupted by stop.'
                return
            self.move_to(wx, wy, speed=2.0, turn_speed=2.0)
            self.get_logger().info(f'Reached ({wx}, {wy}), pausing…')
            time.sleep(2.0)

        result['success'] = True
        result['message'] = 'Test trajectory completed one full lap.'

    # ===================================================================== #
    # Motion primitives
    # ===================================================================== #
    def move_to(self, target_x, target_y, speed=2.0, step_delay=0.05, turn_speed=2.0):
        dx       = target_x - self.current_x
        dy       = target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        if distance < 0.01:
            return

        target_yaw = math.atan2(dy, dx)
        self.get_logger().info(
            f'move_to ({target_x:.2f}, {target_y:.2f})  '
            f'dist={distance:.2f}m  heading={math.degrees(target_yaw):.1f}°'
        )

        self._rotate_to(target_yaw, turn_speed=turn_speed, step_delay=step_delay)
        if self._is_stopped():
            return

        step_dist = speed * step_delay
        num_steps = max(1, int(distance / step_dist))
        for i in range(num_steps):
            if self._is_stopped():
                return
            t = (i + 1) / num_steps
            self.set_pose(self.current_x + dx * t,
                          self.current_y + dy * t,
                          self.current_yaw)
            time.sleep(step_delay)

        self.set_pose(target_x, target_y, self.current_yaw)
        self.current_x = target_x
        self.current_y = target_y

    def _rotate_to(self, target_yaw, turn_speed=2.0, step_delay=0.05):
        yaw_diff   = self.angle_diff(target_yaw, self.current_yaw)
        turn_steps = max(1, int(abs(yaw_diff) / (turn_speed * step_delay)))
        for i in range(turn_steps):
            if self._is_stopped():
                return
            t   = (i + 1) / turn_steps
            yaw = self.current_yaw + yaw_diff * t
            self.set_pose(self.current_x, self.current_y, yaw)
            time.sleep(step_delay)
        self.current_yaw = target_yaw
        self.set_pose(self.current_x, self.current_y, self.current_yaw)

    # ===================================================================== #
    # Gazebo interface
    # ===================================================================== #
    def set_pose(self, x, y, yaw):
        sdf_yaw         = yaw + self.MODEL_FORWARD_OFFSET
        qx, qy, qz, qw = self.yaw_to_quaternion(sdf_yaw)
        req = (
            f'name: "{self.actor_name}", '
            f'position: {{x: {float(x)}, y: {float(y)}, z: {self.z}}}, '
            f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        )
        subprocess.run([
            'ign', 'service',
            '-s',        f'/world/{self.world_name}/set_pose',
            '--reqtype', 'ignition.msgs.Pose',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '2000',
            '--req',     req,
        ], check=False)

    # ===================================================================== #
    # Helpers
    # ===================================================================== #
    def yaw_to_quaternion(self, yaw):
        return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def angle_diff(self, target, current):
        return (target - current + math.pi) % (2 * math.pi) - math.pi

    def _is_stopped(self):
        with self._lock:
            return self._stop_requested

    def _publish_result(self, success: bool, command: str, message: str):
        payload = json.dumps({'success': success, 'command': command, 'message': message})
        self.get_logger().info(f'Result: {payload}')
        self._result_pub.publish(String(data=payload))


def main():
    rclpy.init()
    node = ActorController()
    rclpy.spin(node)
    node.destroy_node()
