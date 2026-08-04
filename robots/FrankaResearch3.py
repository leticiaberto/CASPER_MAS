"""
===============
High-level agent for a Franka Research 3 arm.

Wraps FrankaAdapter the same way Human wraps HumanAdapter and
Tiago wraps TiagoAdapter:
  - extends Agent base class (skill_weights, contexts, role, teamsize)
  - owns the ROS2 executor and spin thread
  - exposes blocking and non-blocking pick_and_place / cancel
  - has shutdown() for clean exit

Spawn in Gazebo is handled by Robot.py (same as every other model).

Usage (from Robot.py)
---------------------
    agent = FrankaResearch3(
        robot_name    = robot_id,
        constraints    = constraints,
        skill_weights = skill_weights,
        contexts      = contexts,
        role          = agent_role,
        teamsize      = teamsize,
        use_sim       = USE_SIM,
        party_duration = party_duration,
    )

    ok, msg = agent.pick_and_place(
        pick_xyz  = (0.5, 0.0, 0.3),
        place_xyz = (0.5, 0.4, 0.3),
    )
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor

from src.entities.Agent import Agent
from fr3_adapters.FrankaAdapter import FrankaAdapter, RobotMode
from robot_common.object_world_to_robot import ObjectToRobot
from robot_common.sdf_surface_resolver import SdfSurfaceResolver
from utils import ROSUtils
from locations import get_scene


ResultCallback = Callable[[bool, str], None]
XYZ = Tuple[float, float, float]

CONST_SCALE = 10

TIME_PICK_THE_DISHES = 600/CONST_SCALE  
TIME_DOING_THE_DISHES = 600/CONST_SCALE    # seconds to "do the dishes" (simulate with sleep)
TIME_PUTTING_AWAY_DISHES = 300/CONST_SCALE    # seconds to "put away dishes" (simulate with sleep)

TIME_PICKING_RICE = 330/CONST_SCALE     # seconds to "pick" the rice (simulate with sleep)
TIME_COOKING_RICE = 2400/CONST_SCALE    # seconds to "cook" the rice (simulate with sleep)
TIME_SERVE_MAIN_DISH = 120/CONST_SCALE

TIME_WELCOMING_GUESTS = 5#15.0  # seconds to "welcome guests" (simulate with sleep)

TIME_PICK_FOOD_GRILL = 300/CONST_SCALE
TIME_GRILL_FOOD = 5000/CONST_SCALE    # seconds
TIME_PICK_GRILLED_FOOD = 300/CONST_SCALE
TIME_SERVE_GRILLED_FOOD = 120/CONST_SCALE

TIME_PICK_VEGETABLE = 300/CONST_SCALE
TIME_CHOP_VEGETABLES = 3000/CONST_SCALE    # seconds
TIME_PICK_CHOPPED_VEGETABLE = 300/CONST_SCALE
TIME_SERVE_SIDES = 120/CONST_SCALE

TIME_PICK_SALAD = 300/CONST_SCALE
TIME_CHOP_SALAD_INGREDIENTS = 2000/CONST_SCALE    # seconds
TIME_PICK_CHOPPED_SALAD = 300/CONST_SCALE
TIME_SERVE_SALAD = 120/CONST_SCALE

#TIME_SERVING_DRINKS = 0

#TIME_HOST = 0

TIME_SAYING_GOODBYE = 15.0

# ---------------------------------------------------------------------------
# Scenes
# ---------------------------------------------------------------------------
# DEFAULT_SCENES now lives in locations.py (shared with Human/Tiago's
# navigation waypoints, same world_name keys) and is looked up via
# get_scene(world_name), which falls back to the "backyard" layout if
# world_name is unset or unrecognized.


class FrankaResearch3(Agent):
    """
    Parameters
    ----------
    robot_name : str
        ROS2 namespace of the robot (e.g. "fr3_robot1").
    constraints : dict
    skill_weights : dict
    contexts : list
    role : str
    teamsize : int
    use_sim : bool
        True  → RobotMode.SIMULATION (Gazebo + FollowJointTrajectory)
        False → RobotMode.PHYSICAL   (real FR3 via Franky)
    franky_ip : str, optional
        IP of the real FR3. Required when use_sim=False.
    franky_gripper_speed : float
    franky_gripper_force : float
    base_height : float
        Height of robot base above world origin (metres).
        Set to 1.03 if spawned at z=1.03 in Gazebo.
    result_timeout : float
        Seconds to wait for a result before timeout (default 600).
    node_name : str, optional
        Override the internal ROS2 node name.
    """

    def __init__(
        self,
        robot_name:           str,
        constraints:          dict          = None,
        skill_weights:        dict          = None,
        contexts:             List[str]     = None,
        role:                 str           = "member",
        teamsize:             int           = 1,
        run_id:               str           = "test",
        use_sim:              bool          = True,
        workspace:            List[str]     = None,
        party_duration:       float         = 3600.0, #1h default
        guests:               int           = 0,
        franky_ip:            Optional[str] = None,
        franky_gripper_speed: float         = 0.05,
        franky_gripper_force: float         = 10.0,
        base_height:          float         = 0.0,
        result_timeout:       float         = 600.0,
        node_name:            Optional[str] = None,
        world_name:           Optional[str] = None,
        pose_timeout:         float         = 15.0,
    ) -> None:

        super().__init__(
            robot_name,
            constraints or {},
            workspace or [],
            skill_weights or {},
            contexts or list((skill_weights or {}).keys()),
            role,
            teamsize,
            party_duration,
            guests,
            run_id,
        )

        self._robot_name     = robot_name
        self._world_name     = world_name
        self._result_timeout = result_timeout

        # Completion signal shared between adapter callback and blocking callers
        self._goal_done_event               = threading.Event()
        self._last_result: Tuple[bool, str] = (False, "No goal sent yet.")

        # ── ROS2 ─────────────────────────────────────────────────────────────
        if not rclpy.ok():
            rclpy.init()

        mode = RobotMode.SIMULATION if use_sim else RobotMode.PHYSICAL

        self.adapter = FrankaAdapter(
            robot_name           = robot_name,
            mode                 = mode,
            result_callback      = self._on_result,
            franky_ip            = franky_ip,
            franky_gripper_speed = franky_gripper_speed,
            franky_gripper_force = franky_gripper_force,
            base_height          = base_height,
            node_name            = node_name,
        )

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.adapter)

        if mode == RobotMode.SIMULATION and self.adapter._gripper is not None:
            self._executor.add_node(self.adapter._gripper)

        self._spin_thread = threading.Thread(
            target = self._executor.spin,
            daemon = True,
            name   = f"franka_{robot_name}_spin",
        )
        self._spin_thread.start()
        time.sleep(0.5)

        # ── Name → XYZ resolution (mirrors Tiago pattern) ────────────────────
        # Auto-resolve sdf_path and models_base_dir via ROSUtils when not given.
        if world_name is not None:
            try:
                sdf_path = ROSUtils._get_sdf_path(world_name)
                print(f"[FrankaResearch3] Auto-resolved SDF: {sdf_path}")
            except FileNotFoundError as exc:
                print(f"[FrankaResearch3] WARNING: SDF not found ({exc}).")
            try:
                models_base_dir = ROSUtils._get_models_dir()
                print(f"[FrankaResearch3] Auto-resolved models dir: {models_base_dir}")
            except FileNotFoundError as exc:
                print(f"[FrankaResearch3] WARNING: models dir not found ({exc}).")

        # ObjectToRobot shares the adapter node — same pattern as Tiago.
        self._otr: Optional[ObjectToRobot] = None
        if world_name is not None:
            self._otr = ObjectToRobot(
                node         = self.adapter,
                robot_name   = robot_name,
                world_name   = world_name,
                pose_timeout = pose_timeout,
            )

        # SdfSurfaceResolver: top-surface z offsets from SDF geometry.
        self._surface: Optional[SdfSurfaceResolver] = None
        if sdf_path:
            self._surface = SdfSurfaceResolver(sdf_path, verbose=False)

        self.adapter.get_logger().info(
            f"[FrankaResearch3] Agent '{robot_name}' ready ({mode.value.upper()}). "
            f"Name resolution: {'enabled' if self._otr else 'disabled (no world_name)'}."
        )
        # FR3 can get it automatically, but it gets the center of the objects.
        # Here I define some specific places just to look better in the video.
        # Selected from DEFAULT_SCENES (locations.py) by world_name — falls
        # back to the "backyard" layout if world_name is unset or unrecognized.
        self.scene = get_scene(world_name)
    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def pick_and_place(
        self,
        pick_xyz:   Optional[XYZ]            = None,
        place_xyz:  Optional[XYZ]            = None,
        pick_name:  Optional[str]            = None,
        place_name: Optional[str]            = None,
        callback:   Optional[ResultCallback] = None,
    ) -> Tuple[bool, str]:
        """
        Execute a pick-and-place motion.

        Accepts either object names or direct XYZ coordinates — or a mix.

        Parameters
        ----------
        pick_xyz, place_xyz : (x, y, z), optional
            Direct world-frame coordinates.
        pick_name, place_name : str, optional
            Gazebo model names resolved via ObjectToRobot + SdfSurfaceResolver.
            Requires world_name at init and the gz-ros2 bridge to be running.
            pick_name  → object centre Z   (gripper grasps mid-height).
            place_name → top-surface Z     (object rests on the surface).
        callback : callable, optional
            Non-blocking if provided; blocking otherwise.
        """
        # ── Resolve pick XYZ ─────────────────────────────────────────
        if pick_xyz is None:
            if pick_name is None:
                return False, "pick_xyz or pick_name required."
            if self._otr is None:
                return False, (
                    "pick_name given but world_name was not set at init — "
                    "cannot resolve object pose."
                )
            result = self._otr.get_pose(pick_name)
            if "error" in result:
                return False, f"Cannot resolve pick '{pick_name}': {result['error']}"
            # FR3 planner works in robot base frame — use robot_pose, not world_pose.
            pos = result["robot_pose"].position
            pick_xyz = (pos.x, pos.y, pos.z)
            print(f"[FrankaResearch3] '{pick_name}' pick XYZ (robot frame): "
                  f"({pick_xyz[0]:.3f}, {pick_xyz[1]:.3f}, {pick_xyz[2]:.3f})")
        else:
            # Raw pick_xyz is in world frame — convert to robot frame.
            pick_xyz = self._otr.world_xyz_to_robot(pick_xyz)


        # ── Resolve place XYZ ────────────────────────────────────────
        if place_xyz is None:
            if place_name is None:
                return False, "place_xyz or place_name required."
            if self._otr is None:
                return False, (
                    "place_name given but world_name was not set at init — "
                    "cannot resolve object pose."
                )
            result = self._otr.get_pose(place_name)
            if "error" in result:
                return False, f"Cannot resolve place '{place_name}': {result['error']}"
            # Use robot_pose + surface offset (robot base frame).
            pos = result["robot_pose"].position
            surface_offset = (
                self._surface.top_surface_offset(place_name)
                if self._surface is not None else 0.0
            )
            place_xyz = (pos.x, pos.y, pos.z + surface_offset)
            print(f"[FrankaResearch3] '{place_name}' place XYZ (robot frame): "
                  f"({place_xyz[0]:.3f}, {place_xyz[1]:.3f}, {place_xyz[2]:.3f}) "
                  f"(surface_offset={surface_offset:.3f})")
        else:
            # Raw place_xyz is in world frame — convert to robot frame.
            place_xyz = self._otr.world_xyz_to_robot(place_xyz)


        # ── Dispatch ─────────────────────────────────────────────────
        if callback is not None:
            self._install_callback(callback)
            accepted = self.adapter.pick_and_place(
                pick_xyz=pick_xyz, place_xyz=place_xyz)
            if not accepted:
                callback(False, "Adapter busy — pick_and_place rejected.")
            return True, "accepted"

        self._goal_done_event.clear()
        accepted = self.adapter.pick_and_place(
            pick_xyz=pick_xyz, place_xyz=place_xyz)
        if not accepted:
            msg = "[FrankaResearch3] Adapter busy — pick_and_place rejected."
            self.adapter.get_logger().warn(msg)
            return False, msg

        finished = self._goal_done_event.wait(timeout=self._result_timeout)
        if not finished:
            msg = f"[FrankaResearch3] Timeout ({self._result_timeout}s)."
            self.adapter.get_logger().error(msg)
            return False, msg

        return self._last_result

    def cancel(self) -> None:
        """Cancel any in-progress motion."""
        self.adapter.cancel()

    @property
    def is_busy(self) -> bool:
        return self.adapter.is_busy

    def create_tasks_sequence(self, task_type: str, group: str, target_place: str, use_names: bool = False) -> List[dict]:
        sequence = []
        if task_type == "pick_place":
            for food, placements in self.scene[group]["placements"].items():
                if use_names:
                    task = {
                        "action":      "pick_and_place",
                        "pick_name":   food,
                        "place_name":  target_place,
                    }
                else:
                    task = {
                        "action":     "pick_and_place",
                        "pick_name":  food,
                        "place_xyz":  placements[target_place],
                    }
                sequence.append(task)
        else:
            print(f"Task type '{task_type}' not recognized.")
        return sequence

    def define_subtask(self, action: str, use_names: bool = False) -> List[dict]:
        sequence = None

        # Vegetables
        if action == "PickVegetables":
            sequence = self.create_tasks_sequence(
                "pick_place", "vegetables_side", "place_chop" if not use_names else "chop_board_1",
                use_names=use_names,
            )
            print("[FR3]: Picking vegetables...")
        elif action == "ChopVegetables":
            sequence = [{"action": "sleep", "time": TIME_CHOP_VEGETABLES}]
            print("[FR3]: Chopping vegetables...")
        elif action == "PickChoppedVegetables":
            sequence = self.create_tasks_sequence(
                "pick_place", "vegetables_side", "place_bowl" if not use_names else "bowl_2",
                use_names=use_names,
            )
            print("[FR3]: Picking chopped vegetables...")

        # Salad
        elif action == "PickSaladIngredients":
            sequence = self.create_tasks_sequence(
                "pick_place", "salads_side", "place_chop" if not use_names else "chop_board_1",
                use_names=use_names,
            )
            print("[FR3]: Picking salad ingredients...")
        elif action == "ChopSaladIngredients":
            sequence = [{"action": "sleep", "time": TIME_CHOP_SALAD_INGREDIENTS}]  # Simulate chopping time
            print("[FR3]: Chopping salad ingredients...")
        elif action == "PickChoppedSaladIngredients":
            sequence = self.create_tasks_sequence(
                "pick_place", "salads_side", "place_bowl" if not use_names else "bowl_1",
                use_names=use_names,
            )
            print("[FR3]: Picking chopped salad ingredients...")

        # Meat + Garlic Bread
        elif action == "PickFoodIngredientsGrill":
            sequence = self.create_tasks_sequence(
                "pick_place", "food_grill", "place_grill" if not use_names else "grill",
                use_names=use_names,
            )
            print("[FR3]: Picking food ingredients for the grill...")
        elif action == "GrillFood":
            sequence = [{"action": "sleep", "time": TIME_GRILL_FOOD}]  # Simulate grilling time
            print("[FR3]: Grilling food...")
        elif action == "PickGrilledFood":
            sequence = self.create_tasks_sequence(
                "pick_place", "food_grill", "place_plate" if not use_names else "plate_1",
                use_names=use_names,
            )
            print("[FR3]: Picking grilled food...")

        # Fish + Squid
        elif action == "PickSeafoodIngredientsGrill":
            sequence = self.create_tasks_sequence(
                "pick_place", "seafood_grill", "place_grill" if not use_names else "fish_grill",
                use_names=use_names,
            )
            print("[FR3]: Picking seafood ingredients for the grill...")
        elif action == "GrillSeafood":
            sequence = [{"action": "sleep", "time": TIME_GRILL_FOOD}]  # Simulate grilling time
            print("[FR3]: Grilling seafood...")
        elif action == "PickGrilledSeafood":
            sequence = self.create_tasks_sequence(
                "pick_place", "seafood_grill", "place_plate" if not use_names else "plate_4",
                use_names=use_names,
            )
            print("[FR3]: Picking grilled seafood...")
            
        elif action == "Reception":
            print("[FR3]: receiving guests...")
            sequence = [{"action": "sleep", "time": 5}]
        elif action == "PrepareDrinks":
            print("[FR3]: Preparing drinks...")
            sequence = [{"action": "sleep", "time": 5}]
        elif action == "SuperviseParty":
            print("[FR3]: Supervising party...")
            sequence = [{"action": "sleep", "time": 5}]
        elif action == "Clean":
            print("[FR3]: Cleaning...")
            sequence = [{"action": "sleep", "time": 5}]
        elif action == "PrepareFood":
            print("[FR3]: Preparing food...")
            sequence = [{"action": "sleep", "time": 5}]

        return sequence
    
    # ------------------------------------------------------------------
    # Agent interface (task-graph dispatch)
    # ------------------------------------------------------------------
    def _execute_task_specific(self, task: dict) -> None:
        """
        Called by the Agent base class task dispatcher.
        """
        sequence = self.define_subtask(task)
        if sequence is None:
            print(f"[FrankaResearch3] No sequence defined for action: '{task}'")
            return
    
        for subtask in sequence:
            action = subtask["action"]
            if action == "pick_and_place":
                ok, msg = self.pick_and_place(
                    pick_xyz   = subtask["pick_xyz"] if "pick_xyz" in subtask else None,
                    place_xyz  = subtask["place_xyz"] if "place_xyz" in subtask else None,
                    pick_name  = subtask["pick_name"] if "pick_name" in subtask else None,
                    place_name = subtask["place_name"] if "place_name" in subtask else None,
                )
                log = self.adapter.get_logger() if self.adapter else None
                if log:
                    try:
                        if ok:
                            log.info(f"[FrankaResearch3] task result: {msg}")
                        else:
                            log.error(f"[FrankaResearch3] task result: {msg}")
                    except Exception as log_exc:
                        # Never let a logging issue take down the whole agent —
                        # the goal itself already finished (ok/msg), so just
                        # fall back to print and keep going.
                        print(
                            f"[FrankaResearch3] (logger error: {log_exc}) "
                            f"task result: {msg}"
                        )
            elif action == "sleep":
                time.sleep(subtask.get("time"))
            elif action in ("grasp", "move_arm", "inspect"):
                print(f"[FrankaResearch3] '{action}' not yet implemented.")
            else:
                print(f"[FrankaResearch3] Unknown task action: '{action}'")

    def shutdown(self) -> None:
        """Cleanly stop executor, destroy nodes, shut down ROS2."""
        print(f"[FrankaResearch3] '{self._robot_name}' shutting down ...")

        self._executor.shutdown(timeout_sec=3.0)
        try:
            if self.adapter._gripper is not None:
                self.adapter._gripper.destroy_node()
        except Exception:
            pass
        try:
            self.adapter.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        print(f"[FrankaResearch3] '{self._robot_name}' shutdown complete.")

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _install_callback(self, callback: ResultCallback) -> None:
        """Temporarily swap in a one-shot callback."""
        original = self.adapter._result_callback

        def one_shot(success: bool, message: str) -> None:
            self.adapter._result_callback = original
            self._on_result(success, message)
            callback(success, message)

        self.adapter._result_callback = one_shot

    def _on_result(self, success: bool, message: str) -> None:
        """Adapter callback — stores result and wakes any blocking caller."""
        self._last_result = (success, message)
        self._goal_done_event.set()