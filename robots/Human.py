"""
Human
=====
High-level agent for a Gazebo / Ignition human actor driven by
actor_controller.py (std_msgs/String topic-based command protocol).

Wraps HumanAdapter the same way Tiago wraps TiagoAdapter:
  - owns the ROS2 executor and spin thread
  - exposes clean blocking and non-blocking public methods
  - fires result_callback(success, message) on completion

Architecture
------------
                ┌─────────────────────────────┐
                │  Human                       │
                │                             │
                │  HumanAdapter  (ROS2 node)  │
                │    ├─ pub /<name>/actor_command  (std_msgs/String JSON)
                │    └─ sub /<name>/actor_result   (std_msgs/String JSON)
                └─────────────────────────────┘

Two-point rendezvous with GuestManager
---------------------------------------
During WelcomeGuests the host blocks on one sync signal published by
GuestManager on /<actor_name>/guest_sync (String JSON):
 
  {"phase": "ready", "guest": N}   — guest spawned at door, adapter live
 
Topic map
---------
  pub /<name>/actor_command   → actor_controller
  sub /<name>/actor_result    ← actor_controller
  pub /<name>/actor_state     → GuestManager reads WelcomeGuests transitions
  sub /<name>/guest_sync      ← GuestManager signals the two sync points

Usage
-----
    human = Human(actor_name="host")

    # Blocking — returns (success, message) when the actor arrives
    ok, msg = human.goto(x=-0.19, y=-6.76, final_yaw=1.57)

    # Non-blocking — pass a callback, returns immediately
    human.goto(x=2.0, y=1.0, final_yaw=0.0,
               callback=lambda ok, msg: print(ok, msg))

    human.stop()
    human.test()
    human.finish()

    human.shutdown()
"""

from __future__ import annotations

import json
import threading
import time
from typing import Callable, Optional, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

from human_adapters.HumanAdapter import HumanAdapter
from src.entities.Agent import Agent

locations = {
    "DiningTable":    {"x": -0.77, "y": -1.54, "yaw": 3.14},
    "House":          {"x": -0.19, "y": -6.76, "yaw": 1.57},
    "GroupOfGuests_1":{"x":  3.25, "y": -2.82, "yaw": 0.0},
    "GroupOfGuests_2":{"x": -2.21, "y":  3.54,  "yaw": 2.15},
    "MainPrepTable":  {"x":  0.95, "y":  4.30, "yaw": 1.57},
    "GrillPrepTable": {"x":  4.14, "y":  4.24, "yaw": 0.8},
    "DrinksTable":    {"x": -4.50, "y":  4.89, "yaw": -1.55},
}

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
# Types
# ---------------------------------------------------------------------------

ResultCallback = Callable[[bool, str], None]


# ---------------------------------------------------------------------------
# Human
# ---------------------------------------------------------------------------

class Human(Agent):
    """
    High-level handle for a Gazebo human actor.

    Parameters
    ----------
    actor_name : str
        Namespace of the actor (must match the actor_controller launch arg).
        E.g. "host", "w1", "visitor1".
    result_timeout : float
        Seconds to wait for a result before declaring a timeout.
        Default: 120 s — covers slow or long-range moves.
    sync_timeout : float
        Seconds to wait for each GuestManager sync signal. Default 300 s.
    node_name : str, optional
        Override the internal ROS2 node name.
    """

    def __init__(
        self,
        actor_name:     str,
        skill_weights,
        contexts,
        role,
        teamsize:     int,
        run_id:               str           = "test",
        result_timeout: float         = 120.0,
        sync_timeout:    float         = 60,
        node_name:     Optional[str] = None,
        use_sim       = True,
        workspace:    str           = None,
        party_duration:       float         = 3600.0, #1h default
        guests:               int           = 0,
    ) -> None:
        constraints = {
            "can_move":               True,
            "can_manipulate":         True,
            "max_size_object_cm":     10,
            "max_payload_kg":         3,
            "max_reach_cm":           85,
            "can_transport_objects":  True,
            "workspace":              workspace,
        }

        super().__init__(actor_name, constraints, skill_weights, contexts, role, teamsize, party_duration, guests, run_id)

        self._actor_name = actor_name
        self._sync_timeout  = sync_timeout

        # ── ROS2 initialisation ───────────────────────────────────────
        if not rclpy.ok():
            rclpy.init()

        # Completion signal shared between the adapter callback and blocking callers
        self._goal_done_event           = threading.Event()
        self._last_result: Tuple[bool, str] = (False, "No command sent yet.")

        # ── HumanAdapter node ─────────────────────────────────────────
        self._adapter = HumanAdapter(
            actor_name      = actor_name,
            result_callback = self._on_result,
            result_timeout  = float(result_timeout),
            node_name       = node_name,
        )

        # ── State publisher  (/<actor_name>/actor_state) ──────────────
        # GuestManager listens here to know when WelcomeGuests fires.
        self._state_pub = self._adapter.create_publisher(
            String, f"/{actor_name}/actor_state", 10,
        )

        # ── Sync pub/sub on /<actor_name>/guest_sync ───────────────────
        # GuestManager → host : "ready"   (guest spawned, adapter live)
        # Host → GuestManager : "walk"    (host at group, guest may now walk)
        self._sync_pub = self._adapter.create_publisher(
            String, f"/{actor_name}/guest_sync", 10,
        )
        self._sync_event = threading.Event()
        self._sync_phase: Optional[str] = None
        self._sync_sub = self._adapter.create_subscription(
            String,
            f"/{actor_name}/guest_sync",
            self._on_guest_sync,
            10,
        )
 
        # ── Executor — SingleThreaded prevents duplicate message dispatch ─
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._adapter)
 
        self._spin_thread = threading.Thread(
            target = self._executor.spin,
            daemon = True,
            name   = f"human_{actor_name}_spin",
        )
        self._spin_thread.start()
 
        time.sleep(0.5)
        self._adapter.get_logger().info(f"[Human] Ready — actor='{actor_name}'.")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def get_destionation_coordinates(self, location_name: str) -> Tuple[float, float, float]:
        """
        Get the coordinates of a named location.

        Parameters
        ----------
        location_name : str
            Name of the location, e.g. "DiningTable", "House", etc.

        Returns
        -------
        (x, y, yaw) : Tuple[float, float, float]
            Coordinates of the location in the world frame.
            Raises KeyError if the location name is not found.
        """
        if location_name not in locations:
            raise KeyError(f"Location '{location_name}' not found.")
        loc = locations[location_name]
        return loc["x"], loc["y"], loc["yaw"]

    def goto(
        self,
        location   = None,
        x               = None,
        y               = None,
        final_yaw       = None,
        callback:  Optional[ResultCallback] = None,
    ) -> Tuple[bool, str]:
        """
        Move the actor to (x, y) then rotate to face final_yaw.

        Parameters
        ----------
        x, y : float
            World-frame target position (metres).
        final_yaw : float
            Heading to face after arriving (radians, world frame).
            Default: 0.0 (facing +X).
        callback : callable, optional
            If provided the call returns immediately and callback(success, message)
            is fired when the actor arrives.
            If omitted the call blocks until the actor arrives (or times out).

        Returns
        -------
        (success, message) when blocking; (True, "accepted") when non-blocking.
        """
         # Option 1: location key
        if location is not None:
            x, y, final_yaw = self.get_destionation_coordinates(location)
        
        # Option 2: direct coordinates
        else:
            if x is None or y is None or final_yaw is None:
                raise ValueError(
                    "You must provide either a valid location "
                    "or x, y, yaw coordinates."
                )

        return self._send("goto", callback, x=x, y=y, final_yaw=final_yaw)

    def stop(
        self,
        callback: Optional[ResultCallback] = None,
    ) -> Tuple[bool, str]:
        """
        Interrupt any in-progress movement immediately.

        The actor stays at its current position.
        Blocking by default; pass a callback for fire-and-forget.
        """
        return self._send("stop", callback)

    def test(
        self,
        callback: Optional[ResultCallback] = None,
    ) -> Tuple[bool, str]:
        """
        Run the built-in square-loop trajectory (one full lap).

        Blocking by default; pass a callback for fire-and-forget.
        """
        return self._send("test", callback)

    def finish(
        self,
        callback: Optional[ResultCallback] = None,
    ) -> Tuple[bool, str]:
        """
        Ask the actor to complete its current action then shut down its
        controller node.

        Blocking by default; pass a callback for fire-and-forget.
        """
        return self._send("finish", callback)

    @property
    def is_busy(self) -> bool:
        """True while a command is in flight."""
        return self._adapter.is_busy

    # ------------------------------------------------------------------
    # State / sync helpers
    # ------------------------------------------------------------------
    def _publish_state(self, action: str, phase: str, **extra) -> None:
        payload = {"action": action, "phase": phase, **extra}
        self._state_pub.publish(String(data=json.dumps(payload)))
 
    def _publish_sync(self, phase: str, guest_number: int) -> None:
        """Publish a sync signal to GuestManager (e.g. 'walk')."""
        payload = {"phase": phase, "guest": guest_number}
        self._sync_pub.publish(String(data=json.dumps(payload)))
        self._adapter.get_logger().info(
            f"[Human] sync → phase='{phase}' guest={guest_number}"
        )
 
    def _wait_for_guest_sync(self, phase: str) -> bool:
        """Block until GuestManager publishes the given phase on guest_sync."""
        self._sync_phase = phase
        self._sync_event.clear()
        self._adapter.get_logger().info(
            f"[Human] Waiting for guest_sync phase='{phase}' …"
        )
        arrived = self._sync_event.wait(timeout=self._sync_timeout)
        self._sync_phase = None
        if not arrived:
            self._adapter.get_logger().error(
                f"[Human] Timed out waiting for guest_sync phase='{phase}'."
            )
        return arrived
 
    def _on_guest_sync(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        # Only wake if this matches the phase we're currently waiting for
        if data.get("phase") == self._sync_phase:
            self._adapter.get_logger().info(f"[Human] guest_sync received: {data}")
            self._sync_event.set()

    # ------------------------------------------------------------------
    # Agent interface (task-graph dispatch)
    # ------------------------------------------------------------------

    def _execute_task_specific(self, task: dict) -> None:
        print(f"[Human] Executing task: {task}")
        action = task.get("action") if isinstance(task, dict) else task

        if action == "PickRice":
            print("[Human] Pick rice.")
            ok, msg = self.goto(location="House")
            time.sleep(TIME_PICKING_RICE)  # Simulate picking time
        elif action == "CookRice":
            print("[Human] Cook rice.")
            ok, msg = self.goto(location="House")
            time.sleep(TIME_COOKING_RICE)  # Simulate cooking time
        elif action == "ServeMainDish":
            print("[Human] Serve main dish.")
            ok, msg = self.goto(location="House")
            time.sleep(5)
            ok, msg = self.goto(location="DiningTable")
            time.sleep(10)
        elif action == "ServeSides":
            print("[Human] Serve sides.")
            ok, msg = self.goto(location="MainPrepTable")
            time.sleep(5) # simulate picking sides
            ok, msg = self.goto(location="DiningTable")
        elif action == "ServeSalad":
            print("[Human] Serve salad.")
            ok, msg = self.goto(location="MainPrepTable")
            time.sleep(5) # simulate picking salad
            ok, msg = self.goto(location="DiningTable")
        elif action == "ServeGrilledFood":
            print("[Human] Serve grilled food.")
            ok, msg = self.goto(location="GrillPrepTable")
            time.sleep(5) # simulate picking food
            ok, msg = self.goto(location="DiningTable")
        elif action == "WelcomeGuests":
            print("[Human] Welcome guests.")
            self._action_welcome_guests()
            time.sleep(50)  # Brief pause before going to check on guests
            ok, msg = self.goto(location="GroupOfGuests_1")
            time.sleep(15)  # Brief pause before moving to next group
            ok, msg = self.goto(location="GroupOfGuests_2")
            time.sleep(15)  # Brief pause before moving to next group
            ok, msg = self.goto(location="DiningTable")
            time.sleep(15)  # Brief pause before moving to next task
        elif action == "PickDishes":
            print("[Human] Pick dishes.")
            ok, msg = self.goto(location="House")
            time.sleep(TIME_PICK_THE_DISHES)  # Simulate putting away dishes time
        elif action == "DoTheDishes":
            print("[Human] Do the dishes.")
            ok, msg = self.goto(location="House")
            time.sleep(TIME_DOING_THE_DISHES)  # Simulate cleaning time
        elif action == "PutTheDishesAway":
            print("[Human] Put the dishes away.")
            ok, msg = self.goto(location="House")
            time.sleep(TIME_PUTTING_AWAY_DISHES)  # Simulate cleaning time
        elif action == "Host":
            print("[Human] Host the party.")
            time.sleep(20)  # Brief pause before going to check on guests
            ok, msg = self.goto(location="GrillPrepTable")
            time.sleep(20)
            ok, msg = self.goto(location="MainPrepTable")
            time.sleep(25)
            ok, msg = self.goto(location="GroupOfGuests_2")
            time.sleep(120)
            ok, msg = self.goto(location="DrinksTable")
            time.sleep(20)  # Brief pause before moving to next group
            ok, msg = self.goto(location="GroupOfGuests_1")
            time.sleep(120)  # Brief pause before moving to next group
            ok, msg = self.goto(location="DiningTable")
            time.sleep(60)
        elif action == "ServeDrinks":
            print("[Human] Serve drinks.")
            ok, msg = self.goto(location="DrinksTable")
            time.sleep(5) # simulate picking food
            ok, msg = self.goto(location="DiningTable")
            time.sleep(10)
            ok, msg = self.goto(location="DrinksTable")
            time.sleep(5) # simulate picking food
            ok, msg = self.goto(location="GroupOfGuests_1")
            time.sleep(10)
            ok, msg = self.goto(location="DrinksTable")
            time.sleep(5) # simulate picking food
            ok, msg = self.goto(location="GroupOfGuests_2")
            time.sleep(10)
        # Vegetables
        elif action == "PickVegetables":
            print("[Human] Pick vegetables.")
            ok, msg = self.goto(location="MainPrepTable")
            time.sleep(TIME_PICK_VEGETABLE)
        elif action == "ChopVegetables":
            print("[Human] Chop vegetables.")
            ok, msg = self.goto(location="MainPrepTable")
            print("Chopping vegetables... (not implemented)")
            time.sleep(TIME_CHOP_VEGETABLES)  # Simulate chopping time
        elif action == "PickChoppedVegetables":
            print("[Human] Pick chopped vegetables.")
            ok, msg = self.goto(location="MainPrepTable")
            time.sleep(TIME_PICK_CHOPPED_VEGETABLE)

        # Salad
        elif action == "PickSaladIngredients":
            print("[Human] Pick salad ingredients.")
            ok, msg = self.goto(location="MainPrepTable")
            time.sleep(TIME_PICK_SALAD)
        elif action == "ChopSaladIngredients":
            print("[Human] Chop salad ingredients.")
            ok, msg = self.goto(location="MainPrepTable")
            time.sleep(TIME_CHOP_SALAD_INGREDIENTS)  # Simulate chopping time
        elif action == "PickChoppedSaladIngredients":
            print("[Human] Pick chopped salad ingredients.")
            ok, msg = self.goto(location="MainPrepTable")
            time.sleep(TIME_PICK_CHOPPED_SALAD)

        # Meat + Garlic Bread
        elif action == "PickFoodIngredientsGrill":
            print("[Human] Pick food ingredients for grill.")
            ok, msg = self.goto(location="GrillPrepTable")
            time.sleep(TIME_PICK_FOOD_GRILL)
        elif action == "GrillFood":
            print("[Human] Grilling food.")
            ok, msg = self.goto(location="GrillPrepTable")
            print("Grilling food... (not implemented)")
            time.sleep(TIME_GRILL_FOOD)  # Simulate grilling time
        elif action == "PickGrilledFood":
            print("[Human] Pick grilled food.")
            ok, msg = self.goto(location="GrillPrepTable")
            time.sleep(TIME_PICK_GRILLED_FOOD)
        elif action == "Reception":
            print("[Human] Host the party.")
            time.sleep(5)
        elif action == "PrepareDrinks":
            print("[Human] Prepare drinks.")
            time.sleep(5)
        elif action == "SuperviseParty":
            print("[Human] Supervise party.")
            time.sleep(5)
        elif action == "Clean":
            print("[Human] Clean up after party.")
            time.sleep(5)
        elif action == "PrepareFood":
            print("[Human] Prepare food.")
            time.sleep(5)
        else:
            print(f"[Human] Unknown task action: {action}")

    def _action_welcome_guests(self) -> None:
        ok, msg = self.goto(location="House")

        self._publish_state("WelcomeGuests", "start", total_guests=self.guests)
 
        for guest_number in range(1, self.guests + 1):
            self._adapter.get_logger().info(
                f"[Human] WelcomeGuests — guest {guest_number}/{self.guests}"
            )
 
            # 1. Trigger GuestManager to spawn this guest
            self._publish_state(
                "WelcomeGuests", "loop",
                guest_index  = guest_number - 1,
                guest_number = guest_number,
                total_guests = self.guests,
            )
 
            # 2. Walk to door (while guest is being spawned in parallel)
            ok, msg = self.goto(location="House")
            if not ok:
                print(f"[Human] Could not reach House for guest {guest_number}: {msg}")
 
            # 3. Wait for GuestManager: guest is spawned and adapter is live
            if not self._wait_for_guest_sync("ready"):
                print(f"[Human] Sync 'ready' timed out for guest {guest_number}.")
                continue
 
            # 4. Greet guest at door
            self._adapter.get_logger().info(
                f"[Human] Greeting guest {guest_number} at the door …"
            )
            time.sleep(TIME_WELCOMING_GUESTS)
 
            # 5. Signal guest to walk to group — host stays at door
            self._publish_sync("walk", guest_number)
 
            # 7. Brief pause before next guest
            if guest_number < 3:  # First few guests are more likely to cause performance hitches in Gazebo, so wait a bit after each
                time.sleep(10)
            else:
                time.sleep(30)# Longer trajectory, so wait a bit more before next guest to have problems with gazebo dropping performance
 
        self._publish_state("WelcomeGuests", "end", total_guests=self.guests)
 
    def _on_party_ending(self) -> None:
        """
        Override of Agent._on_party_ending. Called once, on the supervisor,
        when party_duration has elapsed. Runs in its own thread so the
        caller (Agent.step()) is never blocked by it.
        """
        threading.Thread(
            target=self._action_farewell_guests, daemon=True, name="farewell_guests"
        ).start()

    def _action_farewell_guests(self) -> None:
        """
        Mirror of _action_welcome_guests: walk to the door, announce
        FarewellGuests on /<host>/actor_state so GuestManager can walk
        every already-spawned guest back home, then pause to "say goodbye".
        """
        ok, msg = self.goto(location="House")
        if not ok:
            print(f"[Human] Could not reach House to say goodbye: {msg}")

        self._publish_state("FarewellGuests", "start", total_guests=self.guests)
        self._adapter.get_logger().info("[Human] Saying goodbye to guests …")
        time.sleep(TIME_SAYING_GOODBYE)
        self._publish_state("FarewellGuests", "end", total_guests=self.guests)
        
    def shutdown(self) -> None:
        """Cleanly stop the executor and destroy the ROS2 node."""
        self._adapter.get_logger().info("[Human] Shutting down ...")
        self._executor.shutdown(timeout_sec=3.0)
        try:
            self._adapter.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        print(f"[Human] '{self._actor_name}' shutdown complete.")

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _send(
        self,
        command:  str,
        callback: Optional[ResultCallback],
        **kwargs,
    ) -> Tuple[bool, str]:
        """
        Dispatch a command through the adapter.

        Non-blocking path: wire a one-shot wrapper around the user callback
        and return immediately.

        Blocking path: clear the event, let the adapter's internal callback
        (_on_result) set it, then wait.
        """
        if callback is not None:
            # Non-blocking — install a temporary callback then return
            self._install_callback(command, callback, **kwargs)
            return True, "accepted"

        # Blocking
        self._goal_done_event.clear()
        accepted = self._dispatch(command, **kwargs)
        if not accepted:
            msg = f"[Human] Adapter busy — '{command}' rejected."
            self._adapter.get_logger().warn(msg)
            return False, msg

        self._goal_done_event.wait()
        return self._last_result

    def _dispatch(self, command: str, **kwargs) -> bool:
        """Call the matching HumanAdapter method and return accepted bool."""
        if command == "goto":
            return self._adapter.goto(**kwargs)
        if command == "stop":
            return self._adapter.stop()
        if command == "test":
            return self._adapter.test()
        if command == "finish":
            return self._adapter.finish()
        return False

    def _install_callback(
        self,
        command:  str,
        callback: ResultCallback,
        **kwargs,
    ) -> None:
        """
        Temporarily swap in a one-shot callback, dispatch, then restore.
        Runs in a thread so the caller is not blocked.
        """
        original_callback = self._adapter._result_callback

        def one_shot(success: bool, message: str) -> None:
            self._adapter._result_callback = original_callback
            self._on_result(success, message)   # keep internal state fresh
            callback(success, message)

        self._adapter._result_callback = one_shot
        accepted = self._dispatch(command, **kwargs)
        if not accepted:
            # Restore immediately if rejected
            self._adapter._result_callback = original_callback
            callback(False, "Adapter busy — command rejected.")

    def _on_result(self, success: bool, message: str) -> None:
        """Internal callback — stores result and wakes any blocking caller."""
        self._last_result = (success, message)
        self._goal_done_event.set()