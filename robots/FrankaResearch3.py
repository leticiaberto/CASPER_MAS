"""
FrankaResearch3
==============
High-level agent for a Franka Research 3 arm.
 
- Instantiates FrankaAdapter (from the robots_adapters package) for robot1.
- Builds and dispatches JointTrajectory goals.
- Receives goal results via a callback (non-blocking).
 
Run with:
    python3 FrankaResearch3.py
or via a launch file (recommended for multi-agent setups).
"""
from __future__ import annotations
from src.entities.Agent import Agent
from robots_adapters.FrankaAdapter import FrankaAdapter, RobotMode
import threading
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
 

class FrankaResearch3(Agent):
    def __init__(self, robot_name, skill_weights, contexts, role, teamsize, use_sim, mock=False):
        constraints={
            "can_move": False,
            "can_manipulate": True,
            "max_size_object_cm": 8, # refers to maximum size of object that can be manipulated
            "max_payload_kg": 3,
            "max_reach_cm": 85,
            "can_transport_objects": False,
            "workspace": "station_A"
        }
        super().__init__(robot_name, constraints, skill_weights, contexts, role, teamsize)

        # Internal event used to block send_goal_and_wait()
        self._goal_done_event = threading.Event()
        self._last_result: tuple[bool, str] = (False, "No goal sent yet.")
 
        # Simulation
        adapter = FrankaAdapter(
            robot_name="fr3_robot1",
            mode=RobotMode.SIMULATION,
            result_callback=on_done,
        )

        # Physical robot
        adapter = FrankaAdapter(
            robot_name="fr3_robot1",
            mode=RobotMode.PHYSICAL,
            result_callback=on_done,
            franky_ip="172.16.0.2",        # required for physical
            franky_gripper_speed=0.05,     # optional, defaults shown
            franky_gripper_force=10.0,
        )

        # Same call either way
        adapter.pick_and_place(pick_xyz=(0.5, 0.0, 0.3), place_xyz=(0.5, 0.4, 0.3))

    def start_adapter(self):
        self.adapter.initialize()
    
    def shutdown_adapter(self):
        self.adapter.shutdown()

    def stop_adapter(self):
        self.adapter.stop()

    def _execute_task_specific(self, task):
        print(f"{self.id} executing {task}")
        if task == "grasp":
            return self._grasp(task)
        elif task == "move_arm":
            return self._move_arm(task)
        elif task == "Inspect":
            return self._inspect(task)
        elif task == "pick_and_place": #task["action"] == "pick_and_place"
            return self.adapter.pick_and_place(
                            task["pick"],
                            task["place"]
                        )
        #else:
            #raise NotImplementedError
        
    def _grasp(self, task):
        print(f"{task} Not Implemented")

    def _move_arm(self, task):
        print(f"{task} Not Implemented")
    
    def _inspect(self, task):
        print(f"{task} Not Implemented")