from src.entities.Agent import Agent
from ros_adapters.adapters.FrankaMoveItAdapter import FrankaMoveItAdapter

class FrankaResearch3(Agent):
    def __init__(self, id, skill_weights, contexts, role, teamsize, use_sim, agent_agnostic=True):
        constraints={
            "can_move": False,
            "can_manipulate": True,
            "max_size_object_cm": 8, # refers to maximum size of object that can be manipulated
            "max_payload_kg": 3,
            "max_reach_cm": 85,
            "can_transport_objects": False,
            "workspace": "station_A"
        }
        super().__init__(id, constraints, skill_weights, contexts, role, teamsize)
        self.adapter = FrankaMoveItAdapter(use_sim=use_sim, agent_agnostic=agent_agnostic)

    def start(self):
        self.adapter.initialize()
    
    def stop(self):
        self.adapter.shutdown()

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
