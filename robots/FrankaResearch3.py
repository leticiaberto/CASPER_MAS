from src.entities.Agent import Agent

class FrankaResearch3(Agent):
    def __init__(self, id, skill_weights, contexts, role, teamsize):
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

    def _execute_task_specific(self, task):
        print(f"{self.id} executing {task}")
        if task == "grasp":
            return self._grasp(task)
        elif task == "move_arm":
            return self._move_arm(task)
        #else:
            #raise NotImplementedError
        
    def _grasp(self, task):
        print(f"{task} Not Implemented")

    def _move_arm(self, task):
        print(f"{task} Not Implemented")