from src.entities.Agent import Agent

class Pepper(Agent):
    def __init__(self, id, skill_weights, contexts, role, teamsize):
        constraints={
            "can_move": True,
            "can_manipulate": False,
            "can_show_media": True,
            "can_transport_objects": True, # refers to ability to carry small objects while navigating
            "max_payload_kg": 1, # refers to maximum weight the robot can carry while navigating
            "workspace": "global"
        }
        super().__init__(id, constraints, skill_weights, contexts, role, teamsize)

    def _execute_task_specific(self, task):
        print(f"{self.id} executing {task}")
        if task == "speak":
            return self._speak(task)
        elif task == "navigate":
            return self._navigate(task)
        elif task == "MoveToTable":
            return self._move_to_table(task)
        #else:# Add when we implemente all the behaviors. Removed for now to not stop the running
            #raise NotImplementedError

    def _move_to_table(self, task):
        print(f"{task} Not Implemented")