from src.entities.Agent import Agent
#from robots_adapters.PepperAdapter import PepperAdapter


class Pepper(Agent):
    def __init__(self, id, skill_weights, contexts, role, teamsize, use_sim, agent_agnostic=True, workspace=None, party_duration: float = 3600.0, guests: int = 0):
        constraints={
            "can_move": True,
            "can_manipulate": False,
            "can_show_media": True,
            "can_transport_objects": True, # refers to ability to carry small objects while navigating
            "max_payload_kg": 1, # refers to maximum weight the robot can carry while navigating
            "workspace": workspace
        }
        super().__init__(id, constraints, skill_weights, contexts, role, teamsize, party_duration, guests)
        #self.adapter = PepperAdapter(id, use_sim=use_sim, agent_agnostic=agent_agnostic)

    def start(self):
        self.adapter.initialize()
    
    def stop(self):
        self.adapter.shutdown()

    def _execute_task_specific(self, task):
        print(f"{self.id} executing {task}")
        if task == "speak":
            return self._speak(task)
        elif task == "navigate":
            return self._navigate(task)
        elif task == "MoveToTable":
            return self._move_to_table(task)
        if task["action"] == 'pick_and_place':
            self.adapter.pick_and_place(task['pick'], task['place'])
        elif task["action"] == 'move':
            self.adapter.move_to_pose(task['target'])
        elif task["action"] == 'grasp':
            self.adapter.grasp(task['object'])
        #else:# Add when we implemente all the behaviors. Removed for now to not stop the running
            #raise NotImplementedError

    def _move_to_table(self, task):
        print(f"{task} Not Implemented")