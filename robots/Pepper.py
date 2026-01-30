from src.entities.Agent import Agent

class Pepper(Agent):
    def __init__(self, id, skill_weights, contexts):
        constraints={
            "can_move": True,
            "can_manipulate": False,
            "can_show_media": True,
            "can_transport_objects": True, # refers to ability to carry small objects while navigating
            "max_payload_kg": 1, # refers to maximum weight the robot can carry while navigating
            "workspace": "global"
        }
        super().__init__(id, constraints, skill_weights, contexts)