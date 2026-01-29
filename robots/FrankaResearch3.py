from Agent import Agent

class FrankaResearch3(Agent):
    def __init__(self, id, skill_weights, contexts):
        constraints={
            "can_move": False,
            "can_manipulate": True,
            "max_size_object_cm": 8, # refers to maximum size of object that can be manipulated
            "max_payload_kg": 3,
            "max_reach_cm": 85,
            "can_transport_objects": False,
            "workspace": "station_A"
        }
        super().__init__(id, constraints, skill_weights, contexts)