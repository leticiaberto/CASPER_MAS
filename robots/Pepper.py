from Agent import Agent
from Context import Context

class Pepper(Agent):
    def __init__(self, id, skill_weights, contexts):
        skills = ["manipulation", "navigation", "show_visual_media", "transport_objects"]
        constraints={
            "can_move": True,
            "can_manipulate": False,
            "can_show_media": True,
            "can_transport_objects": True, # refers to ability to carry small objects while navigating
            "max_payload_kg": 1, # refers to maximum weight the robot can carry while navigating
            "workspace": "global"
        }
        super().__init__(id, constraints, skill_weights, contexts)
    

def main():
    contexts = ["delivery", "assembly"] 

    # Setting skill levels and preferences for different contexts. weight[0] is the skill level, weight[1] is the preference
    skill_weights = {
        "delivery": {
            "navigation": [0.6, 0.1],
            "manipulation": [0.2, 0.6],
            "planning": [0.2, 0.3]
        },
        "assembly": {
            "navigation": [0.1, 0.8],
            "manipulation": [0.6, 0.2],
            "planning": [0.3, 0.5]
        }
    }
    
    pepper_mobile_robot = Pepper(
        id="pepper_mobile_1", 
        skill_weights = skill_weights,
        contexts = contexts
    )

    print(f"Pepper Robot {pepper_mobile_robot.id} starting up...")
    pepper_mobile_robot.skills.print_skills_preferences()
    
    delivery = Context("delivery", {"manipulation": 1.0})
    assembly = Context("assembly", {"manipulation": 1.0})

    print(pepper_mobile_robot.evaluate(delivery))   
    print(pepper_mobile_robot.evaluate(assembly))

main()

