from Agent import Agent
from Context import Context

class FrankaResearch3(Agent):
    def __init__(self, id, contexts):
        skills = ["manipulation"]
        constraints={
            "can_move": False,
            "can_manipulate": True,
            "max_size_object_cm": 8, # refers to maximum size of object that can be manipulated
            "max_payload_kg": 3,
            "max_reach_cm": 85,
            "can_transport_objects": False,
            "workspace": "station_A"
        }
        super().__init__(id, constraints, skills, contexts)

def main():
    contexts = ["delivery", "assembly"]
    
    panda_arm_robot = FrankaResearch3(
        id="panda_arm_1", 
        contexts = contexts
    )

    # Setting skill levels for different contexts
    panda_arm_robot.skills.set_skill("manipulation", "delivery", 0.4)
    panda_arm_robot.skills.set_skill("manipulation", "assembly", 1.0)

    # Setting preferences for different contexts
    panda_arm_robot.skills.set_preference("manipulation", "delivery", 1.0)
    panda_arm_robot.skills.set_preference("manipulation", "assembly", 0.8)

    delivery = Context("delivery", {"manipulation": 1.0})
    assembly = Context("assembly", {"manipulation": 1.0})

    print(panda_arm_robot.evaluate(delivery))   
    print(panda_arm_robot.evaluate(assembly))

    # ===================================================
    # Add partners
    # ===================================================
    """panda_arm_robot.add_partner(
        partner_id="pepper_mobile_1",
        skills=pepper_mobile_robot.skills,
        contexts=contexts
    )
    panda_arm_robot.print_partners()"""

main()