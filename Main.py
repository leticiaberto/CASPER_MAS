from Agent import Agent
from Context import Context
from robots.Pepper import Pepper

if __name__ == "__main__":
    contexts = ["delivery", "assembly"]

    # ===================================================
    # Pepper ROBOT SETUP
    # ===================================================
    # General skills the robot possesses
    pepper_skills = ["manipulation", "navigation", "show_visual_media", "transport_objects"]
    
    pepper_mobile_robot = Agent(
        id="pepper_mobile_1", 
        constraints={
            "can_move": True,
            "can_manipulate": False,
            "can_show_media": True,
            "can_transport_objects": True, # refers to ability to carry small objects while navigating
            "max_payload_kg": 1, # refers to maximum weight the robot can carry while navigating
            "workspace": "global"
        }, 
        skills = pepper_skills, 
        contexts = contexts
    )

    # Setting skill levels for different contexts
    pepper_mobile_robot.skills.set_skill("manipulation", "delivery", 0.4)
    pepper_mobile_robot.skills.set_skill("manipulation", "assembly", 0.1)
    pepper_mobile_robot.skills.set_skill("navigation", "delivery", 0.8)

    # Setting preferences for different contexts
    pepper_mobile_robot.skills.set_preference("manipulation", "delivery", 1.0)
    pepper_mobile_robot.skills.set_preference("manipulation", "assembly", 0.8)


    delivery = Context("delivery", {"manipulation": 1.0})
    assembly = Context("assembly", {"manipulation": 1.0})

    print(pepper_mobile_robot.evaluate(delivery))   
    print(pepper_mobile_robot.evaluate(assembly))

    # ===================================================
    # PANDA ARM ROBOT SETUP
    # ===================================================
    # General skills the robot possesses
    panda_skills = ["manipulation"]
    
    panda_arm_robot = Agent(
        id="panda_arm_1", 
        constraints={
            "can_move": False,
            "can_manipulate": True,
            "max_size_object_cm": 8, # refers to maximum size of object that can be manipulated
            "max_payload_kg": 3,
            "max_reach_cm": 85,
            "can_transport_objects": False,
            "workspace": "station_A"
        }, 
        skills = panda_skills, 
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
    panda_arm_robot.add_partner(
        partner_id="pepper_mobile_1",
        skills=pepper_mobile_robot.skills,
        contexts=contexts
    )
    panda_arm_robot.print_partners()