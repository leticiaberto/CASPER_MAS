import time
from src.entities.ContextualSkill import Context
from robots.Pepper import Pepper
from robots.FrankaResearch3 import FrankaResearch3
import argparse
import yaml

def main():
    # ===================================================
    # Load config file
    # ===================================================
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default="configs/default.yaml")
    args = parser.parse_args()

    with open(args.config, "r") as f:
        config = yaml.safe_load(f)

    print("Loaded config:", args.config)

    debug = config.get("debug", False)

    robot_model = config["robot_model"]
    robot_id = config["robot_id"]
    skill_weights = config["skill_weights"]

    # Automatically get contexts
    contexts = list(skill_weights.keys())
    
    # ===================================================
    # Instantiate robot
    # ===================================================
    if(robot_model == "Pepper"):
        robot = Pepper(
            id=robot_id, 
            skill_weights = skill_weights,
            contexts = contexts,
        )
    elif(robot_model == "FrankaResearch3"):
        robot = FrankaResearch3(
            id=robot_id, 
            skill_weights = skill_weights,
            contexts = contexts, 
        )
    else:
        raise ValueError(f"Unknown robot model: {robot_model}")
        
    print(f"Robot {robot_id} starting up...")

    if(debug):
        robot.skills.print_skills_preferences()
    add = False
    try:
        robot.startup()
        # Keep main thread alive
        while True:
            # ===================================================
            # Add partners
            # ===================================================
            if (not add):
                for i in range(100000000):
                    if(i == 800000):
                        robot.export_data()
                        add = True

                        delivery = Context("delivery", {"manipulation": 1.0})
                        assembly = Context("assembly", {"manipulation": 1.0})

                        print(robot.evaluate(delivery))   
                        print(robot.evaluate(assembly))
            time.sleep(10)
    except KeyboardInterrupt:
        print("Stopping robot...")

    finally:
        robot.closeComm()

main()

