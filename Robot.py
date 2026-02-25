import time
from src.entities.ContextualSkill import Context
from robots.Pepper import Pepper
from robots.FrankaResearch3 import FrankaResearch3
import argparse
import yaml

def main():
    # ===================================================
    # Load config file and args
    # ===================================================
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default="configs/robots/default.yaml")
    parser.add_argument("--role", type=str, default="member", choices=["member", "supervisor"])
    parser.add_argument("--exp", type=str, default="configs/exp1.yaml")

    args = parser.parse_args()

    with open(args.robot, "r") as f:
        robot_config = yaml.safe_load(f)

    print("Loaded config:", args.robot)

    debug = robot_config.get("debug", False)

    robot_model = robot_config["robot_model"]
    robot_id = robot_config["robot_id"]
    skill_weights = robot_config["skill_weights"]

    contexts = list(skill_weights.keys())

    # Only one robot is the supervisor, others are members
    agent_role = args.role
    print(f"Agent role: {agent_role}")  

    with open(args.exp, "r") as f:
        exp_config = yaml.safe_load(f)
    
    task_file = exp_config["task"]
    print(f"Task file: {task_file}")

    teamsize = exp_config["team_size"]
    top_k = exp_config["top_k"]
    optimizeMode = exp_config["optimizeMode"]
    
    # ===================================================
    # Instantiate robot
    # ===================================================
    if(robot_model == "Pepper"):
        agent = Pepper(
            id=robot_id, 
            skill_weights = skill_weights,
            contexts = contexts,
            role = agent_role,
            teamsize = teamsize,
        )
    elif(robot_model == "FrankaResearch3"):
        agent = FrankaResearch3(
            id=robot_id, 
            skill_weights = skill_weights,
            contexts = contexts, 
            role = agent_role,
            teamsize = teamsize,
        )
    else:
        raise ValueError(f"Unknown agent model: {robot_model}")
        
    print(f"Agent {robot_id} starting up...")

    if(debug):
        agent.skills.print_skills_preferences()

    # Start task graph
    agent.load_goal(task_file)
    teamComplete = False
    ready = False
    
    try:
        agent.startup()# 1. Start listener (for adding partners), 2. Announce hello, 3. Send my skills
        time.sleep(2)
        # Keep main thread alive
        while True:
            # Gets the skills and preferences of all the members of the team first
            if(not ready):
                if(len(agent.partners) == agent.teamSize -1):
                    teamComplete = True
                    #agent.print_partners()
                if teamComplete:
                    # Add yourself as agent to be considered in the task allocation
                    all_agents = {agent.id: agent.skills} | agent.partners
                    agent.allocate_task(all_agents, optimizeMode, top_k, debug)
                    time.sleep(3)
                    ready = True
            else:
                #if(first):
                time.sleep(5)
                agent.step()
                

    except KeyboardInterrupt:
        print("Stopping agent...")

    finally:
        agent.closeComm()

main()
""""
agent.export_data()
delivery = Context("delivery", {"manipulation": 1.0})
assembly = Context("assembly", {"manipulation": 1.0})

print(agent.evaluate(delivery))   
print(agent.evaluate(assembly))
"""

