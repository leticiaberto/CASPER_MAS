import time
from src.entities.ContextualSkill import Context
from robots.Pepper import Pepper
from robots.FrankaResearch3 import FrankaResearch3
import argparse
import yaml
from src.entities.Supervisor import Supervisor
from src.graph.Graph import Graph, GraphVisualizer

def main():
    # ===================================================
    # Load config file and args
    # ===================================================
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default="configs/robots/default.yaml")
    parser.add_argument("--role", type=str, default="member", choices=["member", "supervisor"])
    parser.add_argument("--task", type=str, default="configs/goals/default_graph.json")
    parser.add_argument("--teamsize", type=int, default=3)
    parser.add_argument("--topk", type=int, default=2)

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

    # Only one robot is the supervisor, others are members
    agent_role = args.role
    print(f"Agent role: {agent_role}")  
    
    task_file = args.task
    print(f"Task file: {task_file}")

    teamsize = args.teamsize
    top_k = args.topk
    
    # ===================================================
    # Instantiate robot
    # ===================================================
    if(robot_model == "Pepper"):
        robot = Pepper(
            id=robot_id, 
            skill_weights = skill_weights,
            contexts = contexts,
            role = agent_role,
        )
    elif(robot_model == "FrankaResearch3"):
        robot = FrankaResearch3(
            id=robot_id, 
            skill_weights = skill_weights,
            contexts = contexts, 
            role = agent_role,
        )
    else:
        raise ValueError(f"Unknown robot model: {robot_model}")
        
    print(f"Robot {robot_id} starting up...")

    if(debug):
        robot.skills.print_skills_preferences()
    
    if(robot.supervisor):
        robot.supervisor.set_teamsize(teamsize)

    # ===================================================
    # Start task graph
    # ===================================================
    # All agents know the task graph, but only the supervisor will score agents (the others can, but will not do it here for simplicity)
    task_graph = Graph()
    task_graph.load_task_graph(task_file)

    if(robot.supervisor):
        robot.supervisor.assign_agents_to_tasks(task_graph.G, [robot], "skill", top_k)
        robot.supervisor.graph_visualizer.export_multiagent_graph(task_graph.G, palette_mode="pastel")
        
    add = False
    try:
        robot.startup()# 1. Start listener (for adding partners), 2. Announce hello, 3. Send my skills, 4. Request skills from others
        # Keep main thread alive
        while True:
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

