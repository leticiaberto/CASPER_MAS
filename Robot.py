import time
from src.entities.ContextualSkill import Context
from robots.Pepper import Pepper
from robots.FrankaResearch3 import FrankaResearch3
import argparse
import yaml
from pathlib import Path
import subprocess
import os

def launch_robot(launch_file: str, robot_name: str, x_pos: str, 
                 y_pos: str = '0.0', z_pos: str = '1.03', yaw: str = '0.0', spawn_delay: str = '3.0'):
    cmd = [
        'ros2', 'launch', 'robots_adapters', launch_file,
        f'robot_name:={robot_name}',
        f'x_pos:={x_pos}',
        f'y_pos:={y_pos}',
        f'z_pos:={z_pos}',
        f'yaw:={yaw}',
        f'spawn_delay:={spawn_delay}',
    ]
    return subprocess.Popen(cmd)  # non-blocking


def main():
    folder_path = Path("data")
    folder_path.mkdir(parents=True, exist_ok=True) # create folder if it does not exist

    # ===================================================
    # Load config file and args
    # ===================================================
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default="configs/robots/default.yaml")
    parser.add_argument("--role", type=str, default="member", choices=["member", "supervisor"])
    parser.add_argument("--exp", type=str, default="configs/exps/exp1.yaml")

    args = parser.parse_args()

    with open(args.robot, "r") as f:
        robot_config = yaml.safe_load(f)

    print("Loaded config:", args.robot)

    debug = robot_config.get("debug", False)

    robot_model = robot_config["robot_model"]
    robot_id = robot_config["robot_id"]
    
    x_pos = robot_config.get("x_pos", "0.0")
    y_pos = robot_config.get("y_pos", "0.0")
    z_pos = robot_config.get("z_pos", "1.0")
    yaw = robot_config.get("yaw", "0.0")
    spawn_delay = robot_config.get("spawn_delay", "3.0")

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

    USE_SIM = exp_config["USE_SIM"]
    mock = exp_config["mock"]
    print(f"Agent agnostic mode: {mock}, USE_SIM={USE_SIM}")

    launch_file = None

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
            use_sim = USE_SIM,
            mock = mock,
        )
    elif(robot_model == "FrankaResearch3"):
        subprocess.run(["python3", "ros2_packages/robots_adapters/scripts/franka_generate_config.py", f"{robot_id}"])
        launch_file = 'franka_sim.launch.py'
        """agent = FrankaResearch3(
            id=robot_id, 
            skill_weights = skill_weights,
            contexts = contexts, 
            role = agent_role,
            teamsize = teamsize,
            use_sim = USE_SIM,
            mock = mock,
        )"""
    else:
        raise ValueError(f"Unknown agent model: {robot_model}")
    
    ## Launch robot in Gazebo if needed
    if USE_SIM and launch_file is not None:
        process = launch_robot(
                launch_file=launch_file,
                robot_name=robot_id,
                x_pos=x_pos,
                y_pos=y_pos,
                z_pos=z_pos,
                yaw=yaw,
                spawn_delay=spawn_delay
            )
        try:
            process.wait()
        except KeyboardInterrupt:
            process.terminate()

    print(f"Agent {robot_id} starting up...")

    agent.start_adapter()   # start ROS if needed

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
                if agent.goal_finished:
                    print("Goal Finished! Stopping agent...")
                    break
                
    except KeyboardInterrupt:
        print("Stopping agent...")

    finally:
        agent.closeComm()
        agent.shutdown_adapter()     # clean exit
    
    agent.closeComm()
    agent.shutdown_adapter()

main()
""""
agent.export_data()
delivery = Context("delivery", {"manipulation": 1.0})
assembly = Context("assembly", {"manipulation": 1.0})

print(agent.evaluate(delivery))   
print(agent.evaluate(assembly))
"""

