"""
========
Unified entry point for all robot agents (Pepper, FrankaResearch3, Tiago, Human).

Every robot model follows the same launch pattern:
  1. If use_sim=True → spawn the robot in Gazebo (non-blocking Popen)
  2. Wait spawn_delay seconds for the simulator to settle
  3. Instantiate the agent class (connects ROS2 adapter)
  4. Run the task-graph loop

Spawn is always handled HERE, not inside the agent class, so the pattern
is identical for every model and easy to extend.

Robot yaml keys
---------------
  robot_model   : Pepper | FrankaResearch3 | Tiago | Human
  robot_id      : unique name / namespace (e.g. "tiago_robot1", "host")
  x_pos         : spawn X  (metres, string)
  y_pos         : spawn Y  (metres, string)
  z_pos         : spawn Z  (metres, string)   — ignored for Human
  yaw           : spawn yaw (radians, string)
  spawn_delay   : seconds to wait after launch before connecting (string)
  skill_weights : dict[context -> weight]
  debug         : bool (optional, default false)
  # Human-only
  actor_type    : WalkingActor | CasualFemale | FemaleVisitor
  color         : red | blue | …  (leave empty for default model)
"""

import argparse
import subprocess
import time
from tkinter.filedialog import test
import yaml
from pathlib import Path
import fcntl

import ros2_path_setup
from utils import Roles  # noqa: F401 — registers all ROS2 adapter packages


# ---------------------------------------------------------------------------
# Lazy imports — only load what's needed so missing packages don't crash
# unrelated models.
# ---------------------------------------------------------------------------

def _import_agent(robot_model: str):
    if robot_model == "Pepper":
        from robots.Pepper import Pepper
        return Pepper
    if robot_model == "FrankaResearch3":
        from robots.FrankaResearch3 import FrankaResearch3
        return FrankaResearch3
    if robot_model == "Tiago":
        from robots.Tiago import Tiago
        return Tiago
    if robot_model == "Human":
        from robots.Human import Human
        return Human
    raise ValueError(
        f"Unknown robot_model '{robot_model}'. "
        "Valid: Pepper | FrankaResearch3 | Tiago | Human"
    )


# ---------------------------------------------------------------------------
# Per-model Gazebo launch helpers
# ---------------------------------------------------------------------------

def _launch_fr3(robot_id, x_pos, y_pos, z_pos, yaw, spawn_delay):
    """Generate config then launch FR3 in sim."""
    subprocess.run([
        "python3",
        "ros2_packages/fr3_adapters/scripts/fr3_generate_config.py",
        robot_id,
    ], check=True)
    cmd = [
        "ros2", "launch", "fr3_adapters", "fr3_sim.launch.py",
        f"robot_name:={robot_id}",
        f"x_pos:={x_pos}",
        f"y_pos:={y_pos}",
        f"z_pos:={z_pos}",
        f"yaw:={yaw}",
        f"spawn_delay:={spawn_delay}",
    ]
    print(f"[Robot] Launching FR3: {' '.join(cmd)}")
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def _launch_tiago(robot_id, world_name, x_pos, y_pos, z_pos, yaw, spawn_delay):
    cmd = [
        "ros2", "launch", "tiago_adapters", "tiago_sim.launch.py",
        f"robot_name:={robot_id}",
        f"world_name:={world_name}",
        f"x_pos:={x_pos}",
        f"y_pos:={y_pos}",
        f"z_pos:={z_pos}",
        f"yaw:={yaw}",
        f"spawn_delay:={spawn_delay}",
    ]
    print(f"[Robot] Launching Tiago: {' '.join(cmd)}")
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def _launch_pepper(robot_id, x_pos, y_pos, z_pos, yaw, spawn_delay):
    cmd = [
        "ros2", "launch", "pepper_adapters", "pepper_sim.launch.py",
        f"robot_name:={robot_id}",
        f"x_pos:={x_pos}",
        f"y_pos:={y_pos}",
        f"z_pos:={z_pos}",
        f"yaw:={yaw}",
        f"spawn_delay:={spawn_delay}",
    ]
    print(f"[Robot] Launching Pepper: {' '.join(cmd)}")
    return subprocess.Popen(cmd)


def _launch_human(robot_id, world_name, actor_type, color, x_pos, y_pos, yaw):
    cmd = [
        "ros2", "launch", "human_adapters", "actor.launch.py",
        f"actor_name:={robot_id}",
        f"world_name:={world_name}",
        f"actor_type:={actor_type}",
        f"color:={color}",
        f"x:={x_pos}",
        f"y:={y_pos}",
        f"yaw:={yaw}",
    ]
    print(f"[Robot] Launching Human: {' '.join(cmd)}")
    return subprocess.Popen(cmd)


# ---------------------------------------------------------------------------
# Agent instantiation (no spawn logic here — spawn is done above)
# ---------------------------------------------------------------------------

def _make_agent(robot_model, AgentClass, robot_id, world_name,
                skill_weights, contexts, agent_role, teamsize,
                use_sim, result_timeout, workspace, party_duration, guests, run_id):
    """Instantiate the correct agent class with its specific parameters."""

    if robot_model == "Pepper":
        return AgentClass(
            robot_name    = robot_id,
            skill_weights = skill_weights,
            contexts      = contexts,
            role          = agent_role,
            teamsize      = teamsize,
            use_sim       = use_sim,
            workspace     = workspace,
            party_duration = party_duration,
            guests         = guests,
            run_id         = run_id,
        )

    if robot_model == "FrankaResearch3":
        return AgentClass(
            robot_name    = robot_id,
            world_name    = world_name,
            skill_weights = skill_weights,
            contexts      = contexts,
            role          = agent_role,
            teamsize      = teamsize,
            use_sim       = use_sim,
            workspace     = workspace,
            party_duration = party_duration,
            guests         = guests,
            run_id         = run_id,
        )

    if robot_model == "Tiago":
        return AgentClass(
            robot_name    = robot_id,
            world_name    = world_name,
            skill_weights = skill_weights,
            contexts      = contexts,
            role          = agent_role,
            teamsize      = teamsize,
            use_sim       = use_sim,
            workspace     = workspace,
            party_duration = party_duration,
            guests         = guests,
            run_id         = run_id,
        )

    if robot_model == "Human":
        return AgentClass(
            actor_name    = robot_id,
            skill_weights = skill_weights,
            contexts      = contexts,
            role          = agent_role,
            teamsize      = teamsize,
            result_timeout= result_timeout,
            use_sim       = use_sim,
            workspace     = workspace,
            party_duration = party_duration,
            guests         = guests,
            run_id         = run_id,
        )

    raise ValueError(f"Unknown robot_model: '{robot_model}'")


# ---------------------------------------------------------------------------
# Gazebo ↔ ROS2 bridge (shared, world-level topics)
# ---------------------------------------------------------------------------

_BRIDGE_NODE_NAME = "/gz_ros2_bridge"   # the node name we give the bridge

def _bridge_already_running() -> bool:
    """Return True if a gz-ros2 bridge node is already up."""
    try:
        nodes = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True, text=True, timeout=5
        ).stdout.splitlines()
        if _BRIDGE_NODE_NAME not in nodes:
            return False

        services = subprocess.run(
            ["ros2", "service", "list"],
            capture_output=True, text=True, timeout=5
        ).stdout
        return "/set_pose" in services
    except Exception:
        return False


def _launch_gz_ros2_bridge(world_name: str):
    """
    Start the shared gz-ros2 bridge if not already running.

    Returns the Popen handle if THIS call started the bridge,
    or None if it was already running (owned by another process).

    IMPORTANT: callers must NOT terminate this process on shutdown
    unless they are certain no other robot is still using it.
    The bridge is shared across all robots in the same world.
    """
    lock_path = "/tmp/gz_ros2_bridge.lock"
    lock_file = open(lock_path, "w")
    fcntl.flock(lock_file, fcntl.LOCK_EX)   # blocks until safe to proceed
    try:
        if _bridge_already_running():
            print("[Robot] gz-ros2 bridge already running — skipping launch.")
            return None

        cmd = [
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            f"/world/{world_name}/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            f"/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
            "--ros-args", "-r", f"__node:={_BRIDGE_NODE_NAME.lstrip('/')}",
        ]
        
        print(f"[Robot] Launching gz-ros2 bridge: {' '.join(cmd)}")
        return subprocess.Popen(cmd)
    finally:
        fcntl.flock(lock_file, fcntl.LOCK_UN)
        lock_file.close()

# ---------------------------------------------------------------------------
# Spawn readiness helpers
# ---------------------------------------------------------------------------

def _wait_for_spawner_chain(launch_proc: subprocess.Popen, timeout: float = 120.0) -> bool:
    """
    Block until the gripper spawner (spawner-5, last in the chain) reports
    'process has finished cleanly' in the launch output.

    This is the exact signal that all three controllers are active — no polling,
    no ANSI-code parsing, no extra ROS2 CLI calls. Reacts within milliseconds
    of spawner-5 finishing.

    Requires the launch process to be started with stdout=subprocess.PIPE.
    """
    sentinel = b"spawner-5]: process has finished cleanly"
    deadline = time.time() + timeout
    print("[Robot] Waiting for gripper spawner to finish ...")
    buf = b""
    while time.time() < deadline:
        if launch_proc.poll() is not None:
            # Drain whatever is left in the pipe before giving up
            buf += launch_proc.stdout.read()
            if sentinel in buf:
                print("[Robot] Gripper spawner finished — all controllers active.")
                return True
            print("[Robot] WARNING: launch process exited before gripper spawner finished.")
            return False
        chunk = launch_proc.stdout.read1(4096)   # non-blocking read of available bytes
        if chunk:
            buf += chunk
            # Print to terminal so the launch output is still visible
            print(chunk.decode(errors="replace"), end="", flush=True)
            if sentinel in buf:
                print("\n[Robot] Gripper spawner finished — all controllers active.")
                return True
        else:
            time.sleep(0.1)
    print(f"[Robot] WARNING: gripper spawner did not finish within {timeout:.0f}s.")
    return False


def _wait_for_actor(robot_id: str, timeout: float = 45.0) -> bool:
    """
    Block until /<robot_id>/robot_state_publisher appears in the ROS2 node list.
    Used for Human actors (no controller_manager).
    """
    import time as _time
    target   = f"/{robot_id}/robot_state_publisher"
    deadline = _time.time() + timeout
    print(f"[Robot] Waiting for actor '{robot_id}' RSP node to appear ...")
    while _time.time() < deadline:
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True, text=True,
        )
        if target in result.stdout.splitlines():
            print(f"[Robot] Actor '{robot_id}' is ready.")
            return True
        _time.sleep(2.0)
    print(f"[Robot] WARNING: actor '{robot_id}' RSP did not appear within {timeout:.0f}s.")
    return False


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    Path("data").mkdir(parents=True, exist_ok=True)

    # ── Args ────────────────────────────────────────────────────────────────
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="fr3_arm_1")
    parser.add_argument("--role",  default="member",
                        choices=["member", "supervisor"])
    parser.add_argument("--exp",   default="exp1")
    args = parser.parse_args()

    # ── Load configs ─────────────────────────────────────────────────────────
    with open('configs/robots/' + args.robot + ".yaml") as f:
        robot_config = yaml.safe_load(f)
    with open('configs/exps/' + args.exp + ".yaml") as f:
        exp_config = yaml.safe_load(f)

    print(f"[Robot] Loaded robot config : {args.robot}")
    print(f"[Robot] Loaded exp config   : {args.exp}")

    # ── Robot config ─────────────────────────────────────────────────────────
    robot_model  = robot_config["robot_model"]
    robot_id     = robot_config["robot_id"]
    debug        = robot_config.get("debug", False)

    x_pos        = str(robot_config.get("x_pos",       "0.0"))
    y_pos        = str(robot_config.get("y_pos",       "0.0"))
    z_pos        = str(robot_config.get("z_pos",       "1.0"))
    yaw          = str(robot_config.get("yaw",         "0.0"))
    spawn_delay  = float(robot_config.get("spawn_delay", 12.0))

    workspace    = robot_config.get("workspace", None)

    # Human-specific (ignored by other models)
    actor_type   = robot_config.get("actor_type", "WalkingActor")
    actor_color  = robot_config.get("color",      "")

    skill_weights = robot_config["skill_weights"]
    contexts      = list(skill_weights.keys())

    result_timeout = float(robot_config.get("result_timeout", 120.0))

    # ── Exp config ───────────────────────────────────────────────────────────
    task_file    = exp_config["task"]
    teamsize     = exp_config["team_size"]
    top_k        = exp_config["top_k"]
    optimizeMode = exp_config["optimizeMode"]
    use_sim      = exp_config["use_sim"]
    world_name   = exp_config.get("world_name", "backyard")
    party_duration = exp_config.get("party_duration", 3600)
    supervisor_team = exp_config.get("supervisor_team", True)
    guests        = exp_config.get("guests", 0)
    
    agent_role   = args.role

    print(f"[Robot] model={robot_model}  id={robot_id}  role={agent_role}  "
          f"use_sim={use_sim}  world={world_name}")

    # ── 1. Spawn in Gazebo (non-blocking) ────────────────────────────────────
    launch_process = None
    if use_sim:
        if robot_model == "FrankaResearch3":
            launch_process = _launch_fr3(
                robot_id, x_pos, y_pos, z_pos, yaw, spawn_delay)

        elif robot_model == "Tiago":
            launch_process = _launch_tiago(
                robot_id, world_name, x_pos, y_pos, z_pos, yaw, spawn_delay)

        elif robot_model == "Pepper":
            launch_process = _launch_pepper(
                robot_id, x_pos, y_pos, z_pos, yaw, spawn_delay)

        elif robot_model == "Human":
            launch_process = _launch_human(
                robot_id, world_name, actor_type, actor_color,
                x_pos, y_pos, yaw)

        bridge_process = _launch_gz_ros2_bridge(world_name)

        if launch_process is not None:
            # Wait for the robot to be genuinely ready before connecting the agent.
            #
            # FR3 / Tiago: poll until controller_manager is visible in the ROS2
            # graph, then poll until all three controllers (JSB, arm, gripper) are
            # in 'active' state. This replaces the blind sleep and correctly
            # handles slow machines where the spawner retries (3×10s per controller).
            #
            # Human: poll until robot_state_publisher is up (no controller_manager).
            #
            # Pepper (and unknown models): fall back to the YAML spawn_delay.
            if robot_model in ("FrankaResearch3", "Tiago"):
                _wait_for_spawner_chain(launch_process)

            elif robot_model == "Human":
                _wait_for_actor(robot_id)
                time.sleep(2.0)

            else:
                # Pepper / unknown — no reliable ROS2 readiness signal
                print(f"[Robot] Waiting {spawn_delay:.0f} s for Gazebo to settle ...")
                time.sleep(spawn_delay)

    # ── 2. Import + instantiate agent ────────────────────────────────────────
    AgentClass = _import_agent(robot_model)

    agent = _make_agent(
        robot_model   = robot_model,
        AgentClass    = AgentClass,
        robot_id      = robot_id,
        world_name    = world_name,
        skill_weights = skill_weights,
        contexts      = contexts,
        agent_role    = agent_role,
        teamsize      = teamsize,
        use_sim       = use_sim,
        result_timeout = result_timeout,
        workspace      = workspace,
        party_duration  = party_duration,
        guests         = guests,
        run_id =        args.exp,
    )

    print(f"[Robot] Agent '{robot_id}' instantiated.")

    # Blocking msgs to test connection and debug
    test = False
    if test:
        if (robot_model == "FrankaResearch3"):
            ok, msg = agent.pick_and_place(pick_name="tomato_1", place_xyz=(1.0, 5.20, 0.875))
            #ok, msg = agent.pick_and_place((0.56, 0.0004, 0.0350), (0.0094, -0.7, 0.0))
            #ok, msg = agent.pick_and_place(pick_name="meat_1", place_name="plate_1")
            #ok, msg = agent.pick_and_place(pick_name="tomato_1", place_xyz=(0.5, 0.4, 0.3))
        elif (robot_model == "Human"):
            ok, msg = agent.goto(x=0, y=0, final_yaw=3.14)
            #ok, msg = agent.goto(location = "DiningTable")
            pass
        elif (robot_model == "Tiago"):
            ok, msg = agent.pick_and_place_objects([("drink_3",   "bench_r2"),])

    # ── 3. Debug info ─────────────────────────────────────────────────────────
    if debug:
        agent.skills.print_skills_preferences()

    # ── 4. Task graph loop ───────────────────────────────────────────────────
    agent.load_goal(task_file)
    team_complete = False
    ready         = False

    try:
        # 1. Start listener, announce hello, send skills
        agent.startup()
        time.sleep(2)

        while True:
            # Drain the message queue on every iteration — this is the only
            # place on the main thread that processes incoming ZMQ messages
            # during the startup handshake (partner discovery) phase.
            # Once ready=True, step() calls spin_once() itself at the top.
            if not ready:
                agent.spin_once()

            # Gets the skills and preferences of all the members of the team first
            if not ready:
                if len(agent.partners) == agent.teamSize - 1:
                    team_complete = True
                    #agent.print_partners()
                if team_complete:
                    if supervisor_team:# Supervisor has tasks too -- Add yourself as agent to be considered in the task allocation
                        all_agents = {agent.id: agent.skills} | agent.partners
                    else:# Supervisor does not have tasks -- Do not add yourself as agent to be considered in the task allocation
                        all_agents = {
                            pid: skills
                            for pid, skills in agent.partners.items()
                            if agent.partners.get(pid).role != Roles.SUPERVISOR
                        }
                
                    agent.allocate_task(all_agents, optimizeMode, top_k, debug)
                    time.sleep(5)
                    ready = True
            else:
                time.sleep(5)
                agent.step()
                if agent.goal_finished:
                    print("[Robot] Goal finished — stopping agent.")
                    break

    except KeyboardInterrupt:
        print("[Robot] KeyboardInterrupt — stopping agent.")

    finally:
        agent.closeComm()
        agent.shutdown()
        if launch_process is not None:
            print("[Robot] Terminating Gazebo launch process.")
            launch_process.terminate()
            # NOTE: the gz-ros2 bridge is intentionally NOT terminated here.
            # It is a shared process that may be used by other robots still
            # running in parallel (e.g. Tiago + FR3).  It will be cleaned up
            # automatically when the container / shell session ends.

            # restart it manually:
            # pkill -f "parameter_bridge"

main()
""""
agent.export_data()
delivery = Context("delivery", {"manipulation": 1.0})
assembly = Context("assembly", {"manipulation": 1.0})

print(agent.evaluate(delivery))   
print(agent.evaluate(assembly))
"""