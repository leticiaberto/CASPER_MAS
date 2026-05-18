#!/usr/bin/env python3
"""
launch_experiment.py
====================
Launches the full simulation experiment by reading agents, supervisor, and
world from an experiment YAML file (default: configs/exps/exp1.yaml).

Launch sequence
---------------
  1. Get IP address  (PubSubProxy/Get_IP_Adress.py — blocking, updates network config)
  2. Open UFW ports  (sudo ufw allow <pub_port>/tcp  &  sudo ufw allow <sub_port>/tcp)
  3. Hub             (PubSubProxy/Hub.py — started and given 3s to bind)
  4. Gazebo world    (ros2 launch simulation world.launch.py world_name:=<world>)
  5. Agents          (Robot.py per agent, staggered, supervisor role assigned
                      to whichever agent matches the 'supervisor' field)

Network config
--------------
  Read from configs/network.yaml:
    pub_port: 5555
    sub_port: 5556
    hub_ip:   172.17.0.2

Validation (raises errors before anything launches)
----------------------------------------------------
  • team_size must equal the number of agents listed
  • supervisor must be present in the agents list
  • supervisor field must exist and not be empty

Crash policy
------------
Any process exiting unexpectedly triggers a full SIGTERM → SIGKILL shutdown
of every other process, then this script exits. Restart from scratch.

Inter-process delays
--------------------
  --world-settle  : seconds after Gazebo launch before the first robot spawns
  --robot-stagger : seconds between successive Robot.py launches

Usage
-----
    python3 launch_experiment.py
    python3 launch_experiment.py --exp configs/exps/exp2.yaml
    python3 launch_experiment.py --world-settle 30 --robot-stagger 20
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml


# ---------------------------------------------------------------------------
# Tunable defaults (overridable via CLI)
# ---------------------------------------------------------------------------
DEFAULT_EXP_FILE      = "configs/exps/exp1.yaml"
DEFAULT_NETWORK_FILE  = "configs/network.yaml"
DEFAULT_WORLD_SETTLE  = 10    # seconds for Gazebo to be ready before first spawn
DEFAULT_ROBOT_STAGGER = 35    # seconds between successive Robot.py launches
                              # Must cover the full FR3 controller chain including
                              # spawner retries (3×10s per controller = up to 90s
                              # worst case, but 60s covers the typical case).
                              # this stagger just prevents
                              # parallel Gazebo spawn requests.


# ---------------------------------------------------------------------------
# Network YAML loading
# ---------------------------------------------------------------------------

def load_network_config(network_file: str) -> dict:
    """
    Load configs/network.yaml and return pub_port, sub_port, hub_ip.

    Expected structure:
        pub_port: 5555
        sub_port: 5556
        hub_ip:   172.17.0.2
    """
    path = Path(network_file)
    if not path.exists():
        _die(f"Network config not found: {network_file}")

    with path.open() as f:
        cfg = yaml.safe_load(f)

    for field in ("pub_port", "sub_port", "hub_ip"):
        if field not in cfg or cfg[field] is None:
            _die(f"Missing required field '{field}' in {network_file}")

    return {
        "pub_port": int(cfg["pub_port"]),
        "sub_port": int(cfg["sub_port"]),
        "hub_ip":   str(cfg["hub_ip"]).strip(),
    }


# ---------------------------------------------------------------------------
# Experiment YAML loading & validation
# ---------------------------------------------------------------------------

def load_and_validate_exp(exp_file: str) -> dict:
    """
    Load the experiment YAML and validate it before anything is launched.
    Raises SystemExit with a clear message on any validation failure.

    Expected YAML structure
    -----------------------
    team_size: 4
    agents:
      - human_host
      - tiago
      - fr3_arm_1
      - fr3_arm_2
    supervisor: human_host
    world_name: backyard
    """
    path = Path(exp_file)
    if not path.exists():
        _die(f"Experiment file not found: {exp_file}")

    with path.open() as f:
        cfg = yaml.safe_load(f)

    # ── Required fields present ──────────────────────────────────────────────
    for field in ("team_size", "agents", "supervisor"):
        if field not in cfg or cfg[field] is None:
            _die(f"Missing required field '{field}' in {exp_file}")

    # ── Type check: agents must be a proper YAML list ────────────────────────
    if not isinstance(cfg["agents"], list):
        _die(
            f"'agents' must be a YAML list (each entry prefixed with '- ').\n"
            f"  Got type: {type(cfg['agents']).__name__}\n"
            f"  Correct format:\n"
            f"    agents:\n"
            f"      - human_host\n"
            f"      - tiago\n"
            f"      - fr3_arm_1\n"
            f"      - fr3_arm_2"
        )

    agents     = [str(a).strip() for a in cfg["agents"] if str(a).strip()]
    team_size  = int(cfg["team_size"])
    supervisor = str(cfg["supervisor"]).strip()

    # ── Semantic validation ──────────────────────────────────────────────────
    if not agents:
        _die("'agents' list is empty — nothing to launch.")

    if not supervisor:
        _die("'supervisor' is empty — a supervisor must be designated.")

    if len(agents) != team_size:
        _die(
            f"team_size={team_size} but {len(agents)} agent(s) listed: {agents}\n"
            f"  Fix 'team_size' or the 'agents' list in {exp_file}."
        )

    if supervisor not in agents:
        _die(
            f"supervisor='{supervisor}' is not in the agents list: {agents}\n"
            f"  The supervisor must be one of the listed agents."
        )

    # ── Attach parsed values for convenience ─────────────────────────────────
    cfg["_agents"]     = agents
    cfg["_supervisor"] = supervisor
    cfg["_world"]      = str(cfg.get("world_name", "default")).strip()
    return cfg


def _die(msg: str) -> None:
    print(f"\n[Launcher] ERROR: {msg}\n", file=sys.stderr)
    sys.exit(1)


def _run(label: str, cmd: list[str]) -> None:
    """Run a command synchronously (blocking). Die on non-zero exit."""
    print(f"[Launcher]  ◆  {label}")
    print(f"            cmd : {' '.join(cmd)}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        _die(f"'{label}' failed with exit code {result.returncode}")


# ---------------------------------------------------------------------------
# Process registry
# ---------------------------------------------------------------------------
_procs: dict[str, subprocess.Popen] = {}


def _launch(label: str, cmd: list[str], log_file: str) -> subprocess.Popen:
    """Start a subprocess, redirect output to a log file, and register it."""
    Path(log_file).parent.mkdir(parents=True, exist_ok=True)
    log = open(log_file, "w")
    print(f"[Launcher]  ▶  {label}")
    print(f"            cmd : {' '.join(cmd)}")
    print(f"            log : {log_file}")
    proc = subprocess.Popen(
        cmd,
        stdout=log,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,   # own process group → clean kill
    )
    _procs[label] = proc
    return proc


def _kill_all(reason: str = "shutdown") -> None:
    """SIGTERM every registered process group, wait 3s, then SIGKILL stragglers."""
    print(f"\n[Launcher] ── {reason} ── terminating all processes ──")
    for label, proc in list(_procs.items()):
        if proc.poll() is None:
            print(f"[Launcher]  ✕  killing {label} (pid {proc.pid})")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
    time.sleep(3)
    for label, proc in list(_procs.items()):
        if proc.poll() is None:
            print(f"[Launcher]  ✕  force-killing {label} (pid {proc.pid})")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    print("[Launcher] All processes terminated.")


def _monitor() -> None:
    """
    Poll all registered processes every 2s.
    Any unexpected exit → kill everything and exit.
    Ctrl-C → clean shutdown.
    """
    print("[Launcher] Monitoring all processes. Ctrl-C to stop.\n")
    try:
        while True:
            time.sleep(2)
            for label, proc in list(_procs.items()):
                rc = proc.poll()
                if rc is not None:
                    _kill_all(reason=f"'{label}' exited with code {rc}")
                    sys.exit(1 if rc != 0 else 0)
    except KeyboardInterrupt:
        _kill_all(reason="KeyboardInterrupt")
        sys.exit(0)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Launch full simulation experiment.")
    parser.add_argument(
        "--exp",
        default=DEFAULT_EXP_FILE,
        help=f"Experiment config YAML (default: {DEFAULT_EXP_FILE})",
    )
    parser.add_argument(
        "--network",
        default=DEFAULT_NETWORK_FILE,
        help=f"Network config YAML (default: {DEFAULT_NETWORK_FILE})",
    )
    parser.add_argument(
        "--world-settle",
        type=float,
        default=DEFAULT_WORLD_SETTLE,
        help=f"Seconds to wait after Gazebo launches before first robot spawn "
             f"(default: {DEFAULT_WORLD_SETTLE})",
    )
    parser.add_argument(
        "--robot-stagger",
        type=float,
        default=DEFAULT_ROBOT_STAGGER,
        help=f"Seconds between successive Robot.py launches "
             f"(default: {DEFAULT_ROBOT_STAGGER})",
    )
    args = parser.parse_args()

    # ── Load & validate configs ──────────────────────────────────────────────
    cfg        = load_and_validate_exp(args.exp)
    net        = load_network_config(args.network)
    agents     = cfg["_agents"]
    supervisor = cfg["_supervisor"]
    world      = cfg["_world"]

    print(f"\n[Launcher] Experiment : {args.exp}")
    print(f"[Launcher] World      : {world}")
    print(f"[Launcher] Agents     : {', '.join(agents)}")
    print(f"[Launcher] Supervisor : {supervisor}")
    print(f"[Launcher] Hub        : {net['hub_ip']}  pub={net['pub_port']}  sub={net['sub_port']}\n")

    Path("logs").mkdir(exist_ok=True)

    # ── 1. Get IP address (blocking) ─────────────────────────────────────────
    _run(
        label = "get_ip_address",
        cmd   = ["python3", "PubSubProxy/Get_IP_Adress.py"],
    )

    # ── 2. Open UFW firewall ports (blocking) ────────────────────────────────
    _run(
        label = f"ufw allow {net['pub_port']}/tcp",
        cmd   = ["sudo", "ufw", "allow", f"{net['pub_port']}/tcp"],
    )
    _run(
        label = f"ufw allow {net['sub_port']}/tcp",
        cmd   = ["sudo", "ufw", "allow", f"{net['sub_port']}/tcp"],
    )

    # ── 3. Hub (background — give it 3s to bind before anything connects) ────
    _launch(
        label    = "hub",
        cmd      = ["python3", "PubSubProxy/Hub.py"],
        log_file = "logs/hub.log",
    )
    print("[Launcher] Waiting 3s for Hub to bind ...\n")
    time.sleep(3)

    # ── 4. Gazebo world ──────────────────────────────────────────────────────
    _launch(
        label    = "gazebo_world",
        cmd      = [
            "ros2", "launch", "simulation", "world.launch.py",
            f"world_name:={world}",
        ],
        log_file = "logs/gazebo_world.log",
    )
    print(f"\n[Launcher] Waiting {args.world_settle:.0f}s for Gazebo to settle ...\n")
    time.sleep(args.world_settle)

    # ── 5. Agents — staggered ────────────────────────────────────────────────
    for i, agent in enumerate(agents):
        config_path = f"configs/robots/{agent}.yaml"
        role        = "supervisor" if agent == supervisor else "member"

        if not Path(config_path).exists():
            _die(
                f"Robot config not found for agent '{agent}': {config_path}\n"
                f"  Create the file or fix the agent name in {args.exp}."
            )

        _launch(
            label    = agent,
            cmd      = [
                "python3", "Robot.py",
                "--robot", config_path,
                "--exp",   args.exp,
                "--role",  role,
            ],
            log_file = f"logs/{agent}.log",
        )

        if i < len(agents) - 1:
            print(f"[Launcher] Waiting {args.robot_stagger:.0f}s before next agent ...\n")
            time.sleep(args.robot_stagger)

    print("\n[Launcher] All agents launched.\n")

    # ── 6. Monitor ───────────────────────────────────────────────────────────
    _monitor()


if __name__ == "__main__":
    main()
