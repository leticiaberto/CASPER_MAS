# CASPER_MAS

This project provides a simple **multi-robot communication framework** in Python where robots can:

- run on the same machine or across different computers  
- communicate in real time  
- exchange skills and preferences at startup  
- support late-joining robots  

Robots broadcast their presence and automatically share their skill tables.

---

## ✅ Features

- Background listener thread (robots can do other tasks while listening)
- Robots can join late and still receive partner skills
- Skill exchange uses a clean protocol:

| Message Type                | Purpose                    |
| --------------------------- | -------------------------- |
| `hello`                     | announce robot presence    |
| `skills_update(target=...)` | directed reply with skills |
| `skills_request`            | optional recovery/refresh  |

Directed replies prevent unnecessary updates in other robots.

---

## 📦 Requirements

### Local Installation (Non-Docker)

Dependencies are installed **only when running locally**.

    pip install -r requirements.txt

> ⚠️ When using Docker, dependencies are already installed inside the image.  
> Do **not** run this step when using Docker.

---

## 🌍 Running Across Multiple Computers

### 1. Choose One Machine as the Hub

Find the hub machine IP:

**Linux**

    ip a

**Windows**

    ipconfig

Example hub IP:

    192.168.1.50

---

### 2. Store Hub IP Automatically

You **do not need to edit any network files manually if runnining in the same machine**.

Simply run:

    python Get_IP_Adress.py

This script automatically detects the hub IP and updates:

    configs/network.yaml

Example generated file:

    hub_ip: 192.168.1.50
    pub_port: 5555
    sub_port: 5556

All robots load this configuration automatically at startup.

---

### 3. Open Firewall Ports

The hub uses:

- TCP 5555 (robots publish)
- TCP 5556 (robots subscribe)

Linux UFW example:

    sudo ufw allow 5555/tcp
    sudo ufw allow 5556/tcp

---

## ⚙️ Robot Skills Configuration

Each robot must define its skills and preferences in the `configs/robots/` folder.

Example file: `configs/robots/robot_1.yaml`

Example structure:

    contexts:
      - assembly
      - party

    skills:
      assembly:
        manipulation: [0.9, 0.8]
        navigation:   [0.4, 0.2]

      party:
        navigation:   [0.7, 0.6]

Each skill entry is defined as:

    skill: [skill_level, skill_preference]

- `skill_level` ∈ [0.0 – 1.0]
- `skill_preference` ∈ [0.0 – 1.0]

Robots automatically export this structure and share it at startup.

---

## 🎯 Goal Definition

Before running any experiment, the **goal of the task must be defined**.

Goals are specified as **JSON files** inside: `configs/goals/`

Each goal file defines **what the system should achieve**, independently of:
- the robots involved
- the experimental setup
- execution parameters
- dependencies

Example goal file: `configs/goals/assembly_goal.json`


Goals are **not passed directly to robots**.  
Instead, they are **referenced by experiment configurations**, allowing the same goal to be reused across multiple experiments.

---

## 🧪 Experiment Configuration

Experiments define **how a goal is executed** and with which parameters.

Experiment configuration files are stored in: `configs/exps/`

An experiment configuration:
- references a goal defined in `configs/goals/`
- specifies experiment-specific parameters (e.g., timeouts, policies, constraints, coordination modes)

Example experiment file: `configs/exps/exp1.yaml`

Experiments are specified using the `--exp` flag.

Example:

    python robot.py \
      --robot configs/robots/pepper_mobile_1.yaml \
      --exp configs/exp1.yaml

---

## 🧠 Supervisor Robot

One robot can be launched as a **supervisor** by adding the `--role supervisor` flag.

Example:

    python3 Robot.py \
      --robot configs/robots/pepper_mobile_1.yaml \
      --role supervisor

The supervisor can monitor, coordinate, or manage other agents depending on the experiment design.

---

## 🖥️ Running on One Machine (Local)

### 1. Start the Hub

The hub relays messages between robots:

    python hub.py

### 2. Start Robots

You must explicitly specify **which robot configuration** to use with `--robot`.

    python robot.py --robot configs/robots/panda_arm_1.yaml
    python robot.py --robot configs/robots/pepper_mobile_1.yaml

Robots will automatically discover each other and exchange skills.

---

## 🐳 Running with Docker

Docker images already include all dependencies.

### Launch Containers

Use the provided script:

    ./launch_container.sh

To assign a custom container name:

    ./launch_container.sh <container_name>

Run this command:
- once for **each robot agent**
- once for the **Hub**

Each container runs exactly **one process** (a robot or the hub).

---

## ▶️ Running the System (Summary)

1. Run `Get_IP_Adress.py` on the hub machine  
2. Start the hub (`hub.py` or Docker container)  
3. Start each robot with:
   - `--robot <robot_config>`
   - optional `--exp <experiment_config>`
   - optional `--role supervisor`

Each robot will:

1. load its robot configuration  
2. load the experiment configuration (if provided)  
3. start the listener in the background  
4. broadcast `hello`  
5. exchange skills automatically  

---

## ✅ Late Joining Robots

If a robot starts later:

- it broadcasts `hello`
- existing robots reply directly with their skills
- the new robot updates its partner table

No extra setup is required.