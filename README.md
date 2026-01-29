# CASPER_MAS

# Multi-Robot Skill Exchange System (ZeroMQ)

This project provides a simple **multi-robot communication framework** in Python where robots can:

* run on the same machine or across different computers
* communicate in real time
* exchange skills and preferences at startup
* support late-joining robots

Robots broadcast their presence and automatically share their skill tables.

---

## ✅ Features

* Background listener thread (robots can do other tasks while listening)
* Robots can join late and still receive partner skills
* Skill exchange uses a clean protocol:

| Message Type                | Purpose                    |
| --------------------------- | -------------------------- |
| `hello`                     | announce robot presence    |
| `skills_update(target=...)` | directed reply with skills |
| `skills_request`            | optional recovery/refresh  |

Directed replies prevent unnecessary updates in other robots.

---

## 📦 Requirements

Install dependencies:

```bash
pip install pyzmq
```

---

## 🖥️ Running on One Machine

### 1. Start the Hub

The hub relays messages between robots:

```bash
python hub.py
```

### 2. Start Robots

Run each robot in its own terminal:

```bash
python robot.py --config configs/panda_arm_1.yaml
python robot.py --config configs/pepper_mobile_1.yaml
```

Robots will automatically discover each other and exchange skills.

---

## 🌍 Running Across Multiple Computers

### 1. Choose One Machine as the Hub

Example hub IP:

```
192.168.1.50
```

Run on that machine:

```bash
python hub.py
```

### 2. Open Firewall Ports

Hub uses:

* TCP 5555 (robots publish)
* TCP 5556 (robots subscribe)

Linux UFW example:

```bash
sudo ufw allow 5555/tcp
sudo ufw allow 5556/tcp
```

### 3. Connect Robots to the Hub IP

On every robot machine, set:

```python
hub_ip = "192.168.1.50"
```

Now robots will communicate across computers.

---

## ⚙️ Robot Skills Configuration

Each robot must define its skills and preferences in the **configs/** folder.

Example file:

```
configs/robot_1.yaml
```

Example structure:

```yaml
contexts:
  - assembly
  - party

skills:
  assembly:
    manipulation: [0.9, 0.8]
    navigation:   [0.4, 0.2]

  party:
    navigation:   [0.7, 0.6]
```

Where each entry is:

```
skill: [skill_level, skill_preference]
```

* skill_level ∈ [0.0 – 1.0]
* preference ∈ [0.0 – 1.0]

Robots automatically export this structure and share it at startup.

---

## ▶️ Running the System

Once the hub is running and robot configs exist, the system is started simply by running:

```bash
python robot.py
```

Each robot will:

1. load its skills from `configs/`
2. start the listener in the background
3. broadcast `hello`
4. exchange skills automatically

---

## ✅ Late Joining Robots

If Robot 2 starts later:

* it broadcasts `hello`
* existing robots reply directly with their skills
* Robot 2 updates its partner table

No extra setup is required.
