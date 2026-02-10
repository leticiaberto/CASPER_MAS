import json
import textwrap
import networkx as nx
import pydot
from networkx.drawing.nx_pydot import to_pydot
import colorsys
import math
import hashlib

# -----------------------------
# Safe node ID
# -----------------------------
def clean_id(name):
    return (
        str(name)
        .replace(" ", "_")
        .replace("[", "")
        .replace("]", "")
        .replace("'", "")
        .replace('"', "")
        .replace("(", "")
        .replace(")", "")
        .replace(",", "_")
    )

def score_agents_for_task(G, agents, mode="combined"):
    """
    Returns a dict {task_id: [(agent_id, score), ...]} sorted by score descending.
    
    Parameters:
    - G: networkx DiGraph with tasks as nodes.
    - agents: list of Agent objects.
    - mode: str, one of ["skill", "preference", "combined"]
        "skill"      -> score = sum of skill levels only
        "preference" -> score = sum of preferences only
        "combined"   -> score = sum(level * (1 + pref))
    """

    assert mode in {"skill", "preference", "combined"}, f"Invalid mode: {mode}"

    task_rankings = {}

    for node_id in G.nodes:
        task = G.nodes[node_id]

        context = task.get("context")
        required_skills = task.get("required_skills", {})
        required_constraints = task.get("required_constraints", {})

        scored_agents = []

        for agent in agents:
            # --- 1. Context check ---
            agent_contexts = set()
            for s in agent.skills.skill_level:
                agent_contexts.update(agent.skills.skill_level[s].keys())

            if context not in agent_contexts:
                continue

            # --- 2. Skill & preference scoring ---
            skill_ok = True
            score = 0.0

            for skill, min_level in required_skills.items():
                level = agent.skills.skill_level.get(skill, {}).get(context, 0.0)
                pref = agent.skills.skill_preference.get(skill, {}).get(context, 0.0)

                if level < min_level:
                    skill_ok = False
                    break

                if mode == "skill":
                    score += level
                elif mode == "preference":
                    score += pref
                elif mode == "combined":
                    score += level * (1 + pref)

            if not skill_ok:
                continue

            # ---- 3. Constraint check (lenient) ----
            constraint_ok = True
            for key, value in required_constraints.items():
                # Constraints become blocking only when explicitly incompatible. Tasks can introduce new constraints without breaking old agents. unknown ≠ forbidden
                if key not in agent.constraints:
                    continue  # assume agent satisfies it. 

                agent_value = agent.constraints[key]

                if isinstance(value, (int, float)):
                    if agent_value < value:
                        constraint_ok = False
                        break
                else:
                    if agent_value != value:
                        constraint_ok = False
                        break

            if not constraint_ok:
                continue

            scored_agents.append((agent.id, score))

        # --- 4. Sort descending ---
        scored_agents.sort(key=lambda x: x[1], reverse=True)
        task_rankings[node_id] = scored_agents

    return task_rankings


# -----------------------------
# Palette classes
# -----------------------------
class PastelPalette:
    """
    Deterministic pastel palette with better hue separation
    """

    def __init__(self, saturation=0.55, value=0.95):
        self.sat = saturation
        self.val = value
        self.map = {}

    def get_color(self, label):
        if label is None:
            return "white"
        if label in self.map:
            return self.map[label]

        # Stable hash → convert to 0..1
        h_bytes = hashlib.md5(label.encode("utf-8")).digest()
        hue = int.from_bytes(h_bytes[:2], "little") / 65535.0  # 0..1

        r, g, b = colorsys.hsv_to_rgb(hue, self.sat, self.val)
        color_hex = "#%02x%02x%02x" % (int(r*255), int(g*255), int(b*255))
        self.map[label] = color_hex
        return color_hex
    
class DistinctPalette:
    """Evenly spaced colors for deterministic distinct palette"""
    def __init__(self, labels, saturation=0.8, value=0.95):
        self.labels = sorted(set(labels))  # deterministic order
        self.map = {}
        n = len(self.labels)
        for i, label in enumerate(self.labels):
            hue = i / max(n, 1)
            r, g, b = colorsys.hsv_to_rgb(hue, saturation, value)
            color_hex = "#%02x%02x%02x" % (int(r*255), int(g*255), int(b*255))
            self.map[label] = color_hex

    def get_color(self, label):
        return self.map.get(label, "white")

# -----------------------------
# Duration to node size (log scale)
# -----------------------------
def duration_to_size(duration, mode="duration", fixed_size=(2.0, 1.2), base=1.2, scale=0.8):
    """
    Returns (width, height) for a node.

    mode:
      - "duration": size scales with duration
      - "fixed": always returns fixed_size
    """

    if mode == "fixed":
        return fixed_size

    # mode == "duration"
    duration = float(duration)
    factor = math.log1p(duration)

    width = base + factor * scale
    height = 0.7 + factor * (scale * 0.5)

    return width, height

# -----------------------------
# Load graph from JSON
# -----------------------------
def load_task_graph(filename):
    with open(filename, "r") as f:
        data = json.load(f)

    G = nx.DiGraph()

    for node in data["tasks"]:
        raw_id = node["id"]
        task_id = clean_id(raw_id)

        # Copy all attributes except id
        attrs = {k: v for k, v in node.items() if k != "id"}
        attrs["original_name"] = raw_id

        G.add_node(task_id, **attrs)

    # Dependencies instead of edges
    for raw_source, raw_target in data.get("dependencies", []):
        source = clean_id(raw_source)
        target = clean_id(raw_target)
        G.add_edge(source, target)

    return G

def graphviz_safe_graph(G):
    """
    Returns a copy of G with only Graphviz-safe node attributes.
    Removes dicts/lists and converts everything to strings.
    """
    G_safe = G.copy()

    for n, attrs in G_safe.nodes(data=True):
        safe_attrs = {}
        for k, v in attrs.items():
            # Drop complex objects completely
            if isinstance(v, (dict, list, set, tuple)):
                continue
            # Force everything else to string
            safe_attrs[k] = str(v)

        G_safe.nodes[n].clear()
        G_safe.nodes[n].update(safe_attrs)

    return G_safe

def get_top_candidates(scored_agents, selected_agent_id, top_k=3):
    """
    Returns the top-k candidates excluding the selected agent.

    Parameters:
    - scored_agents: list of tuples [(agent_id, score), ...] sorted descending
    - selected_agent_id: the agent ID to exclude
    - top_k: maximum number of agents to return

    Returns:
    - list of strings: ["AgentA (0.95)", "AgentB (0.80)", ...]
    """
    top_candidates = []
    for aid, score in scored_agents:
        if aid != selected_agent_id:
            top_candidates.append(f"{aid} ({score:.2f})")
        if len(top_candidates) >= top_k:
            break
    return top_candidates

# -----------------------------
# Multi-agent capability graph
# -----------------------------
def export_multiagent_graph(
    G, capable_agents,
    output_name="data/task_graph_multiagent",
    palette_mode="distinct",
    top_k=3
):
    G_viz = graphviz_safe_graph(G)
    dot = to_pydot(G_viz)
    dot.set_rankdir("TB")
    dot.set_splines("ortho")

    contexts = [G.nodes[n].get("context", "none") for n in G.nodes]
    context_palette = PastelPalette() if palette_mode == "pastel" else DistinctPalette(contexts)

    for node in dot.get_nodes():
        node_id = node.get_name().strip('"')
        if node_id not in G.nodes:
            continue

        task = G.nodes[node_id]
        name = task.get("original_name", node_id)
        context = task.get("context", "none")
        duration = float(task.get("duration", 1))

        scored = capable_agents.get(node_id, [])
        print(scored)
        if scored:
            # Determine selected agent (can be updated externally)
            selected_agent = getattr(task, "selected_agent", "default2")#scored[0][0] if scored else "None"
            selected_agent_score = scored[0][1]
            # Use helper to get top candidates excluding selected
            top_candidates = get_top_candidates(scored, selected_agent, top_k)
            top_candidates_text = "\\n".join(top_candidates) if top_candidates else "None"

        else:
            selected_agent = "None"
            selected_agent_score = ""
            top_candidates_text = "None"

        fillcolor = context_palette.get_color(context)
        width, height = duration_to_size(duration, mode="duration")

        label = (
            f"{name}\\n"
            f"{duration}s\\n"
            f"Ctx {context}\\n"
            f"Selected: {selected_agent} ({selected_agent_score})\\n"
            f"Top candidates:\\n"
            f"{top_candidates_text}"
        )

        node.set_label(label)
        node.set_shape("box")
        node.set_style("rounded")
        node.set_style("filled")
        node.set_fillcolor(fillcolor)
        node.set_fixedsize("true")
        node.set_width(str(width))
        node.set_height(str(height))
        node.set_fontsize("10")


    dot.write_png(f"{output_name}.png")
    dot.write_pdf(f"{output_name}.pdf")
    print(f"Exported multi-agent graph: {output_name}.png / .pdf")



# -----------------------------
# Assigned-agent graph
# -----------------------------
def export_assigned_agent_graph(G,
                               output_name="../../data/task_graph_assigned",
                               palette_mode="distinct"):

    dot = to_pydot(G)
    dot.set_rankdir("TB")
    dot.set_splines("ortho")

    agents = [G.nodes[n].get("assigned_agent")
              for n in G.nodes if G.nodes[n].get("assigned_agent")]

    agent_palette = PastelPalette() if palette_mode=="pastel" else DistinctPalette(agents)

    for node in dot.get_nodes():
        node_id = node.get_name().strip('"')
        if node_id not in G.nodes:
            continue

        task = G.nodes[node_id]
        name = task.get("original_name", node_id)
        duration = float(task.get("duration", 1))
        assigned = task.get("assigned_agent", None)
        fillcolor = agent_palette.get_color(assigned) if assigned else "white"
        width, height = duration_to_size(duration, mode="duration")
        label = f"{name}\n{duration}s\nAgent: {assigned if assigned else 'None'}"

        node.set_label(label)
        node.set_shape("box")
        node.set_style("rounded")
        node.set_style("filled")
        node.set_fillcolor(fillcolor)
        node.set_fixedsize("true")
        node.set_width(str(width))
        node.set_height(str(height))
        node.set_fontsize("10")

    dot.write_png(f"{output_name}.png")
    dot.write_pdf(f"{output_name}.pdf")
    print(f"Exported assigned-agent graph: {output_name}.png / .pdf")

if __name__ == "__main__":
    # Load task graph
    G = load_task_graph("task_graph.json")

    # Determine capable agents per task
    #capable_agents = agents_can_execute(G, agents_skills)

    # Skill-only
    rankings_skill = score_agents_for_task(G, agents, mode="skill")
    print("\n--- Agent rankings per task (skill only) ---")
    for task, agent_rankings in rankings_skill.items():
        print(f"Task: {task}")
        for agent, score in agent_rankings:
            print(f"  Agent: {agent}, Score: {score}")
    # Export graphs
    #export_multiagent_graph(G, capable_agents, palette_mode="pastel")
    #export_assigned_agent_graph(G, palette_mode="pastel")
