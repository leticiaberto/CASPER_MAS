import json
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

def agents_can_execute(G, agents_skills):
    """
    Determine which agents can perform each task.
    
    G: networkx DiGraph
        Each node has 'requires' (comma-separated skills) and 'context'
    agents_skills: dict
        {agent_name: {'skills': {'manipulation': 5, ...}, 'contexts': {'assembly', ...}}}

    Returns:
        capable_agents: dict {node_id: [agent_name, ...]}
    """

    capable_agents = {}

    for node_id in G.nodes:
        task = G.nodes[node_id]
        reqs = [r.strip() for r in task.get("requires", "").split(",") if r.strip()]
        context = task.get("context", None)

        capable_list = []
        for agent_name, info in agents_skills.items():
            skills = info.get("skills", {})
            contexts = info.get("contexts", set())

            # Check if agent has all required skills and context matches
            if context not in contexts:
                continue
            if all(sk in skills and skills[sk] > 0 for sk in reqs):
                capable_list.append(agent_name)

        capable_agents[node_id] = capable_list

    return capable_agents


# -----------------------------
# Palette classes
# -----------------------------
class PastelPalette__:
    """Soft pastel colors, deterministic by label"""
    def __init__(self, saturation=0.35, value=0.95):
        self.sat = saturation
        self.val = value
        self.map = {}

    def get_color(self, label):
        if label is None:
            return "white"
        if label in self.map:
            return self.map[label]

        # stable hash based on md5
        h = int(hashlib.md5(label.encode("utf-8")).hexdigest()[:8], 16)
        hue = (h % 360) / 360.0

        r, g, b = colorsys.hsv_to_rgb(hue, self.sat, self.val)
        color_hex = "#%02x%02x%02x" % (int(r*255), int(g*255), int(b*255))
        self.map[label] = color_hex
        return color_hex

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
def duration_to_size(duration, base=1.2, scale=0.8):
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
    for node in data["nodes"]:
        raw_id = node["id"]
        task_id = clean_id(raw_id)
        attrs = {k: v for k, v in node.items() if k != "id"}
        attrs["original_name"] = raw_id
        # convert requires list to string
        reqs = attrs.get("requires", [])
        if isinstance(reqs, list):
            attrs["requires"] = ", ".join(reqs)
        else:
            attrs["requires"] = str(reqs)
        G.add_node(task_id, **attrs)

    for raw_source, raw_target in data["edges"]:
        source = clean_id(raw_source)
        target = clean_id(raw_target)
        G.add_edge(source, target)

    return G

# -----------------------------
# Multi-agent capability graph
# -----------------------------
def export_multiagent_graph(G, capable_agents,
                            output_name="task_graph_multiagent",
                            palette_mode="distinct"):

    dot = to_pydot(G)
    dot.set_rankdir("TB")
    dot.set_splines("ortho")

    contexts = [G.nodes[n].get("context", "none") for n in G.nodes]
    context_palette = PastelPalette() if palette_mode=="pastel" else DistinctPalette(contexts)

    for node in dot.get_nodes():
        node_id = node.get_name().strip('"')
        if node_id not in G.nodes:
            continue

        task = G.nodes[node_id]
        name = task.get("original_name", node_id)
        duration = float(task.get("duration", 1))
        context = task.get("context", "none")
        agents = capable_agents.get(node_id, [])
        agents_text = ", ".join(agents) if agents else "None"

        fillcolor = context_palette.get_color(context)
        width, height = duration_to_size(duration)

        label = f"{name}\n{duration}s\nCtx: {context}\nCapable: {agents_text}"

        node.set_label(label)
        node.set_shape("box")
        node.set_style("rounded,filled")
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
                               output_name="task_graph_assigned",
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
        width, height = duration_to_size(duration)
        label = f"{name}\n{duration}s\nAgent: {assigned if assigned else 'None'}"

        node.set_label(label)
        node.set_shape("box")
        node.set_style("rounded,filled")
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

    # Example agent definitions
    agents_skills = {
        "RobotArm": {"skills": {"manipulation": 5, "vision": 2}, "contexts": {"assembly"}},
        "MobileRobot": {"skills": {"mobility": 5}, "contexts": {"navigation"}},
        "Drone": {"skills": {"vision": 5}, "contexts": {"quality_check"}},
    }

    # Determine capable agents per task
    capable_agents = agents_can_execute(G, agents_skills)

    # Export graphs
    export_multiagent_graph(G, capable_agents, palette_mode="pastel")
    export_assigned_agent_graph(G, palette_mode="pastel")
