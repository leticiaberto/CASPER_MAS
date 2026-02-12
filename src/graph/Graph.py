import json
from matplotlib.style import context
import networkx as nx
from networkx.drawing.nx_pydot import to_pydot
import math
from utils import PastelPalette, DistinctPalette

class Graph:
    def __init__(self):
        self.G = nx.DiGraph()
    # -----------------------------
    # Safe node ID
    # -----------------------------
    def clean_id(self, name):
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

    # -----------------------------
    # Load graph from JSON
    # -----------------------------
    def load_task_graph(self, filename):
        with open(filename, "r") as f:
            data = json.load(f)

        for node in data["tasks"]:
            raw_id = node["id"]
            task_id = self.clean_id(raw_id)

            # Copy all attributes except id
            attrs = {k: v for k, v in node.items() if k != "id"}
            attrs["original_name"] = raw_id

            self.G.add_node(task_id, **attrs)

        # Dependencies instead of edges
        for raw_source, raw_target in data.get("dependencies", []):
            source = self.clean_id(raw_source)
            target = self.clean_id(raw_target)
            self.G.add_edge(source, target)


class GraphVisualizer:
    def graphviz_safe_graph(self, G):
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

    # -----------------------------
    # Duration to node size (log scale)
    # -----------------------------
    def duration_to_size(self, duration, mode="duration", fixed_size=(2.0, 1.2), base=1.2, scale=0.8):
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

    def _build_node_label(self, name, duration, status_text, context, selected_agent, selected_score, top_candidates):
        selected_text = (
            f"{selected_agent} ({selected_score:.2f})"
            if selected_agent else "None"
        )

        if top_candidates:
            top_text = "\\n".join(agent for agent in top_candidates)
        else:
            top_text = "None"

        return (
            f"{name}\\n"
            f"{duration}s\\n"
            f"Ctx {context}\\n"
            f"Status: {status_text}\\n"
            f"Selected: {selected_text}\\n"
            f"Top candidates:\\n"
            f"{top_text}"
        )

    def _render_node(self, dot_node, palette, name, duration, status_text, context, selected_agent, selected_score, top_candidates, fillType):

        label = self._build_node_label(name, duration, status_text, context, selected_agent, selected_score, top_candidates )

        if fillType == "context":
            fillcolor = palette.get_color(context)
        else:
            key = selected_agent if selected_agent else None
            fillcolor = palette.get_color(key)

        width, height = self.duration_to_size(duration, mode="duration")

        dot_node.set_label(label)
        dot_node.set_shape("box")
        dot_node.set_style("rounded")
        dot_node.set_style("filled")
        dot_node.set_fillcolor(fillcolor)
        dot_node.set_fixedsize("true")
        dot_node.set_width(str(width))
        dot_node.set_height(str(height))
        dot_node.set_fontsize("10")

    # -----------------------------
    # Multi-agent capability graph
    # -----------------------------
    def export_multiagent_graph(self, G, output_name="data/task_graph_multiagent", palette_mode="distinct", fillType="agent"):
        G_viz = self.graphviz_safe_graph(G)
        dot = to_pydot(G_viz)
        dot.set_rankdir("TB")
        dot.set_splines("ortho")

        contexts = [G.nodes[n].get("context", "none") for n in G.nodes]
        palette = PastelPalette() if palette_mode == "pastel" else DistinctPalette(contexts)

        for node in dot.get_nodes():
            node_id = node.get_name().strip('"')

            if node_id not in G.nodes:
                continue

            task = G.nodes[node_id]

            name = task.get("original_name", node_id)
            context = task.get("context", "none")
            duration = float(task.get("duration", 1))
    
            assignment = task.get("assignment")
            if assignment:
                selected_agent = assignment.selected_agent
                selected_score = assignment.selected_score
                top_candidates = assignment.top_candidates
                status_text = assignment.status.value.upper()
            else:
                selected_agent = None
                selected_score = None
                top_candidates = []
                status_text = "UNASSIGNED"

            

            # --- Rendering ---
            self._render_node(node, palette, name, duration, status_text, context, selected_agent, selected_score, top_candidates, fillType)

        dot.write_png(f"{output_name}.png")
        dot.write_pdf(f"{output_name}.pdf")
        print(f"Exported multi-agent graph: {output_name}.png / .pdf")