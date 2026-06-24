import time

from networkx.drawing.nx_pydot import to_pydot
import math
from utils import PastelPalette, DistinctPalette
from src.graph.TaskAssignment import TaskStatus
import networkx as nx
from graphviz import Digraph

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

    def _build_node_label_global(self, name, duration, status_text, context, workspace, selected_agent, selected_score, top_candidates):
        selected_text = (
            f"{selected_agent} ({selected_score:.2f})"
            if selected_agent else "None"
        )

        if top_candidates:
            top_text = "\\n".join(agent for agent in top_candidates)
        else:
            top_text = "None"

        # Normalise workspace to a compact string
        if isinstance(workspace, list):
            ws_text = ", ".join(str(w) for w in workspace)
        elif workspace:
            ws_text = str(workspace)
        else:
            ws_text = "—"

        return (
            f"{name}\\n"
            f"Duration: {duration}s\\n"
            f"Context: {context}\\n"
            f"Workspace: {ws_text}\\n"
            f"Status: {status_text}\\n"
            f"Selected: {selected_text}\\n"
            f"Top candidates:\\n"
            f"{top_text}"
        )

    def _build_node_label_local(self, name, duration, status_text, workspace, selected_agent):
        if isinstance(workspace, list):
            ws_text = ", ".join(str(w) for w in workspace)
        elif workspace:
            ws_text = str(workspace)
        else:
            ws_text = "—"

        return (
            f"{name}\\n"
            f"{duration}s\\n"
            f"{status_text}\\n"
            f"WS: {ws_text}\\n"
            f"{selected_agent}"
        )
    
    def _render_node(self, dot_node, palette, name, duration, status_text, context, workspace, selected_agent, selected_score, top_candidates, fillType, graphType="global"):
        if graphType == "global":
            label = self._build_node_label_global(name, duration, status_text, context, workspace, selected_agent, selected_score, top_candidates)
            if fillType == "context":
                fillcolor = palette.get_color(context)
            else:
                key = selected_agent if selected_agent else None
                fillcolor = palette.get_color(key)
        else:
            label = self._build_node_label_local(name, duration, status_text, workspace, selected_agent)
            fillcolor = TaskStatus._status_color(TaskStatus(status_text.lower()))  # Use status text to determine color


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
    def export_multiagent_graph(self, G, output_name="task_graph_multiagent", palette_mode="distinct", fillType="agent", graphType="global"):
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
            workspace = task.get("required_constraints", {}).get("workspace")
    
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
                status_text = TaskStatus.NOT_ASSIGNED

            # --- Rendering ---
            self._render_node(node, palette, name, duration, status_text, context, workspace, selected_agent, selected_score, top_candidates, fillType, graphType)

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        dot.write_png(f"data/{output_name}{timestamp}.png")
        dot.write_pdf(f"data/{output_name}{timestamp}.pdf")
        print(f"Exported multi-agent graph: {output_name}.png / .pdf")


#######################################################################
    def build_plot_graph(self, graph):
        """
        Build a visualization-only graph from the local task graph.
        Workspace is already stored on local nodes by LocalGraph.build_local_graph.
        """
        Gp = nx.DiGraph()

        # --- Add local nodes ---
        for task_id, data in graph.nodes(data=True):
            Gp.add_node(
                task_id,
                node_type="local",
                status=data["status"],
            )

        # --- Add local edges (solid) ---
        for u, v in graph.edges():
            Gp.add_edge(u, v, edge_type="local")

        # --- External predecessors (dashed incoming) ---
        for task_id, data in graph.nodes(data=True):
            for ext in data["external_predecessors"]:
                if not Gp.has_node(ext):
                    Gp.add_node(ext, node_type="external")
                Gp.add_edge(ext, task_id, edge_type="external")

        # --- External successors (dashed outgoing) ---
        for task_id, data in graph.nodes(data=True):
            for ext in data["external_successors"]:
                if not Gp.has_node(ext):
                    Gp.add_node(ext, node_type="external")
                Gp.add_edge(task_id, ext, edge_type="external")

        return Gp

    def plot_task_graph(self, graph, filename="task_graph"):
        """
        Render the local task graph with:
        - solid rectangles for local tasks (with workspace shown)
        - dashed rectangles for external tasks
        - solid edges for local deps
        - dashed edges for external deps
        Outputs both PDF and PNG.
        Workspace is read directly from local graph nodes (set during LocalGraph.build_local_graph).
        """
        Gp = self.build_plot_graph(graph)

        dot = Digraph(comment="Local Task Graph")
        dot.attr(rankdir="TB", splines="ortho")

        # --- Nodes ---
        for node, data in Gp.nodes(data=True):
            if data["node_type"] == "local":
                # workspace is stored directly on local graph nodes by LocalGraph
                workspace = graph.nodes[node].get("workspace")
                if isinstance(workspace, list):
                    ws_text = ", ".join(str(w) for w in workspace)
                elif workspace:
                    ws_text = str(workspace)
                else:
                    ws_text = "—"

                label = f"{node}\\n{data['status'].value}\\nWS: {ws_text}"
                dot.node(
                    str(node),
                    label=label,
                    shape="box",
                    style="filled",
                    fillcolor=TaskStatus._status_color(data["status"]),
                )
            else:
                # External predecessor / successor
                dot.node(
                    str(node),
                    label=str(node),
                    shape="box",
                    style="dashed,filled",
                    fillcolor="lightgrey",
                )

        # --- Edges ---
        for u, v, data in Gp.edges(data=True):
            if data["edge_type"] == "local":
                dot.edge(str(u), str(v), style="solid")
            else:
                dot.edge(str(u), str(v), style="dashed")

        # --- Save outputs ---
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        base = "data/" + filename + timestamp

        dot.format = "png"
        dot.render(base, cleanup=False)# Keep source

        time.sleep(1) # Ensure file is written before next render

        dot.format = "pdf"
        dot.render(base, cleanup=True)# Delete source after last render