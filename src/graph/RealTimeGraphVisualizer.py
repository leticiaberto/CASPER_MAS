import matplotlib.pyplot as plt
import networkx as nx
import threading
from src.graph.TaskAssignment import TaskStatus
import math
import warnings
warnings.filterwarnings("ignore", message="Starting a Matplotlib GUI outside of the main thread")

class RealTimeGraphVisualizer:
    def __init__(self, G, vis_queue):
        self.G = G
        self.queue = vis_queue

        # Compute layout once
        self.pos = nx.nx_agraph.graphviz_layout(self.G, prog="dot", args="-Grankdir=TB")
        
        self.running = True
        
    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()

        manager = plt.get_current_fig_manager()
        try:
            # Qt backend
            manager.window.showMaximized()
        except Exception:
            try:
                # Tk backend
                manager.resize(
                    manager.window.winfo_screenwidth(),
                    manager.window.winfo_screenheight()
                )
                manager.window.state('zoomed')
            except Exception:
                pass

        while self.running:
            self._process_events()
            self._draw()
            plt.pause(0.1)

    def _process_events(self):
        # just drain queue; attributes already updated
        while not self.queue.empty():
            self.queue.get()

    def duration_to_size(self, duration, base=4000, scale=2500):
        """
        duration: numeric (seconds, steps, etc.)
        base: minimum node size
        scale: visual growth factor
        """
        if duration is None:
            return base

        duration = float(duration)
        factor = math.log1p(duration)   # smooth growth
        return base + factor * scale

    def _draw(self):
        self.ax.clear()

        node_colors = [
            TaskStatus._status_color(self.G.nodes[n]["assignment"].status)
            for n in self.G.nodes
        ]

        labels = {}
        for n in self.G.nodes:
            node_data = self.G.nodes[n]
            assignment = node_data["assignment"]
            workspace = node_data.get("required_constraints", {}).get("workspace")

            if isinstance(workspace, list):
                ws_text = ", ".join(str(w) for w in workspace)
            elif workspace:
                ws_text = str(workspace)
            else:
                ws_text = "—"

            labels[n] = (
                f"{assignment.task_id}\n"
                f"{node_data['duration']:.1f}s\n"
                f"WS: {ws_text}\n"
                f"{assignment.status.value}\n"
                f"{assignment.selected_agent}"
            )

        node_sizes = []
        for n in self.G.nodes:
            duration = self.G.nodes[n]["duration"]

            node_sizes.append(
                self.duration_to_size(duration))

        nx.draw(
            self.G,
            pos=self.pos,
            ax=self.ax,
            node_color=node_colors,
            with_labels=False,
            node_size=node_sizes
        )

        nx.draw_networkx_labels(
            self.G,
            pos=self.pos,
            labels=labels,
            ax=self.ax,
            font_size=12
        )

        self.ax.set_title("Global Goal Status (Real-Time)")