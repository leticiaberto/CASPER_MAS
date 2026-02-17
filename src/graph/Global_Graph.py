import json
from matplotlib.style import context
import networkx as nx
from src.graph.TaskAssignment import TaskStatus

class GlobalGraph:
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

    def is_task_ready(self, task_id):
        predecessors = self.graph.predecessors(task_id)
        return all(self.graph.nodes[p]["assignment"].status == TaskStatus.DONE
                for p in predecessors)
    
    def notify_successors(self, task_id):
        for succ in self.graph.successors(task_id):
            if self.is_task_ready(succ):
                self.graph.nodes[succ]["assignment"].status = TaskStatus.READY