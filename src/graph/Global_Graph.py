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

    def update_status(self, task_id, status):
        """
        Update the status of a task in the global graph.
        """
        if task_id not in self.G:
            raise ValueError(f"Task {task_id} not found.")

        if not isinstance(status, TaskStatus):
            raise TypeError("status must be TaskStatus Enum.")
    
        current = self.G.nodes[task_id].get("status", TaskStatus.NOTASSIGNED)

        allowed_transitions = {
            TaskStatus.NOTASSIGNED: {TaskStatus.ASSIGNED},
            TaskStatus.PENDING: {TaskStatus.ASSIGNED},
            TaskStatus.ASSIGNED: {TaskStatus.READY},
            TaskStatus.READY: {TaskStatus.RUNNING},
            TaskStatus.RUNNING: {TaskStatus.DONE},
            TaskStatus.DONE: set()
        }

        if status not in allowed_transitions.get(current, set()):
            raise ValueError(
                f"Illegal transition {current.name} → {status.name}"
            )

        self.G.nodes[task_id]["status"] = status