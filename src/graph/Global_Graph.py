import json
from matplotlib.style import context
import networkx as nx
from src.graph.TaskAssignment import TaskStatus, TaskOutcome

class GlobalGraph:
    def __init__(self):
        self.G = nx.DiGraph()
        self._outcome_listeners = []
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
    
        assignment = self.G.nodes[task_id].get("assignment")
        if assignment is None:
            current = TaskStatus.NOT_ASSIGNED
        else:
            current = assignment.status

            if current == status:
                return  # idempotent — already in this state, nothing to do

            # The global graph is a passive mirror of what agents report over
            # the network.  Agents may skip intermediate states (e.g. a
            # time_to_clean task goes ASSIGNED → READY without broadcasting
            # PENDING), so we only log a warning instead of raising here.
            if not TaskStatus.is_valid_transition(current, status):
                print(
                    f"[GlobalGraph] Warning: non-standard transition "
                    f"{current.name} → {status.name} for task '{task_id}'. "
                    f"Accepting remote update."
                )

            self.G.nodes[task_id]["assignment"].status = status

            #print("UPDATED: ", self.G.nodes[task_id]["assignment"].status)
    
    def on_outcome(self, callback):
        """Register callback(task_id, agent, previous_outcome, new_outcome)."""
        self._outcome_listeners.append(callback)

    def update_outcome(self, task_id, outcome):
        if task_id not in self.G:
            raise ValueError(f"Task {task_id} not found.")
        if not isinstance(outcome, TaskOutcome):
            raise TypeError("outcome must be a TaskOutcome.")

        assignment = self.G.nodes[task_id].get("assignment")
        if assignment is None:
            raise ValueError(f"Task {task_id} has no assignment.")
        if assignment.status != TaskStatus.DONE:
            raise ValueError(
                f"Task {task_id} is {assignment.status.name}, "
                f"outcome can only be set once DONE."
            )

        previous = assignment.set_outcome(outcome)# also appends to outcome_history
        for cb in self._outcome_listeners:
            cb(task_id, assignment.selected_agent, previous, outcome)
        
        return previous