import networkx as nx
from src.graph.TaskAssignment import TaskStatus

class LocalGraph:
    def __init__(self, agent_id, global_graph, external_done):
        self.agent_id = agent_id
        self.global_graph = global_graph
        self.external_done = external_done

        self.graph = nx.DiGraph()
        self.build_local_graph()

    # -------------------------
    # Build local execution graph
    # -------------------------
    def build_local_graph(self):
        self.graph.clear()

        # Add my tasks
        for node, data in self.global_graph.nodes(data=True):
            if data.get("assignment").selected_agent == self.agent_id:
                self.graph.add_node(node, status=TaskStatus.PENDING)

        # Add internal edges
        for u, v in self.global_graph.edges():
            if u in self.graph and v in self.graph:
                self.graph.add_edge(u, v)

    # -------------------------
    # Rebuild local graph (after reassignment/global update)
    # -------------------------
    def rebuild(self, new_global_graph):
        self.global_graph = new_global_graph
        self.build_local_graph()
        self.update_ready()   # recompute readiness after rebuild

    # -------------------------
    # Check if task is ready
    # -------------------------
    def is_ready(self, task):
        if self.graph.nodes[task]["status"] != TaskStatus.PENDING:
            return False

        # Check all predecessors in the global graph
        for pred in self.global_graph.predecessors(task):
            assigned = self.global_graph.nodes[pred]["assigned_agent"]

            # Local dependency
            if assigned == self.agent_id:
                if self.graph.nodes[pred]["status"] != TaskStatus.DONE:
                    return False
            # External dependency
            else:
                if pred not in self.external_done:
                    return False

        return True
    
    # -------------------------
    # Full recomputation
    # -------------------------
    def update_ready(self):
        for task in self.graph.nodes:
            if self.is_ready(task):
                self.graph.nodes[task]["status"] = TaskStatus.READY

    # -------------------------
    # Incremental update from external completion
    # -------------------------
    def handle_external_completion(self, external_task_id):
        # Only update tasks that depend on this external task
        for task in self.graph.nodes:
            if external_task_id in self.global_graph.predecessors(task):
                if self.is_ready(task):
                    self.graph.nodes[task]["status"] = TaskStatus.READY

    # -------------------------
    # Get tasks ready for execution
    # -------------------------
    def get_ready_tasks(self):
        self.update_ready()# Make sure readiness is up-to-date

        return [
            t for t in self.graph.nodes
            if self.graph.nodes[t]["status"] == TaskStatus.READY
        ]

    # -------------------------
    # State transitions for local execution
    # -------------------------
    def mark_running(self, task):
        self.graph.nodes[task]["status"] = TaskStatus.RUNNING

    def mark_done(self, task):
        self.graph.nodes[task]["status"] = TaskStatus.DONE

    # -------------------------
    # Check if any local tasks depend on a given external task
    # -------------------------
    def depends_on_external(self, external_task_id):
        for task_id in self.graph.nodes:
            if external_task_id in self.global_graph.predecessors(task_id):
                return True
        return False

    