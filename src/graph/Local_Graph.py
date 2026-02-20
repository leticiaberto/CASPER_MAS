import networkx as nx
from src.graph.TaskAssignment import TaskStatus

class LocalGraph:
    def __init__(self, agent_id, global_graph):
        self.agent_id = agent_id
        self.global_graph = global_graph
        self.external_done = set()
        self.task_to_agent = {}

        self.graph = nx.DiGraph()
        self.build_local_graph()

    # -------------------------
    # Build local execution graph
    # -------------------------
    def build_local_graph(self):
        self.graph.clear()

        # Record assignment for ALL tasks we see
        for node, data in self.global_graph.nodes(data=True):
            assignment = data.get("assignment")
            if assignment:
                self.task_to_agent[node] = assignment.selected_agent

        # --- Add my tasks ---
        for node, agent in self.task_to_agent.items():
            if agent == self.agent_id:
                self.graph.add_node(
                    node,
                    status=TaskStatus.PENDING,
                    local_predecessors=set(),
                    external_predecessors=set(),
                    local_successors=set(),
                    external_successors=set(),
                )

        # --- Classify dependencies ---
        for u, v in self.global_graph.edges():
            u_local = u in self.graph
            v_local = v in self.graph

            # Local → Local
            if u_local and v_local:
                self.graph.add_edge(u, v)
                self.graph.nodes[v]["local_predecessors"].add(u)
                self.graph.nodes[u]["local_successors"].add(v)

            # External → Local
            elif not u_local and v_local:
                self.graph.nodes[v]["external_predecessors"].add(u)

            # Local → External
            elif u_local and not v_local:
                self.graph.nodes[u]["external_successors"].add(v)

        # Sanity check: guarantees no “ghost” dependencies
        assert all(
            pred in self.graph.nodes or pred in self.external_done or True
            for _, data in self.graph.nodes(data=True)
            for pred in data["external_predecessors"]
        )

    # -------------------------
    # Rebuild local graph (after reassignment/global update)
    # -------------------------
    def rebuild(self, new_global_graph):
        self.global_graph = new_global_graph
        self.build_local_graph()
        self.update_ready()   # recompute readiness after rebuild

    # -------------------------
    # Full recomputation
    # -------------------------
    def update_ready(self):
        for task in self.graph.nodes:
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
    # Check if task is ready
    # -------------------------
    def is_ready(self, task_id):
        node = self.graph.nodes[task_id]
        if node["status"] != TaskStatus.PENDING:
            return False

        # --- Local dependencies ---
        for pred in node["local_predecessors"]:
            if self.graph.nodes[pred]["status"] != TaskStatus.DONE:
                return False

        # --- External dependencies ---
        for pred in node["external_predecessors"]:
            if pred not in self.external_done:
                return False

        return True
    
    # -------------------------
    # Check if any local tasks depend on a given external task
    # -------------------------
    def depends_on_external(self, external_task_id):
        for data in self.graph.nodes.values():
            if external_task_id in data["external_predecessors"]:
                return True
        return False

    # -------------------------
    # Incremental update from external completion
    # -------------------------
    def handle_external_completion(self, external_task_id, task_status):
        # Register external completion
        if(task_status == TaskStatus.DONE):
            self.external_done.add(external_task_id)

        for task_id, data in self.graph.nodes(data=True):
            if external_task_id in data["external_predecessors"]:
                if self.is_ready(task_id):
                    self.graph.nodes[task_id]["status"] = TaskStatus.READY
                    

    def update_status(self, task_id, status):
        """
        Update the status of a task in the Local graph.
        Slightly different of the Local Graph (here the task is direct in the node and not as assignment)
        """
        if task_id not in self.graph:
            raise ValueError(f"Task {task_id} not found.")

        if not isinstance(status, TaskStatus):
            raise TypeError("status must be TaskStatus Enum.")
    
        current = self.graph.nodes[task_id]["status"]

        if not TaskStatus.is_valid_transition(current, status):
            raise ValueError(
                f"[{task_id}] Illegal transition {current.name} → {status.name}"
            )

        self.graph.nodes[task_id]["status"] = status  

    def get_successors(self, task_id):
        successor_agents = set()

        node = self.graph.nodes[task_id]

        # Only external successors need notification
        for succ_task in node["external_successors"]:
            succ_agent = self.task_to_agent.get(succ_task)

            # Skip if same agent or unknown
            if succ_agent and succ_agent != self.agent_id:
                successor_agents.add(succ_agent)

        return successor_agents

