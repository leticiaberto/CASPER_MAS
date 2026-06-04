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

        tasks_assigned = False
        # --- Add my tasks ---
        for node, agent in self.task_to_agent.items():
            if agent == self.agent_id:
                tasks_assigned = True
                global_data = self.global_graph.nodes[node]
                required_constraints = global_data.get("required_constraints", {})

                # time_to_clean: None means the field is absent (no special gating).
                # False means the supervisor must explicitly clear this task before it can go READY.
                # True means clearance has been received and the task can proceed normally.
                raw_ttc = global_data.get("time_to_clean", None)
                time_to_clean = False if raw_ttc is False else None

                # time_to_clean tasks start as ASSIGNED: the supervisor has
                # allocated them but must explicitly clear them before they
                # can become PENDING → READY.  All other tasks start PENDING.
                initial_status = TaskStatus.ASSIGNED if time_to_clean is False else TaskStatus.PENDING

                self.graph.add_node(
                    node,
                    status=initial_status,
                    local_predecessors=set(),
                    external_predecessors=set(),
                    local_successors=set(),
                    external_successors=set(),
                    required_constraints=required_constraints,
                    workspace=required_constraints.get("workspace"),
                    time_to_clean=time_to_clean,
                )
        if not tasks_assigned:
            print(f"[{self.agent_id}] Warning: No tasks assigned for me.")

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
 
        # time_to_clean tasks sit in ASSIGNED until the supervisor clears them,
        # then transition to PENDING before the normal readiness check runs.
        # Any other status means the task is already past ready or not yet here.
        if node["status"] not in (TaskStatus.PENDING, TaskStatus.ASSIGNED):
            return False
 
        # --- Supervisor clearance gate ---
        # ASSIGNED + time_to_clean=False → clearance not yet received, not ready.
        # ASSIGNED + time_to_clean=True  → clearance received; fall through to dep check.
        if node["status"] == TaskStatus.ASSIGNED and node.get("time_to_clean") is False:
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

    # -------------------------
    # Supervisor clearance for time_to_clean tasks
    # -------------------------
    def receive_clearance(self, task_id):
        """
        Called when the supervisor sends a task_ready_clearance message.
        Flips time_to_clean from False → True so is_ready() can proceed.
        Returns True if the task became READY as a result.
        """
        if task_id not in self.graph:
            raise ValueError(f"Task {task_id} not found in local graph.")
 
        node = self.graph.nodes[task_id]
 
        if node.get("time_to_clean") is not False:
            # Field absent or already cleared — nothing to do
            return False
 
        node["time_to_clean"] = True
 
        # Advance from ASSIGNED → PENDING now that the clearance gate is lifted,
        # so the normal is_ready() dep check can run cleanly.
        if node["status"] == TaskStatus.ASSIGNED:
            node["status"] = TaskStatus.PENDING
 
        if self.is_ready(task_id):
            node["status"] = TaskStatus.READY
            return True
 
        return False
