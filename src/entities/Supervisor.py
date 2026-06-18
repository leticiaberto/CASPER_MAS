from src.graph.GraphVisualizer import GraphVisualizer
from src.graph.TaskAssignment import TaskAssignment, TaskStatus
    
class Supervisor:
    def __init__(self, name, agent, publish_fn):
        self.name = name
        self.graph_visualizer = GraphVisualizer()# All agentes could have this, but only the supervisor will use it for now.
        self.publish = publish_fn
        self.agent = agent   # reference to owner Agent
        self.release_clean_msg = False
    
    @property
    def constraints(self):
        return self.agent.constraints

    @property
    def skill_model(self):
        return self.agent.skills

    #TODO: extend to support contextual trust levels and other constraints. 
    def score_agents_for_task(self, G, agents, mode="combined", supervisor_id=None):
        """
        Returns a dict {task_id: [(agent_id, score), ...]} sorted by score descending.
 
        Parameters:
        - G: networkx DiGraph with tasks as nodes.
        - agents: dict {agent_id: ContextualSkillModel OR PartnerAgent}
        - mode: str, one of ["skill", "preference", "combined"]
        """
 
        assert mode in {"skill", "preference", "combined"}, f"Invalid mode: {mode}"
 
        task_rankings = {}
 
        for node_id in G.nodes:
            task = G.nodes[node_id]

            scored_agents = []

            # If is the main goal, set everyone as part of the team, but the responsible for checking is the supervisor
            if(task.get("is_end_goal", False) and supervisor_id is not None):
                # Skip end goals for all supervisors except the one who owns them
                scored_agents.append((supervisor_id, 1.0))
                for agent_id, agent_obj in agents.items():
                    if agent_id != supervisor_id:
                        scored_agents.append((agent_id, 0.0))
 
            else:# Not end goal, normal process of checking skills and constraints to assign the best agent(s).
                context = task.get("context")
                required_skills = task.get("required_skills", {})
                required_constraints = task.get("required_constraints", {})
    
                # Iterate over dictionary
                for agent_id, agent_obj in agents.items():
    
                    # --- Extract skill model ---
                    if hasattr(agent_obj, "skills"):
                        # PartnerAgent case
                        skill_model = agent_obj.skills
                        constraints = getattr(agent_obj, "constraints", {})
                    else:
                        # ContextualSkillModel case (supervisor's own entry)
                        skill_model = self.skill_model
                        constraints = self.constraints
    
                    # workspace lives inside the constraints dict (loaded from YAML)
                    agent_workspaces = constraints.get("workspace")
    
                    # --- 1. Context check ---
                    agent_contexts = set()
                    for s in skill_model.skill_level:
                        agent_contexts.update(skill_model.skill_level[s].keys())
    
                    if context not in agent_contexts:
                        continue
    
                    # --- 1.5. Workspace check ---
                    # The agent must be able to access ALL workspaces required by the task.
                    # If the task declares workspaces but the agent declares none → reject.
                    task_workspaces = task.get("required_constraints", {}).get("workspace")
    
                    if task_workspaces is not None:
                        if agent_workspaces is None:
                            continue  # task requires specific workspaces; agent declares none → skip
                        task_ws_set = (
                            {w.lower() for w in task_workspaces}
                            if isinstance(task_workspaces, list)
                            else {task_workspaces.lower()}
                        )
                        agent_ws_set = (
                            {w.lower() for w in agent_workspaces}
                            if isinstance(agent_workspaces, list)
                            else {agent_workspaces.lower()}
                        )
                        if not task_ws_set.issubset(agent_ws_set):
                            continue  # agent missing at least one required workspace → skip
    
                    # --- 2. Skill & preference scoring ---
                    skill_ok = True
                    score = 0.0
    
                    for skill, min_level in required_skills.items():
                        level = skill_model.skill_level.get(skill, {}).get(context, 0.0)
                        pref = skill_model.skill_preference.get(skill, {}).get(context, 0.0)
    
                        if level < min_level:
                            skill_ok = False
                            break
    
                        if mode == "skill":
                            score += level
                        elif mode == "preference":
                            score += pref
                        elif mode == "combined":
                            score += level * (1 + pref)
    
                    if not skill_ok:
                        continue
    
                    # --- 3. Constraint check ---
                    constraint_ok = True
                    for key, value in required_constraints.items():
    
                        if key == "workspace":
                            continue  # already enforced by step 1.5 — skip here
    
                        if key not in constraints:
                            continue  # unknown ≠ forbidden
    
                        agent_value = constraints[key]
    
                        if isinstance(value, (int, float)):
                            if agent_value < value:
                                constraint_ok = False
                                break
                        else:
                            if agent_value != value:
                                constraint_ok = False
                                break
    
                    if not constraint_ok:
                        continue
    
                    scored_agents.append((agent_id, score))
    
                # --- 4. Sort descending ---
                scored_agents.sort(key=lambda x: x[1], reverse=True)
                
            task_rankings[node_id] = scored_agents
 
        return task_rankings

    def get_top_candidates(self, task, scored, selected_agent_id, top_k=3):
        """
        Returns the top-k candidates excluding the selected agent.

        Parameters:
        - scored_agents: list of tuples [(agent_id, score), ...] sorted descending
        - selected_agent_id: the agent ID to exclude
        - top_k: maximum number of agents to return

        Returns:
        - list of strings: ["AgentA (0.95)", "AgentB (0.80)", ...]
        """
        if "top_candidates" in task:
            return task["top_candidates"]
    
        top_candidates = []

        for aid, score in scored:
            if aid != selected_agent_id:
                top_candidates.append(f"{aid} ({score:.2f})")
            if len(top_candidates) >= top_k:
                break
        return top_candidates
    

    def get_selected_agent(self, task, node_id, capable_agents):
        """
        Returns:
            selected_agent (Agent or None)
            selected_score (float or None)
            
        """
        if "selected_agent" in task:
            return (task.get("selected_agent"), task.get("selected_score"), capable_agents.get(node_id, []))

        # fallback if not precomputed
        scored = capable_agents.get(node_id, [])

        if not scored:
            return None, None, []

        return scored[0][0], scored[0][1], scored
    

    def assign_agents_to_tasks(self, G, agents, mode, top_k, debug, supervisor_id):
        # Score all agents for all tasks
        capable_agents = self.score_agents_for_task(G, agents, mode, supervisor_id) 

        if(debug):
            print("\n--- Agent rankings per task ---")
            for task, agent_rankings in capable_agents.items():
                print(f"Task: {task}")
                for agent, score in agent_rankings:
                    print(f"  Agent: {agent}, Score: {score}")
        
        # Select + store results inside each task
        unassigned_tasks = []
        for node_id in G.nodes:
            task = G.nodes[node_id]

            # Selected agent
            selected_agent, selected_score, scored = self.get_selected_agent(task, node_id, capable_agents)

            # Top candidates
            top_candidates = self.get_top_candidates(task, scored, selected_agent, top_k)
     
            assignment = TaskAssignment(
                task_id=node_id,
                selected_agent=selected_agent,
                selected_score=selected_score,
                top_candidates=top_candidates,
                status=TaskStatus.ASSIGNED if selected_agent else TaskStatus.NOT_ASSIGNED
            )

            # Store ONE structured object
            G.nodes[node_id]["assignment"] = assignment

            if assignment.status == TaskStatus.NOT_ASSIGNED:
                unassigned_tasks.append(node_id)

            if(debug):
                print(
                    f"Selected agent for task {node_id}: "
                    f"{selected_agent if selected_agent else None}, "
                    f"score: {selected_score}"
                )

        # If any task couldn't be assigned to an agent, the plan is unworkable.
        # Persist the graph as-is for inspection, alert, and shut the whole system down
        # instead of handing out a partial/broken assignment batch.
        if unassigned_tasks:
            self.graph_visualizer.export_multiagent_graph(
                G,
                output_name=f"[{self.name}] Global_Supervisor_Allocation_FAILED",
                palette_mode="pastel",
            )
            print(
                f"[{self.name}] Allocation failed: {len(unassigned_tasks)} task(s) "
                f"could not be assigned to any agent: {unassigned_tasks}. "
                f"Shutting down."
            )
            self.publish(
                "shutdown",
                {
                    "reason": "unassigned_tasks",
                    "unassigned_tasks": unassigned_tasks,
                    "supervisor_id": supervisor_id,
                },
            )
            # Broadcasts aren't delivered back to the sender (same convention
            # as "SUPERVISOR" / "party_over"), so also fire the hook locally.
            self.agent.handle_shutdown("unassigned_tasks", unassigned_tasks)
            return
        
        self.send_task_assignment_batch(G)

    def send_task_assignment_batch(self, G):
        # Collect all serialized assignments
        all_assignments = {
            node_id: G.nodes[node_id]["assignment"].serialize()
            for node_id in G.nodes
        }

        # Publish all at once
        self.publish("task_assignment_batch", all_assignments)

    def release_task(self, task_id, assigned_agent_id):
        """
        Supervisor explicitly clears a task whose time_to_clean was False.
        Sends a task_ready_clearance message directly to the assigned agent.
        """
        self.publish(
            "task_ready_clearance",
            {"task_id": task_id},
            assigned_agent_id,
        )
        print(f"[{self.name}] Sent task_ready_clearance for task '{task_id}' to agent '{assigned_agent_id}'.")