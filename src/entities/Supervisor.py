from src.graph.Graph import GraphVisualizer
from src.graph.TaskAssignment import TaskAssignment, TaskStatus
    
class Supervisor:
    def __init__(self, name):
        self.name = name
        self.role = "supervisor"
        self.graph_visualizer = GraphVisualizer()# All agentes could have this, but only the supervisor will use it for now.
        
    def set_teamsize(self, teamsize):
        self.teamsize = teamsize
        print(f"{self.name} set teamsize to {self.teamsize}")
    
    #TODO: extend to support contextual trust levels and other constraints. 
    def score_agents_for_task(self, G, agents, mode="combined"):
        """
        Returns a dict {task_id: [(agent_id, score), ...]} sorted by score descending.
        
        Parameters:
        - G: networkx DiGraph with tasks as nodes.
        - agents: list of Agent objects.
        - mode: str, one of ["skill", "preference", "combined"]
            "skill"      -> score = sum of skill levels only
            "preference" -> score = sum of preferences only
            "combined"   -> score = sum(level * (1 + pref))
        """

        assert mode in {"skill", "preference", "combined"}, f"Invalid mode: {mode}"

        task_rankings = {}

        for node_id in G.nodes:
            task = G.nodes[node_id]

            context = task.get("context")
            required_skills = task.get("required_skills", {})
            required_constraints = task.get("required_constraints", {})

            scored_agents = []

            for agent in agents:
                # --- 1. Context check ---
                agent_contexts = set()
                for s in agent.skills.skill_level:
                    agent_contexts.update(agent.skills.skill_level[s].keys())

                if context not in agent_contexts:
                    continue

                # --- 2. Skill & preference scoring ---
                skill_ok = True
                score = 0.0

                for skill, min_level in required_skills.items():
                    level = agent.skills.skill_level.get(skill, {}).get(context, 0.0)
                    pref = agent.skills.skill_preference.get(skill, {}).get(context, 0.0)

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

                # ---- 3. Constraint check (lenient) ----
                constraint_ok = True
                for key, value in required_constraints.items():
                    # Constraints become blocking only when explicitly incompatible. Tasks can introduce new constraints without breaking old agents. unknown ≠ forbidden
                    if key not in agent.constraints:
                        continue  # assume agent satisfies it. 

                    agent_value = agent.constraints[key]

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

                scored_agents.append((agent.id, score))

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
    

    def assign_agents_to_tasks(self, G, agents, mode, top_k, debug=False):
        # Score all agents for all tasks
        capable_agents = self.score_agents_for_task(G, agents, mode) 

        if(debug):
            print("\n--- Agent rankings per task ---")
            for task, agent_rankings in capable_agents.items():
                print(f"Task: {task}")
                for agent, score in agent_rankings:
                    print(f"  Agent: {agent}, Score: {score}")
        
        # Select + store results inside each task
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
                status=TaskStatus.ASSIGNED if selected_agent else TaskStatus.PENDING
            )

            # Store ONE structured object
            G.nodes[node_id]["assignment"] = assignment

            if(debug):
                print(
                    f"Selected agent for task {node_id}: "
                    f"{selected_agent if selected_agent else None}, "
                    f"score: {selected_score}"
                )