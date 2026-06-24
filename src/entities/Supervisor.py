from src.graph.GraphVisualizer import GraphVisualizer
from src.graph.TaskAssignment import TaskAssignment, TaskStatus
from src.analysis.ExperimentLogger import ExperimentLogger
    
class Supervisor:
    def __init__(self, name, agent, publish_fn, run_id=None, log_dir="experiment_logs"):
        self.name = name
        self.graph_visualizer = GraphVisualizer()# All agentes could have this, but only the supervisor will use it for now.
        self.publish = publish_fn
        self.agent = agent   # reference to owner Agent
        self.release_clean_msg = False

        # Experiment analytics: records task allocation (initial vs. final,
        # i.e. before/after rebalancing), per-agent workload, and run-level
        # efficiency metrics to CSV for later offline analysis. run_id
        # defaults to a timestamp+uuid inside ExperimentLogger if not given,
        # so multiple runs can be told apart when their CSVs are merged.
        self.experiment_logger = ExperimentLogger(run_id=run_id, output_dir=log_dir)
    
    @property
    def constraints(self):
        return self.agent.constraints

    @property
    def skill_model(self):
        return self.agent.skills

    # Weight given to skill (vs. preference) inside the balance_skill_preference
    # combined score. alpha=0.7 -> skill counts 70%, preference 30%.
    BALANCE_SKILL_PREFERENCE_ALPHA = 0.7

    #TODO: extend to support contextual trust levels and other constraints. 
    def score_agents_for_task(self, G, agents, mode="combined", supervisor_id=None):
        """
        Returns a dict {task_id: [(agent_id, score), ...]} sorted by score descending.
 
        Parameters:
        - G: networkx DiGraph with tasks as nodes.
        - agents: dict {agent_id: ContextualSkillModel OR PartnerAgent}
        - mode: str, one of:
            - "skill": rank by raw skill level.
            - "preference": rank by preference. Preference is stored as a
              RANK (1 = most preferred) that's only meaningful relative to
              the agent's OWN ranked skills within the task's context, so
              it's normalized per-agent-per-context before use:
              preference_score = (N - rank + 1) / N, where N = how many
              skills that agent has ranked within this context. Rank 1 of
              N -> 1.0, rank N of N -> 1/N (never hits 0). Higher is better.
            - "balance_skill_preference": weighted average of skill level
              and the normalized preference_score above (see
              BALANCE_SKILL_PREFERENCE_ALPHA), so a less-skilled but much-
              more-interested agent can still win.
            - "balance_workload": skill/preference are only used as an
              eligibility filter (must meet required_skills); all eligible
              agents get an equal placeholder score, since the actual
              decision is workload-driven and made at selection time in
              get_selected_agent / assign_agents_to_tasks.
        """
 
        assert mode in {"skill", "preference", "balance_skill_preference", "balance_workload"}, f"Invalid mode: {mode}"
 
        task_rankings = {}
 
        for node_id in G.nodes:
            task = G.nodes[node_id]

            scored_agents = []

            # If is the main goal, set everyone as part of the team, but the responsible for checking is the supervisor
            if(task.get("is_end_goal", False) and supervisor_id is not None):
                # Skip end goals for all agents except the one who owns the supervisor
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
                    # NOTE: "preference" here is a RANK (1 = most preferred),
                    # so we convert it into a normalized 0-1 score before
                    # using it in any formula — that way "higher score =
                    # better" holds uniformly across all modes, and an
                    # agent's preference scores are spread across their own
                    # ranked list rather than collapsing toward 0 for
                    # anything past their first couple of picks.
                    #
                    # Normalization is per-agent, per-context: N = how many
                    # skills THIS agent has ranked within THIS task's
                    # context (not the candidate pool, not other contexts).
                    # preference_score = (N - rank + 1) / N, so rank 1 of N
                    # -> 1.0 and rank N of N -> 1/N (never hits 0).
                    skills_ranked_in_context = sum(
                        1
                        for prefs_by_context in skill_model.skill_preference.values()
                        if context in prefs_by_context
                    )

                    skill_ok = True
                    score = 0.0
    
                    for skill, min_level in required_skills.items():
                        level = skill_model.skill_level.get(skill, {}).get(context, 0.0)
                        rank = skill_model.skill_preference.get(skill, {}).get(context)
    
                        if level < min_level:
                            skill_ok = False
                            break
    
                        # Normalization only makes sense for rank >= 1 and
                        # N >= 1; treat a missing/non-positive rank, or no
                        # ranked skills in this context, as "no preference"
                        # (0.0 contribution) rather than dividing by zero.
                        if rank and rank > 0 and skills_ranked_in_context > 0:
                            pref_score = (skills_ranked_in_context - rank + 1) / skills_ranked_in_context
                        else:
                            pref_score = 0.0
    
                        if mode == "skill":
                            score += level
                        elif mode == "preference":
                            score += pref_score
                        elif mode == "balance_skill_preference":
                            score += (
                                self.BALANCE_SKILL_PREFERENCE_ALPHA * level
                                + (1 - self.BALANCE_SKILL_PREFERENCE_ALPHA) * pref_score
                            )
                        elif mode == "balance_workload":
                            # Eligibility-only mode: skill/preference don't
                            # contribute to score here, they only gate
                            # skill_ok above. Real selection happens by
                            # workload in get_selected_agent.
                            pass
    
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

                    # Normalizing fixes cross-task comparison within a mode, not cross-mode comparison.
                    if required_skills:
                        score /= len(required_skills)#  every task's score lands in roughly the same [0,1] range regardless of how many skills it requires.
    
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
    

    def get_selected_agent(self, task, node_id, capable_agents, workload=None):
        """
        Returns:
            selected_agent (Agent or None)
            selected_score (float or None)

        Parameters:
        - workload: dict {agent_id: current_task_count}, used to break ties
          between equally-scored agents by picking the one with fewer tasks
          assigned so far in this allocation run. If two agents are tied on
          both score and workload, the first one in the ranking wins.

          This is also how "balance_workload" mode makes its real decision:
          score_agents_for_task gives every eligible agent the same
          placeholder score for that mode, so ALL eligible agents are "tied"
          here and the selection collapses to "fewest tasks assigned so far,
          then ranking order" — i.e. pure workload balancing among capable
          agents.
        """
        if "selected_agent" in task:
            return (task.get("selected_agent"), task.get("selected_score"), capable_agents.get(node_id, []))

        # fallback if not precomputed
        scored = capable_agents.get(node_id, [])

        if not scored:
            return None, None, []

        if workload is None:
            return scored[0][0], scored[0][1], scored

        # Among the agents tied for the top score, pick the one with the
        # least tasks assigned so far (original ranking order breaks any
        # remaining tie, since `scored` is already sorted by score and
        # `min` keeps the first occurrence on ties).
        top_score = scored[0][1]
        tied_top = [(aid, score) for aid, score in scored if score == top_score]

        selected_agent, selected_score = min(
            tied_top, key=lambda pair: workload.get(pair[0], 0)
        )

        return selected_agent, selected_score, scored
    

    def _eligible_agents_for_task(self, node_id, capable_agents):
        """Returns the set of agent_ids eligible (constraint/skill-wise) for a task."""
        return {aid for aid, _score in capable_agents.get(node_id, [])}

    def _agent_fit_score(self, node_id, agent_id, capable_agents):
        """
        Looks up agent_id's score for node_id within capable_agents (the
        mode-specific skill/preference ranking computed before workload was
        ever considered). Used only to pick the *best fitting* task to move
        during rebalancing — not to drive the original assignment decision.

        Note: rebalancing only ever runs for balance_workload mode, and in
        that mode every eligible agent shares the same placeholder score
        (see score_agents_for_task), so this will tie at that placeholder
        value for every eligible target. That's expected — in
        balance_workload mode, fit genuinely doesn't matter beyond
        eligibility, so ties just fall to whichever eligible target is
        encountered first.
        """
        for aid, score in capable_agents.get(node_id, []):
            if aid == agent_id:
                return score
        return float("-inf")  # not eligible at all

    def rebalance_workload(self, G, capable_agents, workload, top_k=3):
        """
        Rule 4: post-allocation rebalancing for balance_workload mode.

        Greedy approach: while the busiest and least-busy agent differ by
        more than 1 task, try to move one task from the busiest agent to a
        less-loaded agent that is also eligible for that task (per the
        constraint/skill filtering already computed in capable_agents).

        Among the busiest agent's movable tasks, prefer moving the task
        where the receiving (less-loaded) agent has the best skill/
        preference fit score for that task — i.e. the move that wastes the
        least capability. Constraints and skill eligibility are preserved
        exactly as already computed; rebalancing never assigns a task to an
        agent who wasn't already a valid candidate for it.

        Mutates `workload` and each task's stored "assignment" / G node data
        in place. Returns nothing.
        """
        # Snapshot of which task each agent currently holds, for quick lookup
        # of "what can the busiest agent give up".
        def tasks_of(agent_id):
            return [
                node_id for node_id in G.nodes
                if G.nodes[node_id]["assignment"].selected_agent == agent_id
            ]

        max_iterations = len(G.nodes) + 1  # hard safety cap, avoids infinite loops
        for _ in range(max_iterations):
            if not workload:
                break

            busiest_agent = max(workload, key=lambda aid: workload[aid])
            least_loaded_agent = min(workload, key=lambda aid: workload[aid])

            if workload[busiest_agent] - workload[least_loaded_agent] <= 1:
                break  # already balanced within tolerance

            # Find the best (task, target_agent) move: among the busiest
            # agent's tasks, only those movable to SOME agent with strictly
            # lower workload than the busiest agent are candidates. Among
            # those, prefer the target with the best fit score for that task.
            best_move = None  # (fit_score, node_id, target_agent)

            for node_id in tasks_of(busiest_agent):
                eligible = self._eligible_agents_for_task(node_id, capable_agents)

                for target_agent in eligible:
                    if target_agent == busiest_agent:
                        continue
                    if workload.get(target_agent, 0) >= workload[busiest_agent] - 1:
                        continue  # moving here wouldn't help (or would just flip the imbalance)

                    fit = self._agent_fit_score(node_id, target_agent, capable_agents)

                    if best_move is None or fit > best_move[0]:
                        best_move = (fit, node_id, target_agent)

            if best_move is None:
                break  # no beneficial move available; can't improve further

            _, node_id, target_agent = best_move
            assignment = G.nodes[node_id]["assignment"]

            old_score = self._agent_fit_score(node_id, target_agent, capable_agents)
            assignment.selected_agent = target_agent
            assignment.selected_score = old_score

            # top_candidates was computed excluding the previously selected
            # agent; recompute it so it excludes the new one instead and
            # stays consistent with who actually holds the task now.
            scored_for_task = capable_agents.get(node_id, [])
            assignment.top_candidates = self.get_top_candidates(
                {}, scored_for_task, target_agent, top_k=top_k
            )

            workload[busiest_agent] -= 1
            workload[target_agent] = workload.get(target_agent, 0) + 1

    def _check_supervisor_eligibility(self, G, supervisor_id):
        """
        Verifies that the agent designated as supervisor_id actually meets
        the required_skills of the end-goal task (is_end_goal=True, e.g.
        BarbecueParty's "supervise"/management requirement) BEFORE any
        scoring runs.

        score_agents_for_task's is_end_goal branch (see above) never checks
        required_skills for the end-goal task -- it unconditionally hands
        the supervisor a score of 1.0. That is fine for selecting *which
        node* represents the end goal, but it means an unqualified
        supervisor would otherwise sail through with no eligibility check
        at all. This method closes that gap, using the supervisor's own
        skill model (self.skill_model / self.constraints, the same
        "ContextualSkillModel case (supervisor's own entry)" used
        elsewhere in this file) rather than looking the supervisor up in
        the `agents` dict, since the supervisor's own entry is tracked on
        `self`, not in that dict.

        Returns True if eligible (or if there is no end-goal task / no
        required_skills on it -- nothing to check). Returns False and
        triggers the same shutdown path used for unassigned_tasks if the
        supervisor fails its own end-goal's required_skills.
        """
        end_goal_node = None
        end_goal_task = None
        for node_id in G.nodes:
            task = G.nodes[node_id]
            if task.get("is_end_goal", False):
                end_goal_node = node_id
                end_goal_task = task
                break

        if end_goal_task is None:
            return True  # no end-goal task in this graph; nothing to check

        required_skills = end_goal_task.get("required_skills", {})
        if not required_skills:
            return True  # end-goal task declares no skill requirements

        context = end_goal_task.get("context")
        missing = {}
        for skill, min_level in required_skills.items():
            level = self.skill_model.skill_level.get(skill, {}).get(context, 0.0)
            if level < min_level:
                missing[skill] = {"required": min_level, "actual": level}

        if not missing:
            return True

        print(
            f"[{self.name}] Supervisor '{supervisor_id}' does not meet the "
            f"required_skills for end-goal task '{end_goal_node}': {missing}. "
            f"Shutting down."
        )
        self.publish(
            "shutdown",
            {
                "reason": "supervisor_unqualified",
                "supervisor_id": supervisor_id,
                "end_goal_task": end_goal_node,
                "missing_skills": missing,
            },
        )
        # Broadcasts aren't delivered back to the sender (same convention
        # as the unassigned_tasks shutdown below), so also fire the hook
        # locally.
        self.agent.handle_shutdown("supervisor_unqualified", missing)
        return False

    def assign_agents_to_tasks(self, G, agents, mode, top_k, debug, supervisor_id):
        # Verify the supervisor itself is qualified for the end-goal task
        # BEFORE scoring anything -- score_agents_for_task's is_end_goal
        # branch always gives the supervisor a 1.0 regardless of skill, so
        # this check has to happen here, not inside scoring.
        if not self._check_supervisor_eligibility(G, supervisor_id):
            return

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
        workload = {}  # agent_id -> number of tasks assigned so far in this run
        for node_id in G.nodes:
            task = G.nodes[node_id]

            # Selected agent (ties broken by current workload, then ranking order)
            selected_agent, selected_score, scored = self.get_selected_agent(task, node_id, capable_agents, workload)

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
            else:
                workload[selected_agent] = workload.get(selected_agent, 0) + 1

            if(debug):
                print(
                    f"Selected agent for task {node_id}: "
                    f"{selected_agent if selected_agent else None}, "
                    f"score: {selected_score}"
                )

        # Snapshot the assignment as it stands right now (before any
        # rebalancing) so we can later diff "initial vs final" per task
        # for the experiment CSVs, regardless of which mode is running.
        self.experiment_logger.snapshot_initial_assignment(G, workload)

        # Rule 4: rebalance workload after the initial greedy allocation.
        # Only applies to balance_workload mode, and only when every task
        # was successfully assigned (rebalancing an incomplete/broken
        # allocation isn't meaningful — we shut down below instead).
        if mode == "balance_workload" and not unassigned_tasks:
            if(debug):
                print("\n--- Workload before rebalancing ---")
                print(workload)

            self.rebalance_workload(G, capable_agents, workload, top_k)

            if(debug):
                print("--- Workload after rebalancing ---")
                print(workload)

        # Log task allocation (initial vs. final agent/score, rebalance
        # diff, predecessor/successor-based efficiency metrics) and the
        # per-agent workload + run-level summary, to CSV for offline
        # analysis. Logged here -- after rebalancing, before the
        # unassigned-tasks check -- so failed/partial runs are captured
        # too (num_unassigned_tasks in the summary row will reflect that).
        self.experiment_logger.log_allocation(G, capable_agents, workload, mode, top_k, debug)

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

    # ----------------------------
    # Experiment analytics
    # ----------------------------
    def log_task_execution(self, agent_id, task_id, global_graph_G, extra_fields=None):
        """
        Records, for offline analysis, the order in which `agent_id`
        executed its tasks plus a "supervisor view" snapshot of `task_id`
        in the global task graph at the moment it ran (predecessors,
        successors, whether it was a bottleneck/source/leaf, and how many
        downstream tasks it just unblocked). See
        ExperimentLogger.log_task_execution for the full field list.

        Any agent can call this through `self.supervisor.log_task_execution(...)`
        if it holds the supervisor role, but since every agent only has a
        `self.supervisor` attribute when role == SUPERVISOR, regular team
        members should go through `Agent.log_task_execution(...)` instead
        (see Agent.py), which forwards to the supervisor's logger over
        the same `global_graph` every agent already holds a read-only
        copy of.
        """
        return self.experiment_logger.log_task_execution(agent_id, task_id, global_graph_G, extra_fields)