"""
ExperimentLogger
================

Centralizes all CSV logging used for *offline* analysis of experiments:
how tasks were allocated, whether/how the allocation was rebalanced, how
efficient that allocation was (idle agents, workload spread, skill-fit
quality, critical-path awareness), and -- per agent -- the actual order
tasks were executed in together with a "supervisor view" snapshot of each
task's global importance at the moment it ran.

Design notes
------------
- This module deliberately does NOT import TaskAssignment / GlobalGraph /
  LocalGraph. It only relies on the attribute surface that Supervisor.py
  and Agent.py already use (assignment.selected_agent, assignment.status,
  assignment.top_candidates, networkx G.predecessors/G.successors, task
  dict .get(...)), accessed defensively with getattr/hasattr so it keeps
  working even if those classes evolve a bit.
- All writers are append-friendly: each row carries a `run_id` so multiple
  experiment runs can be concatenated into one CSV and compared later
  (e.g. with pandas `groupby("run_id")`).
- Nothing here changes scheduling/allocation behavior -- it only observes
  and records it.
"""

import csv
import json
import os
import statistics
import time
import uuid


class ExperimentLogger:
    """
    One ExperimentLogger instance == one experiment run.

    Typical usage (Supervisor side):
        logger = ExperimentLogger(run_id="exp_2026_06_24_01", output_dir="logs")
        ...
        logger.snapshot_initial_assignment(G, capable_agents, workload)
        ... rebalance happens ...
        logger.log_allocation(G, capable_agents, workload, mode, top_k)

    Typical usage (Agent side, once per executed task):
        logger.log_task_execution(agent_id, task_id, global_graph.G, order_index)
    """

    def __init__(self, run_id=None, output_dir="experiment_logs"):
        self.run_id = run_id or f"run_{int(time.time())}_{uuid.uuid4().hex[:6]}"
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        # Filled in by snapshot_initial_assignment(), consumed by log_allocation()
        self._initial_snapshot = None  # {node_id: {"agent":..., "score":..., "top_candidates":...}}
        self._initial_workload = None  # {agent_id: count}

        # Running per-agent execution order counter, kept here so callers
        # don't have to thread an index through every step().
        self._exec_order_counters = {}

        # Running idle-event counter, used only to give each row in
        # idle_events.csv a stable sequence number within the run.
        self._idle_event_counter = 0

        self._task_allocation_path = os.path.join(self.output_dir, "task_allocation.csv")
        self._agent_workload_path = os.path.join(self.output_dir, "agent_workload.csv")
        self._allocation_summary_path = os.path.join(self.output_dir, "allocation_summary.csv")
        self._task_execution_path = os.path.join(self.output_dir, "task_execution.csv")
        self._idle_events_path = os.path.join(self.output_dir, "idle_events.csv")
        self._shutdown_events_path = os.path.join(self.output_dir, "shutdown_events.csv")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _safe_get_assignment_fields(assignment):
        """Pull the fields we care about off a TaskAssignment-like object, defensively."""
        if assignment is None:
            return None, None, []
        agent = getattr(assignment, "selected_agent", None)
        score = getattr(assignment, "selected_score", None)
        top_candidates = getattr(assignment, "top_candidates", None) or []
        return agent, score, top_candidates

    @staticmethod
    def _status_value(assignment):
        status = getattr(assignment, "status", None)
        # TaskStatus is an Enum-like; .value/.to_wire() are both seen in Agent.py
        if status is None:
            return None
        if hasattr(status, "to_wire"):
            try:
                return status.to_wire()
            except Exception:
                pass
        return getattr(status, "value", str(status))

    @staticmethod
    def _best_possible_score(node_id, capable_agents):
        """Top score any eligible agent had for this task, ignoring who was actually picked."""
        scored = capable_agents.get(node_id, []) if capable_agents else []
        if not scored:
            return None
        return max(score for _, score in scored)

    @staticmethod
    def _as_array(items):
        """
        Serialize a list as a JSON array string, e.g. ["B", "C"], instead
        of a delimiter-joined string like "B; C".

        Spreadsheet apps disagree on the CSV column delimiter (some
        locales default to ";" instead of ","), so any joiner character
        risks being mis-split into separate columns on open. A JSON array
        always stays inside one quoted CSV cell regardless of delimiter,
        and is trivial to read back later with `json.loads(cell)` in
        pandas (e.g. `df["successors"].apply(json.loads)`) for plotting.
        """
        return json.dumps(list(items))

    @staticmethod
    def _write_rows(path, fieldnames, rows):
        """Append rows to CSV, writing the header only if the file doesn't exist yet."""
        file_exists = os.path.isfile(path)
        with open(path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            for row in rows:
                writer.writerow(row)

    # ------------------------------------------------------------------
    # Step 1: capture the assignment BEFORE rebalancing
    # ------------------------------------------------------------------
    def snapshot_initial_assignment(self, G, workload):
        """
        Call this right after the initial greedy assignment loop in
        assign_agents_to_tasks (i.e. right after the `for node_id in
        G.nodes:` block that builds `assignment`, and BEFORE
        rebalance_workload mutates anything). Captures, per task, the
        agent/score/top_candidates that were picked initially, plus a
        snapshot of the initial per-agent workload -- both needed later
        to compute the "what changed during rebalance" diff.
        """
        snapshot = {}
        for node_id in G.nodes:
            assignment = G.nodes[node_id].get("assignment")
            agent, score, top_candidates = self._safe_get_assignment_fields(assignment)
            snapshot[node_id] = {
                "agent": agent,
                "score": score,
                "top_candidates": list(top_candidates),
                "status": self._status_value(assignment),
            }
        self._initial_snapshot = snapshot
        self._initial_workload = dict(workload) if workload else {}
        return snapshot

    # ------------------------------------------------------------------
    # Step 2: after rebalance (or immediately, if mode has no rebalance
    # step), log everything: per-task allocation, per-agent workload, and
    # the run-level efficiency summary.
    # ------------------------------------------------------------------
    def log_allocation(self, G, capable_agents, workload, mode, top_k, debug=False):
        """
        Writes three CSVs:
          - task_allocation.csv   (one row per task)
          - agent_workload.csv    (one row per agent)
          - allocation_summary.csv (one row for the whole run)

        Must be called after the final assignment is settled (i.e. after
        rebalance_workload, if it ran). If snapshot_initial_assignment()
        was never called, every task is treated as "no rebalance happened"
        (initial == final), which is the correct behavior for modes that
        don't rebalance.
        """
        initial_snapshot = self._initial_snapshot or {}
        initial_workload = self._initial_workload or {}

        task_rows = []
        rebalanced_count = 0
        skill_fit_gaps = []  # best_possible_score - actual_score, per assigned task
        unassigned_count = 0

        for node_id in G.nodes:
            node_data = G.nodes[node_id]
            assignment = node_data.get("assignment")
            final_agent, final_score, final_top_candidates = self._safe_get_assignment_fields(assignment)
            final_status = self._status_value(assignment)

            init = initial_snapshot.get(node_id, {})
            initial_agent = init.get("agent", final_agent)
            initial_score = init.get("score", final_score)

            was_rebalanced = (
                node_id in initial_snapshot
                and initial_agent is not None
                and final_agent is not None
                and initial_agent != final_agent
            )
            if was_rebalanced:
                rebalanced_count += 1

            predecessors = list(G.predecessors(node_id)) if G.has_node(node_id) else []
            successors = list(G.successors(node_id)) if G.has_node(node_id) else []

            best_possible = self._best_possible_score(node_id, capable_agents)
            if final_agent is None:
                unassigned_count += 1
            elif best_possible is not None and final_score is not None:
                skill_fit_gaps.append(best_possible - final_score)

            task_rows.append({
                "run_id": self.run_id,
                "task_id": node_id,
                "mode": mode,
                "is_end_goal": node_data.get("is_end_goal", False),
                "context": node_data.get("context"),
                "initial_agent": initial_agent,
                "initial_score": initial_score,
                "final_agent": final_agent,
                "final_score": final_score,
                "was_rebalanced": was_rebalanced,
                "final_status": final_status,
                "top_candidates": self._as_array(str(c) for c in final_top_candidates),
                "best_possible_score": best_possible,
                "skill_fit_gap": (best_possible - final_score) if (best_possible is not None and final_score is not None) else None,
                "num_predecessors": len(predecessors),
                "num_successors": len(successors),
                "predecessors": self._as_array(map(str, predecessors)),
                "successors": self._as_array(map(str, successors)),
                "is_bottleneck": len(successors) >= 2,  # unblocks 2+ downstream tasks
                "is_leaf": len(successors) == 0,
                "is_source": len(predecessors) == 0,
            })

        self._write_rows(
            self._task_allocation_path,
            fieldnames=list(task_rows[0].keys()) if task_rows else [
                "run_id", "task_id", "mode", "is_end_goal", "context", "initial_agent",
                "initial_score", "final_agent", "final_score", "was_rebalanced",
                "final_status", "top_candidates", "best_possible_score", "skill_fit_gap",
                "num_predecessors", "num_successors", "predecessors", "successors",
                "is_bottleneck", "is_leaf", "is_source",
            ],
            rows=task_rows,
        )

        # ---- agent_workload.csv ----
        all_agents = set(initial_workload.keys()) | set(workload.keys() if workload else [])
        # Also pick up agents that ended with 0 tasks but were eligible for
        # at least one task, so truly idle agents show up with count 0
        # rather than being absent from the CSV entirely.
        if capable_agents:
            for scored in capable_agents.values():
                for aid, _score in scored:
                    all_agents.add(aid)

        agent_rows = []
        final_counts = {aid: 0 for aid in all_agents}
        if workload:
            final_counts.update(workload)

        for agent_id in sorted(all_agents):
            initial_count = initial_workload.get(agent_id, 0)
            final_count = final_counts.get(agent_id, 0)
            agent_rows.append({
                "run_id": self.run_id,
                "agent_id": agent_id,
                "mode": mode,
                "initial_task_count": initial_count,
                "final_task_count": final_count,
                "task_count_delta": final_count - initial_count,
                "is_idle": final_count == 0,
            })

        self._write_rows(
            self._agent_workload_path,
            fieldnames=["run_id", "agent_id", "mode", "initial_task_count",
                        "final_task_count", "task_count_delta", "is_idle"],
            rows=agent_rows,
        )

        # ---- allocation_summary.csv ----
        final_task_counts = [row["final_task_count"] for row in agent_rows]
        num_agents = len(agent_rows)
        num_idle_agents = sum(1 for row in agent_rows if row["is_idle"])
        num_tasks = len(task_rows)

        summary_row = {
            "run_id": self.run_id,
            "mode": mode,
            "top_k": top_k,
            "num_tasks": num_tasks,
            "num_agents": num_agents,
            "num_unassigned_tasks": unassigned_count,
            "num_rebalanced_tasks": rebalanced_count,
            "pct_rebalanced": (rebalanced_count / num_tasks * 100.0) if num_tasks else 0.0,
            "num_idle_agents": num_idle_agents,
            "pct_idle_agents": (num_idle_agents / num_agents * 100.0) if num_agents else 0.0,
            "max_tasks_per_agent": max(final_task_counts) if final_task_counts else 0,
            "min_tasks_per_agent": min(final_task_counts) if final_task_counts else 0,
            "workload_spread": (max(final_task_counts) - min(final_task_counts)) if final_task_counts else 0,
            "workload_stdev": statistics.pstdev(final_task_counts) if len(final_task_counts) > 1 else 0.0,
            "avg_tasks_per_agent": statistics.mean(final_task_counts) if final_task_counts else 0.0,
            "avg_skill_fit_gap": statistics.mean(skill_fit_gaps) if skill_fit_gaps else None,
            "max_skill_fit_gap": max(skill_fit_gaps) if skill_fit_gaps else None,
            "num_bottleneck_tasks": sum(1 for row in task_rows if row["is_bottleneck"]),
            "num_source_tasks": sum(1 for row in task_rows if row["is_source"]),
            "num_leaf_tasks": sum(1 for row in task_rows if row["is_leaf"]),
            "timestamp": time.time(),
        }

        self._write_rows(
            self._allocation_summary_path,
            fieldnames=list(summary_row.keys()),
            rows=[summary_row],
        )

        if debug:
            print(f"[ExperimentLogger] Wrote {len(task_rows)} task rows, "
                  f"{len(agent_rows)} agent rows, 1 summary row for run '{self.run_id}'.")

        return summary_row

    # ------------------------------------------------------------------
    # Per-agent execution order + "supervisor view" of each task as it runs
    # ------------------------------------------------------------------
    def log_task_execution(self, agent_id, task_id, global_graph_G, extra_fields=None):
        """
        Call this once per task actually executed by an agent (e.g. right
        after `_execute_task_specific(task)` inside Agent.step(), or right
        when the task is marked RUNNING/DONE -- pick one consistent point).

        Records:
          - the order in which THIS agent executed its tasks (auto-
            incrementing per agent_id, 1-indexed)
          - a "supervisor view" snapshot of the task's position/importance
            in the GLOBAL task graph at execution time: how many
            predecessors/successors it has, whether it was a bottleneck
            (>=2 successors), whether it was a source/leaf, and how many
            of its successors were, at that moment, already
            done/ready/blocked -- a proxy for "how much did finishing this
            unblock, right now".

        global_graph_G is the supervisor's networkx DiGraph (the same `G`
        used throughout Supervisor.py / Agent.global_graph.G), so this
        works the same whether called from the supervisor's own agent or
        from a regular team member (every agent holds a read-only copy of
        the full DAG per Agent.load_goal()).
        """
        self._exec_order_counters[agent_id] = self._exec_order_counters.get(agent_id, 0) + 1
        order_index = self._exec_order_counters[agent_id]

        predecessors = list(global_graph_G.predecessors(task_id)) if global_graph_G.has_node(task_id) else []
        successors = list(global_graph_G.successors(task_id)) if global_graph_G.has_node(task_id) else []

        def node_status_value(n):
            assignment = global_graph_G.nodes[n].get("assignment")
            return self._status_value(assignment)

        successor_statuses = {s: node_status_value(s) for s in successors}
        newly_unblockable = sum(
            1 for s in successors
            if all(
                node_status_value(p) in ("done", "DONE")
                for p in global_graph_G.predecessors(s)
            )
        )

        row = {
            "run_id": self.run_id,
            "agent_id": agent_id,
            "task_id": task_id,
            "execution_order": order_index,
            "num_predecessors": len(predecessors),
            "num_successors": len(successors),
            "predecessors": self._as_array(map(str, predecessors)),
            "successors": self._as_array(map(str, successors)),
            "is_bottleneck": len(successors) >= 2,
            "is_source": len(predecessors) == 0,
            "is_leaf": len(successors) == 0,
            "successor_statuses": json.dumps(successor_statuses),
            "successors_unblocked_by_this": newly_unblockable,
            "timestamp": time.time(),
        }
        if extra_fields:
            row.update(extra_fields)

        self._write_rows(self._task_execution_path, fieldnames=list(row.keys()), rows=[row])
        return row

    # ------------------------------------------------------------------
    # Runtime idle-agent detection
    # ------------------------------------------------------------------
    # Statuses that count as "this agent currently has something to do".
    # READY = queued and pickable right now; RUNNING = actively executing.
    # ASSIGNED-but-not-yet-READY (still blocked on a predecessor) does NOT
    # count as busy -- the agent genuinely has nothing to *do* yet, which
    # is the whole point of detecting idleness.
    _BUSY_STATUSES = {"ready", "running"}

    def check_and_log_idle_agents(self, G, trigger_task_id=None, trigger_agent_id=None, debug=False):
        """
        Runtime idle check: call this every time the supervisor's global
        graph status changes (i.e. from inside
        Agent.update_task_status_received_supervisor, right after
        self.global_graph.update_status(...) runs), so it always sees a
        fresh picture of who is doing what.

        An agent is "idle" at this instant if:
          - it holds at least one non-end-goal task overall (so agents
            with zero tasks for the whole run are an *allocation* problem,
            already covered by agent_workload.csv's is_idle, not a
            *runtime* one -- nothing to report here), AND
          - none of its tasks are currently READY or RUNNING, AND
          - at least one OTHER agent currently has a task that IS READY or
            RUNNING (no point flagging idleness if literally nobody has
            anything to do, e.g. everything is still blocked on
            predecessors).

        The is_end_goal task is excluded entirely from this check (from
        both "is this agent busy" and "do other agents have something to
        do"), since it's a bookkeeping/supervision node rather than
        operational work.

        Appends one row per detected idle agent to idle_events.csv, with
        the idle agent's id and a full snapshot of every agent's current
        tasks + statuses at that instant, so later analysis can see
        exactly what the rest of the team was doing while this agent had
        nothing to do.

        Returns the list of agent_ids found idle at this call (empty list
        if none).
        """
        # Build {agent_id: [(task_id, status), ...]} excluding the
        # end-goal task, from the live assignment data on G.
        agent_tasks = {}
        for node_id in G.nodes:
            node_data = G.nodes[node_id]
            if node_data.get("is_end_goal", False):
                continue
            assignment = node_data.get("assignment")
            agent_id = getattr(assignment, "selected_agent", None) if assignment else None
            if agent_id is None:
                continue
            status = (self._status_value(assignment) or "").lower()
            agent_tasks.setdefault(agent_id, []).append((node_id, status))

        if len(agent_tasks) < 2:
            return []  # nothing to compare against

        def is_busy(tasks):
            return any(status in self._BUSY_STATUSES for _task_id, status in tasks)

        busy_agents = {aid for aid, tasks in agent_tasks.items() if is_busy(tasks)}

        # No one has anything ready/running yet (e.g. everything still
        # blocked on predecessors) -- not idleness, just an early phase.
        if not busy_agents:
            return []

        idle_agent_ids = [aid for aid, tasks in agent_tasks.items() if aid not in busy_agents]

        if not idle_agent_ids:
            return []

        # Snapshot of every agent's tasks/statuses, shared across all rows
        # emitted by this call so the "what was everyone else doing" view
        # is consistent for every idle agent flagged at this same instant.
        team_snapshot = {
            aid: [{"task_id": t, "status": s} for t, s in tasks]
            for aid, tasks in agent_tasks.items()
        }

        rows = []
        for idle_agent_id in idle_agent_ids:
            self._idle_event_counter += 1
            rows.append({
                "run_id": self.run_id,
                "idle_event_seq": self._idle_event_counter,
                "idle_agent_id": idle_agent_id,
                "idle_agent_tasks": self._as_array(
                    f"{t}:{s}" for t, s in agent_tasks[idle_agent_id]
                ),
                "busy_agent_ids": self._as_array(sorted(busy_agents)),
                "num_busy_agents": len(busy_agents),
                "num_idle_agents_this_event": len(idle_agent_ids),
                "team_snapshot": json.dumps(team_snapshot),
                "trigger_task_id": trigger_task_id,
                "trigger_agent_id": trigger_agent_id,
                "timestamp": time.time(),
            })

        self._write_rows(self._idle_events_path, fieldnames=list(rows[0].keys()), rows=rows)

        if debug:
            print(f"[ExperimentLogger] Idle agent(s) detected: {idle_agent_ids} "
                  f"while busy: {sorted(busy_agents)}")

        return idle_agent_ids
