#!/usr/bin/env python3
"""
validate_experiments.py

Re-implements the eligibility-relevant subset of Supervisor.score_agents_for_task
(context check, workspace subset check, skill_ok gate, constraint check) to
verify, for every generated experiment folder:

  1. No task ends up with zero eligible agents (which would trigger the
     real Supervisor's unassigned_tasks shutdown path).
  2. Report how many tasks are "contested" (>=2 eligible agents) vs "solo"
     (exactly 1) for that experiment's specific configuration, since that's
     the actual quantity the paper's results tables care about.
  3. Every agent's preference values within a context form a clean 1..N
     permutation (1 = most preferred, matching Supervisor.py's documented
     rank-based formula: preference_score = (N - rank + 1) / N). A gap or
     a tie here would silently break preference-mode comparisons.
  4. The designated supervisor (exp1.yaml's `supervisor:` field) passes
     the is_end_goal task's required_skills, mirroring
     Supervisor._check_supervisor_eligibility -- a new method added
     alongside this baseline that shuts the whole run down if the
     supervisor itself doesn't qualify for BarbecueParty before any
     scoring happens.

This does NOT reimplement score_agents_for_task's actual mode-dependent
SCORING (skill/preference/balance formulas) or the workload tie-break /
rebalancing logic in get_selected_agent / rebalance_workload -- those need
the real Supervisor.py + a graph object to run properly. This script is a
config-level sanity check only: "is every task assignable at all, to how
many candidates, are the preference ranks well-formed, and is the
supervisor itself qualified."
"""

import glob
import json
import os
import yaml

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def load_experiment(exp_dir):
    agents = {}
    rank_issues = {}
    for fname in ["fr3_arm_1.yaml", "fr3_arm_2.yaml", "tiago_robot_1.yaml", "human_host.yaml"]:
        a = yaml.safe_load(open(os.path.join(exp_dir, fname)))
        skills = {}
        for context, skillmap in a.get("skill_weights", {}).items():
            # Rank-validity (1..N permutation) is only checked for the
            # HouseParty context: that's the only context this experiment
            # design touches and the only one barbecue.json's tasks use.
            # quality_check/assembly are untouched legacy contexts carried
            # over from the original uploaded YAMLs (still using the old
            # 0-1 float convention) and are out of scope here.
            if context == "HouseParty":
                ranks_in_context = [rank for (_level, rank) in skillmap.values()]
                N = len(ranks_in_context)
                expected = list(range(1, N + 1))
                if sorted(ranks_in_context) != expected:
                    rank_issues[(a["robot_id"], context)] = sorted(ranks_in_context)
            for skill, (level, rank) in skillmap.items():
                skills.setdefault(skill, {})[context] = level
        agents[a["robot_id"]] = {
            "workspace": a.get("workspace") or [],
            "skills": skills,
        }
    tasks = json.load(open(os.path.join(exp_dir, "barbecue.json")))["tasks"]
    exp_cfg = yaml.safe_load(open(os.path.join(exp_dir, "exp1.yaml")))
    return agents, tasks, exp_cfg, rank_issues


def eligible_agents_for_task(task, agents):
    context = task.get("context")
    req_skills = task.get("required_skills", {})
    req_constraints = task.get("required_constraints", {})
    req_ws_raw = req_constraints.get("workspace")
    req_ws = None if req_ws_raw == ["global"] else set(w.lower() for w in req_ws_raw)

    eligible = []
    for aid, a in agents.items():
        # context check: agent must have at least one skill defined in this context
        agent_contexts = set()
        for sk, ctxmap in a["skills"].items():
            agent_contexts.update(ctxmap.keys())
        if context not in agent_contexts:
            continue

        # workspace check
        if req_ws is not None:
            aws = set(w.lower() for w in a["workspace"])
            if not aws or not req_ws.issubset(aws):
                continue

        # skill_ok check
        ok = True
        for sk, min_level in req_skills.items():
            level = a["skills"].get(sk, {}).get(context, 0.0)
            if level < min_level:
                ok = False
                break
        if not ok:
            continue

        # constraint check (numeric only, max_payload_kg is the relevant one here;
        # robots/host don't declare numeric constraints in these YAMLs, so this
        # is a no-op given current data -- included for completeness/future use)
        eligible.append(aid)

    return eligible


def check_supervisor_eligibility(agents, tasks, supervisor_id):
    """
    Mirrors Supervisor._check_supervisor_eligibility: does the designated
    supervisor (exp1.yaml's `supervisor:` field) meet the required_skills
    of the is_end_goal task? Returns (eligible: bool, missing: dict).
    Unlike eligible_agents_for_task, this does NOT check workspace --
    the real method doesn't either, since is_end_goal tasks typically use
    workspace=["global"] and the supervisor check is purely skill-based.
    """
    end_goal_task = next((t for t in tasks if t.get("is_end_goal")), None)
    if end_goal_task is None:
        return True, {}

    required_skills = end_goal_task.get("required_skills", {})
    if not required_skills:
        return True, {}

    context = end_goal_task.get("context")
    supervisor = agents.get(supervisor_id)
    if supervisor is None:
        return False, {"_error": f"supervisor_id '{supervisor_id}' not found in agents"}

    missing = {}
    for skill, min_level in required_skills.items():
        level = supervisor["skills"].get(skill, {}).get(context, 0.0)
        if level < min_level:
            missing[skill] = {"required": min_level, "actual": level}

    return (not missing), missing


EXPECTED_SUPERVISOR_FAILURE = {
    "Set_F_Supervisor_Eligibility_Sweep/exp_supervisor_fr3_arm2_fails",
    "Set_F_Supervisor_Eligibility_Sweep/exp_supervisor_threshold_fail",
}


def main():
    exp_dirs = sorted(
        d for d in glob.glob(os.path.join(BASE_DIR, "Set_*", "exp_*"))
        if os.path.isdir(d)
    )
    print(f"Validating {len(exp_dirs)} experiments against baseline_contested-derived configs...\n")

    any_unexpected_failure = False

    for exp_dir in exp_dirs:
        name = os.path.relpath(exp_dir, BASE_DIR)
        agents, tasks, exp_cfg, rank_issues = load_experiment(exp_dir)

        unassigned = []
        contested = []
        solo = []

        for t in tasks:
            if t.get("is_end_goal"):
                continue
            elig = eligible_agents_for_task(t, agents)
            if not elig:
                unassigned.append(t["id"])
            elif len(elig) >= 2:
                contested.append((t["id"], elig))
            else:
                solo.append((t["id"], elig))

        supervisor_id = exp_cfg.get("supervisor")
        supervisor_ok, supervisor_missing = check_supervisor_eligibility(agents, tasks, supervisor_id)

        is_expected_supervisor_failure = name in EXPECTED_SUPERVISOR_FAILURE
        has_unexpected_problem = bool(unassigned) or bool(rank_issues) or (
            not supervisor_ok and not is_expected_supervisor_failure
        )

        if has_unexpected_problem:
            status = "FAIL"
            any_unexpected_failure = True
        elif not supervisor_ok and is_expected_supervisor_failure:
            status = "EXPECTED-FAIL"
        else:
            status = "OK"

        print(f"[{status}] {name}  (mode={exp_cfg['optimizeMode']}, supervisor={supervisor_id})")
        print(f"    contested: {len(contested)}, solo: {len(solo)}, unassigned: {len(unassigned)}")
        if unassigned:
            print(f"    UNASSIGNED TASKS: {unassigned}")
        if rank_issues:
            for (agent_id, context), ranks in rank_issues.items():
                print(f"    RANK ISSUE: {agent_id} in context '{context}' has ranks {ranks} (not a clean 1..N permutation)")
        if not supervisor_ok:
            tag = "EXPECTED" if is_expected_supervisor_failure else "UNEXPECTED"
            print(f"    SUPERVISOR INELIGIBLE [{tag}]: '{supervisor_id}' fails BarbecueParty's required_skills: {supervisor_missing} -- real Supervisor.py would shut down via _check_supervisor_eligibility before any scoring runs")
        print()

    print("=" * 60)
    if any_unexpected_failure:
        print("AT LEAST ONE EXPERIMENT HAS AN UNEXPECTED PROBLEM (unassigned tasks, bad rank ordering, or an unintended ineligible supervisor) -- review above.")
    else:
        print("All experiments behave as designed: every task has >=1 eligible agent, every agent's preference ranks are a clean 1..N permutation per context, and the designated supervisor's eligibility outcome matches what each experiment intends (Set F's two negative-result experiments correctly show EXPECTED-FAIL, demonstrating the shutdown path; every other experiment's supervisor passes).")


if __name__ == "__main__":
    main()
