# Tables for Paper

All values pulled directly from `baseline_contested/` and the 19 generated experiment configs, cross-checked against `validate_experiments.py`. Tables 1, 2, 3, and 7 are **verified eligibility facts** (workspace/skill gate outcomes, directly computed from the configs — the same logic the real `Supervisor.py` runs). Tables 4, 4b, and 6 are **derived predictions** of which agent *wins* a scoring contest, computed from `Supervisor.py`'s actual confirmed scoring formula (read directly from the code) but not from an executed run — see the note before Table 4b for exactly what is and isn't verified.

---

## Table 1 — Agent Skill & Preference Profile (`HouseParty` context)

Preference is an integer **rank** among each agent's own skills (1 = most preferred), per `Supervisor.py`'s documented formula `preference_score = (N − rank + 1) / N`. A "—" means the agent does not have that skill in this context (treated as level 0.0 by the eligibility gate).

| Skill | fr3_arm_1 | fr3_arm_2 | tiago_robot_1 | human_host |
|---|---|---|---|---|
| **manipulation** | 1.00 (rank 1) | 0.85 (rank 2) | 0.80 (rank 2) | 0.80 (rank 4) |
| **supervise** | 0.55 (rank 2) | 0.20 (rank 3) | 0.60 (rank 4) | 0.80 (rank 2) |
| **chopping** | 0.45 (rank 3) | 0.90 (rank 1) | 0.30 (rank 7) | 0.40 (rank 7) |
| **grill** | 1.00 (rank 4) | — | 0.55 (rank 5) | — |
| **transport** | — | — | 1.00 (rank 1) | 1.00 (rank 3) |
| **communication** | — | — | 0.75 (rank 3) | 1.00 (rank 1) |
| **openDoor** | — | — | 0.50 (rank 6) | 1.00 (rank 5) |
| **cook** | — | — | — | 0.90 (rank 6) |
| *N (skills ranked)* | *4* | *3* | *7* | *7* |

**Design intent per skill** (each is a deliberate choice, not incidental):
- *manipulation*: fr3_arm_1 (1.00) > fr3_arm_2 (0.85) > host (0.80) = tiago (0.80, tied). **host and tiago are deliberately tied here.** host's manipulation was originally set to 0.90 — higher than *both* specialist robot arms — which was a real design bug: in `skill` mode (highest raw level wins), host won every manipulation-gated contested task purely on this number, regardless of which robot the task was meant to showcase. Lowering host to exactly 0.80 (tied with tiago, still below fr3_arm_2's 0.85) restores fr3_arm_2 as the clean, unambiguous skill-mode winner on every single-skill Pick*/Chop* task. On multi-skill tasks where host and tiago are *also* tied on transport (1.00 = 1.00) — the Serve*/Dishes* family — the tie now genuinely resolves via `get_selected_agent`'s workload tie-break rather than either agent's raw skill (see Tables 3 and 4).
- *grill*: **deliberately engineered to diverge skill mode from preference mode.** fr3_arm_1 has the higher raw LEVEL (1.00 vs. tiago's 0.55), so `skill` mode picks fr3_arm_1 on the grill chain. But fr3_arm_1 ranks grill LAST among its own 4 skills (rank 4, its lowest preference), while tiago ranks it 5th of 7 — high enough that tiago's *combined* preference score on grill-chain tasks (manipulation + grill) edges past fr3_arm_1's, so `preference` mode picks tiago instead. Without this rank choice, fr3_arm_1 would dominate the grill chain on both axes, leaving no mode divergence there (see Table 4b).
- *communication*: host is **both** the skill leader (1.00) **and** its single strongest preference (rank 1 of 7) — it is the agent expected to welcome guests, so this preference is not allowed to be displaced even by its own `supervise`/management preference. tiago clears the `WelcomeGuests` gate (0.75 ≥ 0.70) and can at best tie host's preference score, never exceed it (see Table 5).
- *supervise*: gates the is_end_goal task `BarbecueParty` (0.5 floor) and is added to **all four agents**, since any of them could in principle be designated supervisor in `exp1.yaml`. **host is the default designated supervisor** (as of this revision — previously fr3_arm_1) and clears the gate with a wide margin: level 0.80 vs. the 0.5 floor. host ranks `supervise` 2nd (just behind `communication`) — "the human has the stronger preference for manage" is interpreted as stronger than the *other agents'* supervise preference, not stronger than host's own expected welcoming role. fr3_arm_1 (0.55, rank 2 of 4) and tiago (0.6, rank 4 of 7) both clear the floor too and remain valid supervisor candidates by level if ever designated; fr3_arm_2 (0.2) does **not** clear it — see Table 7 / Set F for experiments that exploit this.
- *chopping*: fr3_arm_2 is the skill leader **and** ranks it 1st preference; fr3_arm_1's 0.45 is a fallback value, only relevant if its workspace were ever swapped onto `Prep_Table`.
- *transport*: tiago and host are tied at the skill level (1.00), but only tiago ranks it as a top preference (1st vs. host's 3rd).
- *cook*: host is the **only** agent with this skill at all — gates `CookRice`, which is host-solo by explicit design.

---

## Table 2 — Agent Workspace Access

| Agent | Mobility | Workspace(s) |
|---|---|---|
| fr3_arm_1 | Fixed-base | Grill_Table |
| fr3_arm_2 | Fixed-base | Prep_Table |
| tiago_robot_1 | Mobile | Prep_Table, Grill_Table, Drinks_Table, House, Garden, Kitchen_Table, Kitchen_Stove, Kitchen_sink, Kitchen_shelves *(all 9)* |
| human_host | Mobile | Prep_Table, Grill_Table, Drinks_Table, House, Garden, Kitchen_Table, Kitchen_Stove, Kitchen_sink, Kitchen_shelves *(all 9)* |

**Constraint:** fr3_arm_1 and fr3_arm_2 never share a workspace in any experiment (verified programmatically across all 19 configs) — they are fixed-base arms with no physical room to occupy the same table.

---

## Table 3 — Task Eligibility Summary, All 19 Experiments

`barbecue.json` has 22 tasks total: 21 ordinary tasks plus 1 is_end_goal task (`BarbecueParty`, handled separately — see Table 7). All counts below are over the 21 ordinary tasks only. Experiments now live in per-set subfolders (e.g. `Set_A_Mode_Sweep/exp_mode_skill/`); the Set column below corresponds to those folder names.

"Contested" and "Solo" both count **tasks**, not agents: for each of the 21 tasks, count how many agents are eligible (workspace + skill gate cleared); "Contested" = that count is ≥2, "Solo" = that count is exactly 1. So "Solo: 1" means *1 task* has only *1* eligible agent — not that 1 agent shares one task. Which task(s) fall into Solo, and which single agent is left, is given in the footnote below the table. Computed by `validate_experiments.py` against each experiment's actual generated files; zero unassigned tasks in every case. A second, separate check confirms whether the designated supervisor (`host` by default as of this revision — previously `fr3_arm_1`) passes the is_end_goal task `BarbecueParty`'s `required_skills` (`manipulation ≥ 0.4`, `supervise ≥ 0.5`) — this mirrors a new method, `Supervisor._check_supervisor_eligibility`, added so an unqualified supervisor triggers the same shutdown path as an unassigned task, rather than being silently waved through (the original `score_agents_for_task` never checked `required_skills` on end-goal tasks at all). Set F (new) deliberately designates a *different* supervisor or lowers the supervisor's own level, to exercise this check's failure path directly.

| Set | Experiment | optimizeMode | Contested | Solo | Unassigned | Supervisor eligible? |
|---|---|---|---|---|---|---|
| A | exp_mode_skill | skill | 20 | 1¹ | 0 | Yes |
| A | exp_mode_preference | preference | 20 | 1¹ | 0 | Yes |
| A | exp_mode_balance_skill_preference | balance_skill_preference | 20 | 1¹ | 0 | Yes |
| A | exp_mode_balance_workload | balance_workload | 20 | 1¹ | 0 | Yes |
| B | exp_skill_low | skill | 20 | 1¹ | 0 | Yes |
| B | exp_skill_mid | skill | 20 | 1¹ | 0 | Yes |
| B | exp_skill_high | skill | 20 | 1¹ | 0 | Yes |
| C | exp_pref_baseline_order | preference | 20 | 1¹ | 0 | Yes |
| C | exp_pref_inverted | preference | 20 | 1¹ | 0 | Yes |
| C | exp_pref_communication_top | preference | 20 | 1¹ | 0 | Yes |
| D | exp_workspace_baseline | balance_workload | 20 | 1¹ | 0 | Yes |
| D | exp_workspace_tiago_no_grill_no_prep | balance_workload | 13 | 8² | 0 | Yes |
| D | exp_workspace_tiago_drinks_only | balance_workload | 6 | 15³ | 0 | Yes |
| E | exp_interaction_baseline | balance_skill_preference | 20 | 1¹ | 0 | Yes |
| E | exp_interaction_low_skill_high_pref | balance_skill_preference | 20 | 1¹ | 0 | Yes |
| E | exp_interaction_equal_skill_diff_pref | balance_skill_preference | 20 | 1¹ | 0 | Yes |
| F | exp_supervisor_host | balance_workload | 20 | 1¹ | 0 | Yes (wide margin) |
| F | exp_supervisor_fr3_arm2_fails | balance_workload | 20 | 1¹ | 0 | **No** (expected) |
| F | exp_supervisor_threshold_fail | balance_workload | 20 | 1¹ | 0 | **No** (expected) |

**Footnotes — exactly which task(s) are Solo, and to which single agent:**
1. **`CookRice`** → `host` (sole `cook`-capable agent). This is the only Solo task in 17 of the 19 experiments — none of Sets A/B/C/E/F touch the workspace or skill values that would change it (Set F only touches the *supervisor* and the supervisor's own `supervise` value, neither of which affects this task). `ServeMainDish` was *also* Solo in an earlier revision of this baseline (when host's manipulation level was 0.9), but became a genuine tiago-vs-host contest once host's manipulation bug was fixed (see Table 1) and `ServeMainDish`'s own requirement was correspondingly lowered.
2. `exp_workspace_tiago_no_grill_no_prep` adds 7 more Solo tasks on top of the baseline one: **`ServeSalad`**, **`ServeSides`** → `host` (need `Prep_Table`+`Garden`, tiago lost `Prep_Table`); **`PickFoodIngredientsGrill`**, **`GrillFood`**, **`PickGrilledFood`** → `fr3_arm_1` (need `Grill_Table`, tiago lost it); **`ServeGrilledFood`** → `host` (needs `Grill_Table`+`Garden`); **`Host`** → `host` (needs all five of Grill_Table/Garden/House/Drinks_Table/Prep_Table, tiago no longer qualifies). Total: 8. (`ServeMainDish` does NOT appear here — it needs `Kitchen_Stove`+`Garden`, neither of which this experiment touches, so it remains a tiago-vs-host contest.)
3. `exp_workspace_tiago_drinks_only` adds 14 more Solo tasks on top of the baseline one (tiago restricted to `Drinks_Table` only, so it loses every contest it was in): **`WelcomeGuests`**, **`ServeSalad`**, **`ServeDrinks`**, **`ServeSides`**, **`PickRice`**, **`ServeMainDish`**, **`ServeGrilledFood`**, **`Host`**, **`PickDishes`**, **`DoTheDishes`**, **`PutTheDishesAway`** → all `host`; **`PickFoodIngredientsGrill`**, **`GrillFood`**, **`PickGrilledFood`** → all `fr3_arm_1`. Total: 15.

Sets A/B/C/E hold workspace fixed, so contested/solo counts don't move — the *winner* within each contested task is what changes (Tables 4–6). Set D is the one factor that directly changes contest *eligibility*: removing tiago's access to Grill_Table+Prep_Table drops 7 tasks out of contention (20→13); restricting it to Drinks_Table only drops 14 (20→6). Set F changes neither Contested nor Solo (the 21 ordinary tasks are completely unaffected by who is supervisor) — it changes only the **Supervisor eligible?** column, which is the entire point: it isolates the end-goal gate from everything else in the system. See Table 7 for the full breakdown of Set F's two intentional failures.

---

## A note before Tables 4–4b–6: predicted vs. measured

Tables 4, 4b, and 6 show **predicted winners**, derived analytically from `Supervisor.py`'s actual, now-confirmed scoring formula (read directly from `score_agents_for_task`'s scoring loop):
- *skill mode*: `score = Σ level` — the SUM of the agent's skill level across every skill the task requires (not an average or max).
- *preference mode*: `score = Σ (N − rank + 1) / N` — the SUM of each skill's normalized preference score across every skill the task requires.
- *balance_skill_preference mode*: `score = Σ [α·level + (1−α)·pref_score]`, where `α = BALANCE_SKILL_PREFERENCE_ALPHA` (0.7 per the docstring referenced in Table 6).

This is a real formula read from the code, not a guess — but two things remain genuinely unverified:
1. **These predictions were never executed against the real running `Supervisor.py`** — only the eligibility gate (`skill_ok`, workspace subset check) was exercised directly, via `validate_experiments.py`. The scoring arithmetic above was hand-computed against the same formula the code uses, and cross-checked for consistency, but not run through the live scheduler.
2. **Ties are NOT resolved by this analysis.** Several tasks (the 9-task Serve*/Dishes* family — see Table 4b) are genuine ties under `skill` mode; the real winner there depends on `get_selected_agent`'s workload tie-break (fewest tasks already assigned, then ranking order — read directly from the code, but its *outcome* depends on task processing order within a run, which is outside any single file available here).

Running the actual 19 configs through your live scheduler and replacing Tables 4, 4b, and 6 with real output is the natural next step before publication, particularly to confirm the tie-break outcomes on the Serve*/Dishes* family.

---

## Table 4 — Set B: Skill-Mode Winner on the Pick* Contest (fr3_arm_2 manipulation sweep)

Task shown: `PickSaladIngredients` (manipulation ≥ 0.6), representative of all four Pick* salad/vegetable tasks, which share the same eligible pool and requirement.

| Experiment | fr3_arm_2 level | Eligible agents | Skill scores | Predicted skill-mode winner |
|---|---|---|---|---|
| exp_skill_low | 0.55 | tiago, host | tiago 0.80, host 0.80 | **tie** (tiago vs. host — resolved by workload tie-break, not skill; fr3_arm_2 excluded — below 0.6 gate) |
| exp_skill_mid | 0.60 | fr3_arm_2, tiago, host | fr3_arm_2 0.60, tiago 0.80, host 0.80 | **tie** (tiago vs. host; fr3_arm_2 is now eligible but is the clear loser at 0.60) |
| exp_skill_high | 1.00 | fr3_arm_2, tiago, host | fr3_arm_2 1.00, tiago 0.80, host 0.80 | **fr3_arm_2** (clean win — only agent at the ceiling) |

The sweep demonstrates the eligibility boundary (exp_skill_low) and fr3_arm_2's transition from absent, to present-but-tied-out-of-the-running (exp_skill_mid, where it's eligible but loses outright to the 0.80 tie), to the clean skill-mode leader (exp_skill_high, the only agent at the ceiling) — all without touching any other agent's configuration. Note that in exp_skill_low and exp_skill_mid, the actual winner between tiago and host is NOT determined by this sweep at all (they're tied at 0.80 throughout) — it falls to `get_selected_agent`'s workload tie-break, which is outside what this table predicts.

---

## Table 4b — Where `skill` Mode and `preference` Mode Pick Different Winners (Full Map)

This is the direct answer to "does task allocation actually differ by optimize mode": across all 20 contested tasks in `baseline_contested` (computed with `Supervisor.py`'s real formula — `score = Σ level` for skill mode, `score = Σ (N−rank+1)/N` for preference mode, summed across each task's required skills, not averaged or maxed), **7 of 20 (35%) pick a different winner depending on mode**, with no tie ambiguity on either side of any diverging row.

| Task | `skill` mode winner | `preference` mode winner | Diverges? |
|---|---|---|---|
| WelcomeGuests | host | host | No |
| **PickSaladIngredients** | fr3_arm_2 | **tiago_robot_1** | **Yes** |
| ChopSaladIngredients | fr3_arm_2 | fr3_arm_2 | No |
| **PickChoppedSaladIngredients** | fr3_arm_2 | **tiago_robot_1** | **Yes** |
| ServeSalad | tiago_robot_1 (skill tie w/ host) | tiago_robot_1 | No |
| ServeDrinks | tiago_robot_1 (skill tie w/ host) | tiago_robot_1 | No |
| **PickVegetables** | fr3_arm_2 | **tiago_robot_1** | **Yes** |
| ChopVegetables | fr3_arm_2 | fr3_arm_2 | No |
| **PickChoppedVegetables** | fr3_arm_2 | **tiago_robot_1** | **Yes** |
| ServeSides | tiago_robot_1 (skill tie w/ host) | tiago_robot_1 | No |
| PickRice | tiago_robot_1 (skill tie w/ host) | tiago_robot_1 | No |
| ServeMainDish | tiago_robot_1 (skill tie w/ host) | tiago_robot_1 | No |
| **PickFoodIngredientsGrill** | fr3_arm_1 | **tiago_robot_1** | **Yes** |
| **GrillFood** | fr3_arm_1 | **tiago_robot_1** | **Yes** |
| **PickGrilledFood** | fr3_arm_1 | **tiago_robot_1** | **Yes** |
| ServeGrilledFood | tiago_robot_1 (skill tie w/ host) | tiago_robot_1 | No |
| Host | host | host | No |
| PickDishes | tiago_robot_1 (skill tie w/ host) | tiago_robot_1 | No |
| DoTheDishes | tiago_robot_1 (skill tie w/ host) | tiago_robot_1 | No |
| PutTheDishesAway | tiago_robot_1 (skill tie w/ host) | tiago_robot_1 | No |

**The two diverging clusters, and why each was engineered:**
- **Pick* salad/vegetable tasks (4 tasks):** `skill` mode picks `fr3_arm_2` — it has the highest raw `manipulation` level among eligible agents (0.85 vs. tiago/host's 0.80). `preference` mode picks `tiago_robot_1` instead — tiago ranks `manipulation` 2nd of its own 7 skills (pref_score 6/7 ≈ 0.86), which beats fr3_arm_2 ranking it 2nd of only 3 skills (pref_score 2/3 ≈ 0.67), since the *normalized* rank position, not the raw level, is what preference mode scores.
- **Grill chain (3 tasks):** `skill` mode picks `fr3_arm_1` — its raw `manipulation`+`grill` level sum (2.0) dwarfs tiago's (1.35). `preference` mode picks `tiago_robot_1` instead, by design: fr3_arm_1 ranks `grill` dead last among its own 4 skills (its lowest preference, despite being its strongest *skill*), pulling its preference sum down to 1.25, just below tiago's 1.29 (grill ranked 5th of 7, but manipulation still 2nd of 7).

**Why the other 13 tasks don't diverge:** `WelcomeGuests`/`Host` are deliberately undivergeable — host is both skill leader and top preference on `communication`, by design (host is "expected to welcome guests"). The 9 Serve*/Dishes* tasks tie on `skill` (tiago and host are tied on both `manipulation` and `transport`), so they aren't a skill-vs-preference divergence at all — they're a *skill-mode ambiguity* resolved by workload tie-break, with `preference` mode breaking the same tie cleanly in tiago's favor (its transport+manipulation preference sum beats host's).

---

## Table 5 — Set C: Preference-Mode Score for `WelcomeGuests` (tiago rank sweep)

Task shown: `WelcomeGuests` (communication ≥ 0.7, openDoor ≥ 0.3). Tiago's skill **levels** are identical across all three rows — only its preference **rank** for communication changes. Host's communication rank is fixed at **1st of 7** throughout this set — by design, host is both the skill leader on communication and its single strongest preference, since it is the agent expected to welcome guests, and that preference is not allowed to be displaced even by its own `supervise`/management preference (see Table 1).

| Experiment | Tiago communication rank | Tiago pref_score | Host pref_score (rank 1/7, fixed) | Predicted preference-mode winner |
|---|---|---|---|---|
| exp_pref_baseline_order | 3rd of 7 | 0.71 | 1.00 | **host** |
| exp_pref_inverted | 5th of 7 | 0.43 | 1.00 | **host** (margin widens) |
| exp_pref_communication_top | 1st of 7 (most preferred) | **1.00** | 1.00 | **tie** (resolved by workload tie-break) |

This isolates a true preference-only effect: tiago's communication *skill level* (0.75) never changes, yet its competitiveness against the host shifts purely from re-ranking. Because host's communication preference is deliberately set as its unshakeable top rank, tiago can close the gap all the way to a tie at best — it can never out-rank the host on this specific task by preference alone, which is the intended behavior: the host should not lose the welcoming role to a re-ranking trick on another agent's preferences.

---

## Table 6 — Set E: `balance_skill_preference` — Can Preference Rescue a Skill Failure?

Task shown: `PickSaladIngredients` (manipulation ≥ 0.6). fr3_arm_2's preference rank for manipulation is promoted to 1st of 3 (from 2nd of 3) while its skill level is dropped below the eligibility gate. (fr3_arm_2 has 3 ranked skills as of this revision — manipulation, chopping, and the newly-added `supervise` — not 2.)

| Experiment | fr3_arm_2 manipulation level | fr3_arm_2 manipulation rank | Eligible for this task? | Outcome |
|---|---|---|---|---|
| exp_interaction_baseline | 0.85 | 2nd of 3 | Yes | Contests tiago/host normally |
| exp_interaction_low_skill_high_pref | 0.55 | **1st of 3** (promoted) | **No** — fails `skill_ok` (0.55 < 0.6) | Excluded regardless of preference |

This is the key negative result for the paper's architecture discussion: `skill_ok` is a hard precondition checked *before* any scoring runs (skill, preference, or blended), so no amount of preference can substitute for failing the minimum skill bar. `balance_skill_preference` only ever re-ranks *among* already-eligible agents.

| Experiment | Host communication rank | Tiago communication rank | Both skill level | Predicted `WelcomeGuests`/`Host` winner |
|---|---|---|---|---|
| exp_interaction_equal_skill_diff_pref | 7th of 7 (demoted) | 1st of 7 (promoted) | 0.80 (tied) | **tiago** — with skill tied exactly, the preference term is the only tie-breaker left, and it now favors tiago |

This second row is the positive counterpart: once skill is equalized, the preference component of the blended score *does* change the outcome — confirming the 0.7/0.3 skill/preference blend has real, non-trivial weight rather than being dominated by skill in every case. Note this experiment's reordering pushes host's `supervise` preference **up** to rank 1 of 7 (from its baseline rank 2) as an unavoidable side effect of demoting `communication` all the way to last — host's `supervise` *skill level* (0.8) is unchanged, and host *is* the actual designated supervisor (as of this revision), so `BarbecueParty`'s gate still checks `supervise`'s LEVEL, not its rank — the gate is unaffected by this experiment regardless. But it is a reminder that this experiment is deliberately adversarial to the host's communication preference specifically, not a "natural" configuration — Table 5 shows the host cannot be beaten on `WelcomeGuests` preference under any of the *intended* Set C variants; this row only flips the outcome by artificially crushing host's communication rank to the bottom of its own list.

---

## Table 7 — Supervisor Self-Eligibility Check (Set F, Architecture Result)

Unlike Tables 4–6, this is not a comparison between candidate agents for a task — it is a single structural safeguard: does the agent designated `supervisor:` in `exp1.yaml` qualify for the is_end_goal task `BarbecueParty` at all? This was previously **unenforceable**: the original `score_agents_for_task`'s is_end_goal branch unconditionally scored the supervisor 1.0 regardless of skill, so `BarbecueParty`'s `required_skills` were dead code for that branch. A new method, `Supervisor._check_supervisor_eligibility`, closes this gap by running before any scoring starts. Set F's three experiments (`Set_F_Supervisor_Eligibility_Sweep/`) exercise both the pass and fail paths directly, rather than leaving the fail path purely hypothetical.

| Experiment | Supervisor | `manipulation` (≥0.4) | `supervise` (≥0.5) | Result |
|---|---|---|---|---|
| exp_supervisor_host | host | 0.80 — Pass | 0.80 — Pass | **Eligible** (margin: +0.40 / +0.30) |
| exp_supervisor_fr3_arm2_fails | fr3_arm_2 | 0.85 — Pass | **0.20 — Fail** | **Shutdown** — wrong agent designated |
| exp_supervisor_threshold_fail | host | 0.80 — Pass | **0.45 — Fail** | **Shutdown** — right agent, level dropped below floor |

These two failures isolate two structurally different causes of the same outcome:
- **`exp_supervisor_fr3_arm2_fails`** changes *who* is supervisor (host → fr3_arm_2) with zero other edits. fr3_arm_2's `supervise` level was already 0.2 in `baseline_contested` (its lowest-ranked, least-preferred skill) — this experiment needed no new values, only a different `supervisor:` field, to demonstrate the gate rejecting an unqualified candidate.
- **`exp_supervisor_threshold_fail`** keeps the *right* agent (host) as supervisor but drops its own `supervise` level from 0.8 to 0.45 — just under the 0.5 floor. This isolates the level threshold itself, independent of agent identity: the same agent that passes in `exp_supervisor_host` fails here purely because one number crossed a line.

Both failures trigger the identical code path: `Supervisor._check_supervisor_eligibility` returns `False`, which causes `assign_agents_to_tasks` to call `self.publish("shutdown", {"reason": "supervisor_unqualified", ...})` followed by `self.agent.handle_shutdown("supervisor_unqualified", missing)` — the exact same shutdown mechanism already used for `unassigned_tasks`, just gated on a different precondition. `validate_experiments.py` labels these two rows `EXPECTED-FAIL` rather than `FAIL`, since the failure is the demonstration, not a defect.

**Note on `BarbecueParty`'s requirements:** its original `transport ≥ 0.4` and `communication ≥ 0.2` requirements were **removed** earlier in this design's history, back when `fr3_arm_1` was still the default supervisor and had neither skill — leaving them in place would have shut down every experiment immediately once the check started being enforced. They remain removed now that `host` is the default supervisor; host's elevated `manipulation`/`supervise` requirements are sufficient on their own to make the gate meaningful (Set F's two failures above prove the gate still bites even with the reduced requirement set).

**Why this belongs in the paper as its own result:** it demonstrates a different failure mode than `skill_ok` filtering out a *candidate* (Table 6) — here, an unqualified *supervisor* is structurally incapable of completing the run at all, regardless of mode, regardless of how well every other agent is matched to its tasks. Unlike a contested-task failure (which just excludes one candidate from one task), a supervisor-eligibility failure halts the entire allocation before it starts.

---

## Suggested placement in the paper

- **Table 1 + Table 2** -> Experimental Setup / System Configuration section, as the canonical description of the test environment.
- **Table 3** -> opens the Results section, establishing how many tasks each experiment actually puts into contention before discussing *who* wins.
- **Table 4b** -> the single best table to answer "does optimize mode actually change task allocation" directly — lead with this if the paper needs one headline table for that claim (7 of 20 contested tasks, 35%, pick a different winner depending on mode).
- **Tables 4-6** -> one per relevant Results subsection (Skill Sensitivity, Preference Sensitivity, Mode Interaction), each introduced by 2-3 sentences of the same explanation already drafted in `MANIFEST.md`.
- **Table 7** -> Architecture / Robustness subsection, paired with a short description of `_check_supervisor_eligibility`, the rationale for removing `BarbecueParty`'s transport/communication requirements, and the two distinct failure modes (wrong agent vs. right agent below threshold) that Set F demonstrates.
