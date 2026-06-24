# Experiment Manifest

All 19 experiments below are derived from a single baseline, `baseline_contested/`, by changing exactly one independent variable per experiment. Every experiment folder is a full, self-contained copy of the 5 config files (`exp1.yaml`, `fr3_arm_1.yaml`, `fr3_arm_2.yaml`, `tiago_robot_1.yaml`, `human_host.yaml`) plus `barbecue.json`. Everything not listed in an experiment's diff is byte-for-byte identical to `baseline_contested/`, so any difference in the resulting task assignment can be attributed to that one change.

**Preference format:** the second number in every `skill_weights.HouseParty.<skill>` entry is an INTEGER RANK among that agent's own skills in that context (1 = most preferred, 2 = second most preferred, etc — matching Supervisor.py's documented formula `preference_score = (N - rank + 1) / N`). It is NOT a 0-1 weight. `quality_check`/`assembly` contexts still use the original float convention from the uploaded files; they are untouched legacy data, out of scope for this design (no task in `barbecue.json` uses those contexts).

**Baseline design notes** (full rationale and exact rank/level table in `build_baseline_contested.py`):
- `fr3_arm_1` (Grill_Table only) and `fr3_arm_2` (Prep_Table only) are fixed-base arms that cannot physically share a workspace — there is no room for both at one table. Each keeps its own single workspace; there is no arm-vs-arm contest anywhere in this design, by physical necessity.
- `tiago_robot_1` and `human_host` are the only MOBILE agents and both have access to all 9 workspaces in the task graph (Prep_Table, Grill_Table, Drinks_Table, House, Garden, Kitchen_Table, Kitchen_Stove, Kitchen_sink, Kitchen_shelves), so either can be workspace-eligible for almost any task. In `baseline_contested`, tiago is itself eligible for 17 of 21 non-end-goal tasks (excluded only from ChopSaladIngredients/ChopVegetables, where its chopping skill level falls below the task floor, and CookRice, gated by the host-only `cook` skill; ServeMainDish remains gated above tiago's manipulation level), and 19 of 21 tasks overall have >=2 eligible agents once the relevant arm and/or host are counted.
- Four new skills were added beyond the original `manipulation`/`transport`/`communication`/`openDoor`: `chopping` (gates `ChopSaladIngredients`/`ChopVegetables`, alongside a lowered `manipulation` floor of 0.5), `grill` (gates all three grill-chain tasks, alongside a lowered `manipulation` floor of 0.5), `cook` (gates `CookRice`, alongside a lowered `manipulation` floor of 0.5 -- human_host is the ONLY cook-capable agent, so this task is host-solo by explicit design rather than as a side effect of a generic manipulation level), and `supervise` (gates the is_end_goal task `BarbecueParty`, at a 0.5 floor -- added to ALL FOUR agents, since any of them could in principle be designated supervisor in exp1.yaml. As of this revision, `host` is the default designated supervisor (changed from the original fr3_arm_1); host clears the gate with a wide margin (manipulation 0.9, supervise 0.8, both well above the 0.4/0.5 floors). Set F specifically designates OTHER agents (fr3_arm_2) or lowers host's own supervise level, to exercise the FAILURE path of Supervisor.py's `_check_supervisor_eligibility` method, called before any scoring runs: if the designated supervisor fails BarbecueParty's required_skills, the whole run shuts down via the same publish('shutdown', ...) + agent.handle_shutdown(...) pattern already used for unassigned_tasks. BarbecueParty's pre-existing `transport`/`communication` requirements were REMOVED (back when fr3_arm_1 was still the default supervisor and had neither skill, leaving them in place would have shut down every experiment immediately once the new check started actually enforcing them); they remain removed now since host's elevated requirements (manipulation+supervise only) are still sufficient and changing them again isn't necessary.).
- Each task family now has a 2-3 deep candidate pool with a deliberate skill gradient, so 'skill' mode, 'preference' mode, and 'balance_workload' mode can each pick a DIFFERENT winner: manipulation ranks fr3_arm_1 (1.0) > host (0.9) > fr3_arm_2 (0.85) > tiago (0.8, deliberately below the 0.9 floor used by ServeMainDish); chopping ranks fr3_arm_2 (0.9, RANK 1 of 3 -- strong preference) > host (0.4) > fr3_arm_1 (0.45, a fallback value in case workspaces are ever swapped) > tiago (0.3, RANK 7 of 7 -- least interest); grill ranks fr3_arm_1 (1.0, RANK 2 of 4, just behind its top manipulation preference) > tiago (0.55, a real but clearly second-place candidate); supervise ranks host (0.8, RANK 2 of 7 -- second only to its own communication preference, since 'the human has the stronger preference for manage' is interpreted as stronger than the OTHER agents' supervise preference, not stronger than the host's own expected role of welcoming guests) > tiago (0.6, RANK 4 of 7) > fr3_arm_1 (0.55, RANK 3 of 4 -- a valid supervisor candidate by level if ever designated, but NOT the default as of this revision) > fr3_arm_2 (0.2, RANK 3 of 3 -- the agent Set F deliberately designates to demonstrate the gate FAILING, see exp_supervisor_fr3_arm2_fails). Tiago's strongest preference is transport (RANK 1 of 7); the host's is communication (RANK 1 of 7 -- host is both skill leader AND top preference there, consistent with being the agent expected to welcome guests); fr3_arm_1's is manipulation (RANK 1 of 4); fr3_arm_2's is chopping (RANK 1 of 3) — each set per explicit design intent, not incidentally.
- Only `CookRice` and `ServeMainDish` remain host-solo in `baseline_contested`: `CookRice` because human_host is the only agent with the `cook` skill at all, `ServeMainDish` because it is gated at manipulation>=0.9, above tiago's deliberate 0.8 ceiling and unreachable by either arm, since neither declares Kitchen_Stove workspace.
- Set F (new) is the only set that changes `exp1.yaml`'s `supervisor:` field or touches a skill purely for ITS effect on the end-goal gate rather than on any contested task: `exp_supervisor_host` is the passing reference point (host, wide margin), `exp_supervisor_fr3_arm2_fails` designates fr3_arm_2 instead (whose supervise level, 0.2, was already below the 0.5 floor in baseline_contested -- no other change needed), and `exp_supervisor_threshold_fail` keeps host as supervisor but drops ITS supervise level to 0.45. Both failure experiments trigger `Supervisor._check_supervisor_eligibility`'s shutdown path; `validate_experiments.py` labels them EXPECTED-FAIL rather than FAIL, since the failure is the intended demonstration, not a bug.

Experiments are organized into per-set subfolders (`Set_A_Mode_Sweep/`, `Set_B_Skill_Level_Sweep/`, etc). Run each experiment by pointing your launcher at that experiment's `exp1.yaml` inside its set subfolder (it references the other files by agent id, matching the filenames in the same folder).

---

## Set A — Mode Sweep
*(folder: `Set_A_Mode_Sweep/`)*

### `Set_A_Mode_Sweep/exp_mode_skill/`

**Config:** optimizeMode = 'skill', all agent configs identical to baseline.

**Hypothesis / what to look for:** Isolates the effect of the scoring mode itself. Compare task->agent assignments across the four exp_mode_* folders with everything else held constant to see how scoring strategy alone changes allocation.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode: preference -> skill

### `Set_A_Mode_Sweep/exp_mode_preference/`

**Config:** optimizeMode = 'preference', all agent configs identical to baseline.

**Hypothesis / what to look for:** Isolates the effect of the scoring mode itself. Compare task->agent assignments across the four exp_mode_* folders with everything else held constant to see how scoring strategy alone changes allocation. NOTE: this is the same mode as baseline_contested/, so this folder is identical to baseline — included so all four modes have their own clearly-named folder, not because anything changed.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode: unchanged (preference) — baseline already uses this mode; this folder exists as the reference cell for set A, identical to baseline_contested/

### `Set_A_Mode_Sweep/exp_mode_balance_skill_preference/`

**Config:** optimizeMode = 'balance_skill_preference', all agent configs identical to baseline.

**Hypothesis / what to look for:** Isolates the effect of the scoring mode itself. Compare task->agent assignments across the four exp_mode_* folders with everything else held constant to see how scoring strategy alone changes allocation.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode: preference -> balance_skill_preference

### `Set_A_Mode_Sweep/exp_mode_balance_workload/`

**Config:** optimizeMode = 'balance_workload', all agent configs identical to baseline.

**Hypothesis / what to look for:** Isolates the effect of the scoring mode itself. Compare task->agent assignments across the four exp_mode_* folders with everything else held constant to see how scoring strategy alone changes allocation.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode: preference -> balance_workload

## Set B — Skill Level Sweep (mode fixed: skill)
*(folder: `Set_B_Skill_Level_Sweep/`)*

### `Set_B_Skill_Level_Sweep/exp_skill_low/`

**Config:** optimizeMode = 'skill'. fr3_arm_2 manipulation level set to 0.55.

**Hypothesis / what to look for:** Below the 0.6 Pick-task threshold, but still above the 0.5 manipulation floor on Chop* tasks (so fr3_arm_2 keeps its Chop* eligibility via 'chopping', unaffected by this change). fr3_arm_2 drops out of the PickSaladIngredients/PickChoppedSaladIngredients/PickVegetables/PickChoppedVegetables contest entirely (skill_ok fails on manipulation), leaving fr3_arm_1 (1.0) and host (0.9) to contest those tasks alone.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> skill
- `fr3_arm_2.yaml`: skill_weights.HouseParty.manipulation[0] -> 0.55

### `Set_B_Skill_Level_Sweep/exp_skill_mid/`

**Config:** optimizeMode = 'skill'. fr3_arm_2 manipulation level set to 0.6.

**Hypothesis / what to look for:** Exactly at the Pick-task threshold: fr3_arm_2 stays eligible but is now the weakest of the three contestants on raw skill (0.6 vs fr3_arm_1's 1.0 and host's 0.9). In 'skill' mode fr3_arm_1 should win every contested Pick task.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> skill
- `fr3_arm_2.yaml`: skill_weights.HouseParty.manipulation[0] -> 0.6

### `Set_B_Skill_Level_Sweep/exp_skill_high/`

**Config:** optimizeMode = 'skill'. fr3_arm_2 manipulation level set to 1.0.

**Hypothesis / what to look for:** Tied with fr3_arm_1 at the ceiling (1.0), both above host's 0.9. In 'skill' mode this should produce a fr3_arm_1 / fr3_arm_2 tie on every contested Pick task, broken only by get_selected_agent's workload tie-break (fewest tasks assigned so far, then ranking order) -- a useful demonstration of the tie-break rule in isolation from any skill difference.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> skill
- `fr3_arm_2.yaml`: skill_weights.HouseParty.manipulation[0] -> 1.0

## Set C — Preference Rank Sweep (mode fixed: preference)
*(folder: `Set_C_Preference_Rank_Sweep/`)*

### `Set_C_Preference_Rank_Sweep/exp_pref_baseline_order/`

**Config:** optimizeMode = 'preference'. tiago_robot_1 preference RANKING set to (most to least preferred): ['transport', 'manipulation', 'communication', 'supervise', 'openDoor', 'grill', 'chopping']. Skill levels unchanged.

**Hypothesis / what to look for:** Restates tiago's baseline_contested preference order explicitly under 'preference' mode (no change from baseline_contested, included as the set C reference point): transport(1) > manipulation(2) > communication(3) > supervise(4) > openDoor(5) > grill(6) > chopping(7). Tiago's preference score should favor transport-heavy tasks (ServeGrilledFood, PickDishes, PutTheDishesAway, all req. transport>=0.6-1.0) over communication-weighted WelcomeGuests, when contesting the host.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> preference
- `tiago_robot_1.yaml`: skill_weights.HouseParty.*[1] (rank) reordered to ['transport', 'manipulation', 'communication', 'supervise', 'openDoor', 'grill', 'chopping']

### `Set_C_Preference_Rank_Sweep/exp_pref_inverted/`

**Config:** optimizeMode = 'preference'. tiago_robot_1 preference RANKING set to (most to least preferred): ['chopping', 'grill', 'openDoor', 'supervise', 'communication', 'manipulation', 'transport']. Skill levels unchanged.

**Hypothesis / what to look for:** Fully inverts tiago's preference order: chopping now RANK 1 (most preferred), transport now RANK 7 (least preferred) -- the exact opposite of baseline_contested. Despite tiago's raw transport SKILL (1.0) being completely unchanged and still tied with the host's 1.0, tiago's preference-mode standing on transport-heavy tasks (ServeDrinks, ServeGrilledFood, PickDishes, PutTheDishesAway) should now be its WORST preference score (1/7), while its chopping preference score becomes its best (7/7) -- even though tiago's chopping skill level (0.3) remains its weakest skill by far and Chop* tasks aren't even in tiago's eligible set (host/fr3_arm_2 only, per Set B). This isolates the case where a preference reordering has NO visible effect on any actual assignment, because the newly-top-preferred skill isn't tied to any task tiago is eligible for -- a useful null-result contrast to the other two variants in this set.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> preference
- `tiago_robot_1.yaml`: skill_weights.HouseParty.*[1] (rank) reordered to ['chopping', 'grill', 'openDoor', 'supervise', 'communication', 'manipulation', 'transport']

### `Set_C_Preference_Rank_Sweep/exp_pref_communication_top/`

**Config:** optimizeMode = 'preference'. tiago_robot_1 preference RANKING set to (most to least preferred): ['communication', 'transport', 'manipulation', 'supervise', 'openDoor', 'grill', 'chopping']. Skill levels unchanged.

**Hypothesis / what to look for:** Promotes communication to RANK 1 (was rank 3), demoting transport and manipulation down one slot each; supervise/openDoor/grill/chopping stay in their baseline_contested relative order. Since tiago's communication skill LEVEL (0.75, unchanged) already clears WelcomeGuests' 0.7 threshold, this is the cleanest test in set C: expect tiago's preference score for WelcomeGuests to rise from 0.71 (rank 3/7) to 1.00 (rank 1/7), but this STILL ONLY TIES the host's own communication preference score (also 1.00, rank 1/7 -- host is both skill leader AND top preference on communication, since it is the agent expected to welcome guests). Unlike Table 5's other two rows, this is the one case in set C where tiago cannot out-rank the host on preference alone; any tie here would be resolved by get_selected_agent's workload tie-break, not by preference score.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> preference
- `tiago_robot_1.yaml`: skill_weights.HouseParty.*[1] (rank) reordered to ['communication', 'transport', 'manipulation', 'supervise', 'openDoor', 'grill', 'chopping']

## Set D — Workspace Sweep (mode fixed: balance_workload)
*(folder: `Set_D_Workspace_Sweep/`)*

### `Set_D_Workspace_Sweep/exp_workspace_baseline/`

**Config:** optimizeMode = 'balance_workload', workspaces unchanged from baseline_contested: fr3_arm_1 -> Grill_Table only, fr3_arm_2 -> Prep_Table only (arms never share a table), tiago and host -> all 9 workspaces.

**Hypothesis / what to look for:** Reference point for set D. tiago contests the host (and, on the grill/prep tasks, the relevant arm) on 19 of 21 tasks -- only CookRice and ServeMainDish stay host-solo, both gated at manipulation>=0.9, above tiago's deliberate 0.8 ceiling and unreachable by either arm. The arms remain workspace-disjoint specialists with no contest between them, by physical necessity.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> balance_workload

### `Set_D_Workspace_Sweep/exp_workspace_tiago_no_grill_no_prep/`

**Config:** optimizeMode = 'balance_workload'. tiago_robot_1 loses Grill_Table and Prep_Table specifically (the two arm-occupied tables), keeping every other workspace it had in baseline_contested.

**Hypothesis / what to look for:** tiago drops out of all 3 grill-chain contests (PickFoodIngredientsGrill/GrillFood/PickGrilledFood revert to fr3_arm_1-solo) and all 4 Prep_Table Pick* contests (revert to fr3_arm_2-vs-host). It ALSO drops out of every task whose workspace list combines Grill_Table or Prep_Table WITH another room, even though those rooms themselves are untouched: ServeSalad/ServeSides (Prep_Table + Garden), ServeGrilledFood (Grill_Table + Garden), and Host (which requires all five of Grill_Table/Garden/House/Drinks_Table/Prep_Table simultaneously) all revert to host-solo as well. Only WelcomeGuests, ServeDrinks, PickRice, PickDishes, DoTheDishes, and PutTheDishesAway remain tiago-vs-host contests, since none of those six require Grill_Table or Prep_Table. This is a useful illustration that a workspace requirement combining multiple rooms is exactly as restrictive as its single hardest-to-reach room -- removing tiago's access to just two tables collapses more than a third of its original 17-task eligible set.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> balance_workload
- `tiago_robot_1.yaml`: workspace: [all 9] -> [all 9 minus Grill_Table, Prep_Table]

### `Set_D_Workspace_Sweep/exp_workspace_tiago_drinks_only/`

**Config:** optimizeMode = 'balance_workload'. tiago_robot_1 workspace cut all the way down to ['Drinks_Table'] only -- the deepest restriction in this set.

**Hypothesis / what to look for:** tiago becomes ineligible for every task it previously contested against the host or the arms: every remaining contest requires House, Garden, Prep_Table, Grill_Table, or a Kitchen_* room, none of which tiago can reach anymore (even ServeDrinks, which needs Drinks_Table AND Garden together, is lost). Expect tiago to become fully idle on the HouseParty context under this restriction, with the host and the two arms absorbing every task tiago previously contested. This is the clean 'workspace can fully exclude an otherwise skill-eligible agent' demonstration for the paper, at maximum restriction.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> balance_workload
- `tiago_robot_1.yaml`: workspace: [all 9] -> [Drinks_Table]

## Set E — Interaction (mode fixed: balance_skill_preference)
*(folder: `Set_E_Interaction/`)*

### `Set_E_Interaction/exp_interaction_baseline/`

**Config:** optimizeMode = 'balance_skill_preference', all agent configs unchanged.

**Hypothesis / what to look for:** Reference point for set E. With alpha=0.7, scores should sit closer to pure 'skill' mode than pure 'preference' mode for these baseline configs. Compare against exp_mode_skill and exp_mode_preference.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> balance_skill_preference

### `Set_E_Interaction/exp_interaction_low_skill_high_pref/`

**Config:** optimizeMode = 'balance_skill_preference'. fr3_arm_2 manipulation LEVEL lowered to 0.55 (below the 0.6 manipulation requirement on every Pick* task it shares with fr3_arm_1/host: PickSaladIngredients, PickChoppedSaladIngredients, PickVegetables, PickChoppedVegetables -- but still above the 0.5 manipulation floor on Chop* tasks) while its preference RANKING is swapped so manipulation becomes RANK 1 of 3 (pref_score 1.0, was rank 2 / 0.67) and chopping drops to RANK 2 of 3 (pref_score 0.67, was rank 1 / 1.0 in baseline_contested); supervise stays RANK 3 of 3 throughout. Its chopping skill LEVEL (0.9) is untouched, so it remains eligible for Chop* tasks regardless.

**Hypothesis / what to look for:** Tests whether a high preference rank can ever compensate for failing the hard skill_ok eligibility gate (Supervisor.py line 146). It cannot: skill_ok is a precondition checked before scoring, so fr3_arm_2 should be filtered out of every Pick* task it shares with fr3_arm_1 and the host, regardless of manipulation now being its top-ranked preference -- while remaining the sole chopping-skilled candidate on Chop* tasks (alongside host, since host's chopping=0.4 clears the 0.4 floor too), completely unaffected by this change. This experiment demonstrates that 'balance_skill_preference' blends score AMONG eligible agents only -- it does not let preference rescue an agent that fails the minimum skill bar.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> balance_skill_preference
- `fr3_arm_2.yaml`: skill_weights.HouseParty.manipulation level -> 0.55 (was 0.85); preference RANKS swapped: manipulation rank 2->1, chopping rank 1->2 (supervise stays rank 3/3, unchanged)

### `Set_E_Interaction/exp_interaction_equal_skill_diff_pref/`

**Config:** optimizeMode = 'balance_skill_preference'. human_host and tiago_robot_1 given IDENTICAL communication skill level (0.8, both clear WelcomeGuests' 0.7 and Host's 0.2 communication requirements) but OPPOSITE preference RANKS: human_host's communication demoted to RANK 7 of 7 (its least preferred skill, was rank 1 in baseline_contested -- this pushes EVERY other host skill up one slot, so supervise becomes its new rank-1 preference as a side effect, not by direct intent), tiago's communication promoted to RANK 1 of 7 (its most preferred skill, was rank 3). All other relative orderings for both agents are preserved, just shifted to make room.

**Hypothesis / what to look for:** With skill level tied exactly at a value that clears every communication-gated task's threshold, the 0.3 preference slice of the balance_skill_preference score becomes the only tie-breaker: host's communication pref_score drops from 1.0 (rank 1/7) to 0.14 (rank 7/7), while tiago's rises from 0.71 (rank 3/7) to 1.0 (rank 1/7). Expect tiago to win WelcomeGuests and Host (the communication-gated tasks where it is workspace- and skill-eligible alongside the host) purely on this preference inversion, demonstrating the alpha blend actually matters when skill is equalized -- unlike the previous experiment in this set, where skill ineligibility dominated regardless of preference rank. NOTE: this experiment's reordering pushes host's supervise preference UP to rank 1 of 7 (from rank 2) as an unavoidable side effect of demoting communication all the way to last -- the host's supervise SKILL LEVEL (0.8) is unchanged. Since host IS the actual designated supervisor in exp1.yaml (as of this revision), BarbecueParty's end-goal eligibility check still passes here regardless: _check_supervisor_eligibility reads supervise's LEVEL (0.8 >= 0.5), not its preference RANK, so this experiment's reordering has zero effect on the end-goal outcome even though it incidentally makes supervise host's top preference. See Set F for experiments that actually move the supervise LEVEL, which is what the gate checks.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> balance_skill_preference
- `human_host.yaml`: communication level -> 0.8 (was 1.0); communication rank -> 7/7 (was 1/7, now its least preferred; supervise rises to rank 1/7 as a side effect, was rank 2/7)
- `tiago_robot_1.yaml`: communication level -> 0.8 (was 0.75); communication rank -> 1/7 (was 3/7, now its most preferred)

## Set F — Supervisor Eligibility Sweep (mode fixed: balance_workload)
*(folder: `Set_F_Supervisor_Eligibility_Sweep/`)*

### `Set_F_Supervisor_Eligibility_Sweep/exp_supervisor_host/`

**Config:** optimizeMode = 'balance_workload'. supervisor unchanged from baseline_contested: host (manipulation=0.9, supervise=0.8, both comfortably above BarbecueParty's 0.4/0.5 floors).

**Hypothesis / what to look for:** Reference point for set F. Supervisor._check_supervisor_eligibility passes with a wide margin (0.5 headroom on manipulation, 0.3 on supervise) -- this is the 'everything is fine' case, included so the two failure cases below have something to contrast against.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> balance_workload

### `Set_F_Supervisor_Eligibility_Sweep/exp_supervisor_fr3_arm2_fails/`

**Config:** optimizeMode = 'balance_workload'. supervisor changed from host to fr3_arm_2. No other configuration is touched -- fr3_arm_2's supervise level is already 0.2 in baseline_contested (its lowest-ranked, least-preferred skill, RANK 3 of 3), well below BarbecueParty's 0.5 floor.

**Hypothesis / what to look for:** Supervisor._check_supervisor_eligibility should fail on the 'supervise' requirement (0.2 < 0.5) the moment assign_agents_to_tasks is called, BEFORE any scoring runs for any task. Expect the same shutdown path used for unassigned_tasks: publish('shutdown', {'reason': 'supervisor_unqualified', ...}) followed by agent.handle_shutdown('supervisor_unqualified', missing). fr3_arm_2's manipulation (0.85) clears BarbecueParty's other requirement fine -- this experiment isolates a single-skill failure (supervise only), not a wholesale mismatch.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> balance_workload; supervisor: host -> fr3_arm_2

### `Set_F_Supervisor_Eligibility_Sweep/exp_supervisor_threshold_fail/`

**Config:** optimizeMode = 'balance_workload'. supervisor stays host (the RIGHT agent, unchanged), but host's own 'supervise' skill LEVEL is lowered from 0.8 to 0.45 -- just below BarbecueParty's 0.5 floor. Its preference RANK for supervise (2 of 7) is untouched, and its manipulation level (0.9) is untouched.

**Hypothesis / what to look for:** Demonstrates the failure mode is about the SKILL LEVEL threshold itself, not about which agent is chosen -- complementing exp_supervisor_fr3_arm2_fails, which fails because the WRONG agent was picked. Here the right agent is still picked, but no longer qualifies. Expect an identical shutdown to the previous experiment (same 'supervise' key in the missing-skills payload), but for a different underlying reason -- this is the 'right candidate became unqualified' case rather than the 'wrong candidate was chosen' case. Also demonstrates this failure is independent of the host's communication/transport/etc skills, none of which are touched.

**Exact diff from baseline:**
- `exp1.yaml`: optimizeMode -> balance_workload
- `human_host.yaml`: skill_weights.HouseParty.supervise level -> 0.45 (was 0.8); rank unchanged (2/7)
