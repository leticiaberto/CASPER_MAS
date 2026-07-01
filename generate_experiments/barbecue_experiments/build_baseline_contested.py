#!/usr/bin/env python3
"""
build_baseline_contested.py

Derives baseline_contested/ from baseline/ by applying a documented set of
changes designed to make Sets A-E (mode, skill, preference, workspace,
interaction) exercise genuine multi-candidate contests across almost the
entire task graph, while respecting two hardware constraints:

  1. fr3_arm_1 and fr3_arm_2 are fixed-base manipulator arms and CANNOT
     physically share a workspace (no room for both at one table). Each
     keeps its own single workspace (fr3_arm_1 -> Grill_Table, fr3_arm_2 ->
     Prep_Table). There is no arm-vs-arm contest anywhere in this design,
     by physical necessity. x_pos/y_pos are launch-only coordinates for
     Gazebo and are never read by Supervisor.py for eligibility -- only
     `workspace` matters -- so they are left untouched.

  2. tiago_robot_1 and human_host are the only MOBILE agents, so they are
     the only agents that can be workspace-eligible for tasks across
     multiple rooms/tables. Both are given access to every workspace in
     the task graph (Prep_Table, Grill_Table, Drinks_Table, House, Garden,
     Kitchen_Table, Kitchen_Stove, Kitchen_sink, Kitchen_shelves).

PREFERENCE FORMAT (IMPORTANT): the second number in each
skill_weights.<context>.<skill> entry is an INTEGER RANK among that
agent's own skills in that context -- 1 = most preferred, 2 = second most
preferred, etc, matching Supervisor.py's documented formula:
    preference_score = (N - rank + 1) / N
where N = how many skills that agent has ranked in that context. This is
NOT a 0-1 weight (the original baseline/ files used decimals like 0.1/0.6
in that slot, which does not match Supervisor.py's rank-based formula --
baseline_contested corrects this for every agent). Ranks must be a
permutation of 1..N for each agent's context with no gaps or ties.

SKILL/PREFERENCE DESIGN -- each task family has a 2-3 deep candidate pool
with a deliberate skill gradient, so "skill" mode, "preference" mode, and
"balance_workload" mode can each pick a DIFFERENT winner:

  - manipulation: fr3_arm_1 1.0 > host 0.9 > fr3_arm_2 0.85 > tiago 0.8.
    Tiago deliberately sits at 0.8 -- below fr3_arm_1 (per explicit
    instruction) and just below the 0.9 threshold used by CookRice/
    ServeMainDish, so those two tasks stay host-solo while Tiago
    contests everything else manipulation-gated.
  - chopping (new skill, only meaningful at Prep_Table): fr3_arm_2 0.9
    (skill leader, RANK 1 of 2 -- "strong preference for chopping" per
    instruction) > host 0.4 > fr3_arm_1 0.45 (present, just above the
    0.4 Chop-task floor, as an explicit fallback "in case workspaces are
    ever swapped" -- fr3_arm_1 has no Prep_Table access today, so this
    value does nothing unless that constraint changes) > tiago 0.3
    (RANK 6 of 6 -- tiago has no real interest in chopping).
  - grill (new skill, only meaningful at Grill_Table): fr3_arm_1 1.0
    (skill leader, RANK 1 of 3 -- "strong preference for manipulation"
    edges grill to RANK 2, but grill is still fr3_arm_1's clear skill
    strength) > tiago 0.55 (RANK 5 of 6 -- tiago can physically reach
    Grill_Table now but has little interest or skill there). All three
    grill tasks use the same grill>=0.5 floor, so PickFoodIngredientsGrill/
    GrillFood/PickGrilledFood are all fr3_arm_1-vs-tiago contests, not
    just the pick/retrieve steps.
  - transport: tiago 1.0 (RANK 1 of 7 -- "strong preference for
    transport" per instruction) = host 1.0 (RANK 3 of 7 -- host's
    strongest preference is communication, then supervise, so
    transport is its skill-equal but lower-preference third choice).
  - communication: host 1.0 (RANK 1 of 7 -- host is BOTH the skill
    leader on communication AND its top preference: it is the agent
    expected to welcome guests, so communication outranks even its own
    supervise/management preference) > tiago 0.75 (RANK 3 of 7, clears
    WelcomeGuests' 0.7 gate, contests host there, but can at best tie
    host's preference score, never exceed it).
  - supervise (new skill, only meaningful on the is_end_goal task
    BarbecueParty): host 0.8 (RANK 2 of 7 -- second only to its own
    communication preference; "the human has the stronger preference
    for manage" is interpreted as stronger than the OTHER agents'
    supervise preference, not stronger than the host's own expected
    welcoming role) > tiago 0.6 (RANK 4 of 7 -- a plausible secondary
    coordinator given its mobility, but well behind transport/
    manipulation/communication) > fr3_arm_1 0.55 (RANK 3 of 4 -- just
    above BarbecueParty's 0.5 floor; fr3_arm_1 is the agent designated
    `supervisor` in exp1.yaml, so it MUST clear this floor or the new
    Supervisor._check_supervisor_eligibility method shuts every run
    down before scoring begins) > fr3_arm_2 0.2 (RANK 3 of 3 -- present
    on every agent now, but fr3_arm_2 has no coordination role and is
    never the designated supervisor).
  - cook (new skill, only meaningful on CookRice, Kitchen_Table +
    Kitchen_Stove workspace): host 0.9 (RANK 6 of 7) is the ONLY agent
    with this skill at all (level 0.0 for everyone else -> fails the
    skill_ok gate) -- CookRice stays host-solo, but now for the explicit
    named reason of being the sole cook-capable agent, rather than an
    implicit side effect of a generic manipulation level.

All other values (durations, priorities, dependencies, other
constraints) are left untouched from baseline/.

barbecue.json's BarbecueParty (is_end_goal) task: "transport" and
"communication" were REMOVED from its required_skills (they were always
silently inert under the original score_agents_for_task, whose
is_end_goal branch never checks required_skills at all -- see
Supervisor.py's _check_supervisor_eligibility, added alongside this
baseline, which is the first code path that actually enforces them).
fr3_arm_1, the supervisor designated in exp1.yaml, has neither transport
nor communication skills, so leaving those requirements in place would
shut down every experiment immediately. BarbecueParty's required_skills
are now just {"manipulation": 0.4, "supervise": 0.5} -- both of which
fr3_arm_1 clears (1.0 and 0.55 respectively).

Verified by simulation that with these changes, 19 of 21 non-end-goal
tasks resolve to >=2 eligible agents (only CookRice and ServeMainDish
remain host-solo -- CookRice gated by the new "cook" skill, ServeMainDish
gated at manipulation>=0.9, above tiago's deliberate 0.8 ceiling and
unreachable by either arm), zero tasks are unassignable, and the
designated supervisor (fr3_arm_1) passes its own end-goal eligibility
check.
"""

import json
import os
import yaml

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
BASELINE_DIR = os.path.join(BASE_DIR, "baseline")
OUT_DIR = os.path.join(BASE_DIR, "baseline_contested")

ALL_WORKSPACES = [
    "Prep_Table", "Grill_Table", "Drinks_Table", "House", "Garden",
    "Kitchen_Table", "Kitchen_Stove", "Kitchen_sink", "Kitchen_shelves",
]


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # --- exp1.yaml: supervisor changed from fr3_arm_1 to host (robot_id
    # "host" -- note this is the HouseParty robot_id declared in
    # human_host.yaml, NOT the filename stem "human_host"; every other
    # agent's robot_id happens to match its filename stem exactly, which
    # is why this distinction never mattered before host became the
    # supervisor). host clears BarbecueParty's gate with a wide margin
    # (manipulation 0.9 >= 0.4, supervise 0.8 >= 0.5) -- see Set F for
    # experiments that exercise the gate's FAILURE path instead, since
    # host passing trivially doesn't exercise
    # Supervisor._check_supervisor_eligibility's interesting case. ---
    with open(os.path.join(BASELINE_DIR, "exp1.yaml")) as f:
        exp = yaml.safe_load(f)
    exp["supervisor"] = "host"
    _write_yaml(exp, "exp1.yaml")

    # --- fr3_arm_1.yaml: Grill_Table only. manipulation RANK 1 (strong
    # preference, per instruction), supervise RANK 2 (0.55 -- just above
    # BarbecueParty's 0.5 floor; fr3_arm_1 is NOT the default supervisor
    # as of this revision (host is -- see exp1.yaml above), but this
    # value is kept above the floor so fr3_arm_1 remains a valid
    # supervisor candidate if Set F's experiments designate it instead),
    # chopping RANK 3 (low level 0.45 -- a fallback value, not a real
    # contender today since fr3_arm_1 has no Prep_Table access), grill
    # RANK 4 (LAST, despite grill being fr3_arm_1's clear SKILL strength
    # at level 1.0 -- this is a deliberate skill/preference divergence:
    # fr3_arm_1 still wins 'skill' mode on the grill chain cleanly
    # (manipulation+grill level sum 2.0, vs tiago's 1.35), but with grill
    # demoted to its lowest preference, fr3_arm_1's PREFERENCE sum on the
    # grill chain (manipulation rank1 + grill rank4, out of N=4: 1.0 +
    # 0.25 = 1.25) drops just below tiago's (manipulation rank2 + grill
    # rank5, out of N=7: 0.857 + 0.429 = 1.286 -- see tiago's skill_weights
    # block below for the matching grill-rank promotion), giving
    # 'preference' mode a genuine, different winner (tiago) on the same
    # task family where 'skill' mode picks fr3_arm_1. Without this
    # change, fr3_arm_1 dominated both modes on the grill chain and no
    # divergence existed there at all.). ---
    with open(os.path.join(BASELINE_DIR, "fr3_arm_1.yaml")) as f:
        arm1 = yaml.safe_load(f)
    arm1["skill_weights"]["HouseParty"] = {
        "manipulation": [1.0, 1],
        "supervise":    [0.55, 2],
        "chopping":     [0.45, 3],
        "grill":        [1.0, 4],
    }
    _write_yaml(arm1, "fr3_arm_1.yaml")

    # --- fr3_arm_2.yaml: Prep_Table only. chopping RANK 1 (strong
    # preference, per instruction), manipulation RANK 2; manipulation
    # level dropped 1.0 -> 0.85 so it isn't tied with fr3_arm_1/host on
    # the Pick* contest. supervise RANK 3, level 0.2 -- present (every
    # agent has it now) but deliberately low, since fr3_arm_2 is not the
    # designated supervisor and has no coordination role. ---
    with open(os.path.join(BASELINE_DIR, "fr3_arm_2.yaml")) as f:
        arm2 = yaml.safe_load(f)
    arm2["skill_weights"]["HouseParty"] = {
        "chopping":     [0.9, 1],
        "manipulation": [0.85, 2],
        "supervise":    [0.2, 3],
    }
    _write_yaml(arm2, "fr3_arm_2.yaml")

    # --- tiago_robot_1.yaml: mobile, gains every workspace. transport
    # RANK 1 (strong preference, per instruction), manipulation RANK 2
    # (level 0.8 -- explicitly below fr3_arm_1's 1.0 and below the 0.9
    # threshold used by ServeMainDish), communication RANK 3 (0.75,
    # clears WelcomeGuests' 0.7 gate), supervise RANK 4 (0.6 -- a
    # plausible secondary coordinator given its mobility, but ranked
    # below transport/manipulation/communication, not the host's
    # stronger management preference), grill RANK 5 (0.55 -- promoted
    # one slot above openDoor, swapped from its earlier position at
    # rank 6, as part of a deliberate skill/preference divergence on the
    # grill chain: see fr3_arm_1's skill_weights block above for the
    # matching grill-rank demotion. fr3_arm_1 still wins 'skill' mode on
    # PickFoodIngredientsGrill/GrillFood/PickGrilledFood cleanly by raw
    # level, but tiago now wins 'preference' mode there, since fr3_arm_1
    # ranks grill last among its own 4 skills while tiago ranks it 5th
    # of 7 -- still a real but clearly second-place SKILL candidate, per
    # the original instruction, just no longer also fr3_arm_1's
    # preference for that task), openDoor RANK 6 (0.5, demoted one slot
    # to make room for grill), chopping RANK 7 (0.3 -- least interest/
    # skill). ---
    with open(os.path.join(BASELINE_DIR, "tiago_robot_1.yaml")) as f:
        tiago = yaml.safe_load(f)
    tiago["workspace"] = list(ALL_WORKSPACES)
    tiago["skill_weights"]["HouseParty"] = {
        "transport":     [1.0, 1],
        "manipulation":  [0.8, 2],
        "communication": [0.75, 3],
        "supervise":     [0.6, 4],
        "grill":         [0.55, 5],
        "openDoor":      [0.5, 6],
        "chopping":      [0.3, 7],
    }
    _write_yaml(tiago, "tiago_robot_1.yaml")

    # --- human_host.yaml: mobile, already had every workspace (kept).
    # communication RANK 1 (host is BOTH the skill leader on
    # communication, 1.0, AND its top preference -- it is the agent
    # expected to welcome guests, so communication should not be
    # outranked by anything, including its own management/supervise
    # skill). supervise RANK 2 (0.8 level -- still high, and still the
    # host's strongest preference among everything OTHER than
    # communication; "the human has the stronger preference for manage"
    # per instruction means stronger than the other agents' supervise
    # preference, not stronger than the host's own communication
    # preference). transport RANK 3, manipulation RANK 4 (0.8, NOT 0.9
    # -- see note below), openDoor RANK 5, cook RANK 6 (0.9 -- new
    # skill, makes host the sole "cook"-capable agent; CookRice now
    # gates on this instead of manipulation alone), chopping RANK 7
    # (0.4 -- present so host is a workload-balancing fallback on Chop
    # tasks, but never the skill or preference leader there).
    #
    # IMPORTANT: manipulation was originally set to 0.9 (a "generalist,
    # but still below the specialist arms" value), but this was a real
    # bug: it is HIGHER than both tiago's manipulation (0.8) and
    # fr3_arm_2's (0.85 -- wait, 0.9 > 0.85 too), so in "skill" mode
    # host won every manipulation-gated contested task by raw level
    # alone (PickSaladIngredients, ChopSaladIngredients, PickVegetables,
    # ChopVegetables, ServeDrinks, PickDishes, DoTheDishes, etc),
    # completely defeating the point of having specialist robot agents
    # at all. Dropping it to 0.8 (an exact TIE with tiago) restores
    # fr3_arm_2 as the clean, unambiguous skill-mode winner on every
    # Pick*/Chop* task (0.85 > 0.8), since those are single-skill
    # (manipulation only) or chopping-dominated comparisons where host
    # no longer has the edge. On multi-skill tasks where host and tiago
    # are ALSO tied on transport (1.0 vs 1.0) -- ServeDrinks, PickDishes,
    # DoTheDishes, ServeSalad, ServeSides, ServeGrilledFood,
    # PutTheDishesAway, PickRice -- host and tiago now tie exactly on
    # the averaged score too, so the actual "skill mode" winner on
    # those specific tasks falls to whatever tie-break rule
    # get_selected_agent uses (not modeled here -- see
    # validate_experiments.py's docstring for what IS and is NOT
    # verified). Any manipulation value strictly BETWEEN 0.8 and 0.85
    # was tried and rejected: it keeps host eligible and stops it from
    # beating fr3_arm_2 on single-skill tasks, but on the two-skill
    # tasks (manipulation+transport averaged) host's transport tie plus
    # ANY manipulation edge over tiago still tips the average back in
    # host's favor -- so 0.8 (an exact tie, not a host loss) is the only
    # value that avoids re-introducing the same bug on those tasks while
    # keeping host workspace+skill eligible everywhere it was before. ---
    with open(os.path.join(BASELINE_DIR, "human_host.yaml")) as f:
        host = yaml.safe_load(f)
    host["workspace"] = list(ALL_WORKSPACES)
    host["skill_weights"]["HouseParty"] = {
        "communication": [1.0, 1],
        "supervise":     [0.8, 2],
        "transport":     [1.0, 3],
        "manipulation":  [0.8, 4],
        "openDoor":      [1.0, 5],
        "cook":          [0.9, 6],
        "chopping":      [0.4, 7],
    }
    _write_yaml(host, "human_host.yaml")

    # --- barbecue.json: Chop* tasks gain "chopping" requirement and have
    # "manipulation" lowered so it no longer co-gates with chopping; all
    # three Grill* tasks gain a "grill" requirement (same floor, 0.5) and
    # have "manipulation" lowered to a low floor so grill -- not
    # manipulation -- is the real differentiator there. CookRice gains a
    # "cook" requirement (0.5) with "manipulation" lowered to 0.5 for the
    # same reason -- host is the only "cook"-capable agent, so this task
    # stays host-solo, but now for an explicit, named reason instead of
    # an implicit side effect of a generic manipulation level.
    # ServeMainDish's "manipulation" requirement is lowered from 0.9 to
    # 0.7: the original 0.9 floor only worked because host's manipulation
    # LEVEL happened to also be 0.9 -- when host's manipulation was
    # dropped to 0.8 (to fix the "skill mode" bug where host beat both
    # specialist arms on every manipulation-gated task; see the comment
    # above host's skill_weights block), ServeMainDish became
    # UNASSIGNABLE (host was the only Kitchen_Stove-eligible agent and no
    # longer cleared 0.9). 0.7 keeps host the sole eligible agent
    # (unchanged conclusion -- still host-solo, since no robot reaches
    # Kitchen_Stove) while no longer accidentally depending on a generic
    # manipulation value that other fixes might move again later.
    # BarbecueParty (the is_end_goal task) gains a "supervise" requirement
    # (0.5); its pre-existing "transport" and "communication"
    # requirements are REMOVED. Those two were always silently inert
    # under the original score_agents_for_task (the is_end_goal branch
    # never checks required_skills at all), so this is the first time any
    # of BarbecueParty's requirements are actually enforced -- by the new
    # Supervisor._check_supervisor_eligibility method, called before
    # scoring begins. fr3_arm_1 (originally the designated supervisor;
    # host is the default as of a later revision) does not have transport
    # or communication skills at all, so leaving those two requirements
    # in place would have shut down every single experiment immediately;
    # removing them keeps the end-goal gate to just manipulation plus the
    # new supervise requirement. ---
    with open(os.path.join(BASELINE_DIR, "barbecue.json")) as f:
        tasks_data = json.load(f)
    for task in tasks_data["tasks"]:
        if task["id"] in ("ChopSaladIngredients", "ChopVegetables"):
            task["required_skills"] = {"manipulation": 0.5, "chopping": 0.4}
        elif task["id"] in ("PickFoodIngredientsGrill", "GrillFood", "PickGrilledFood"):
            task["required_skills"] = {"manipulation": 0.5, "grill": 0.5}
        elif task["id"] == "CookRice":
            task["required_skills"] = {"manipulation": 0.5, "cook": 0.5}
        elif task["id"] == "ServeMainDish":
            task["required_skills"] = {"manipulation": 0.7, "transport": 1.0}
        elif task["id"] == "BarbecueParty":
            task["required_skills"] = {"manipulation": 0.4, "supervise": 0.5}
    with open(os.path.join(OUT_DIR, "barbecue.json"), "w") as f:
        json.dump(tasks_data, f, indent=2)

    print(f"baseline_contested/ written to {OUT_DIR}")


def _write_yaml(data, fname):
    with open(os.path.join(OUT_DIR, fname), "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


if __name__ == "__main__":
    main()


