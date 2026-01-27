class Task:
    def __init__(
        self,
        id,
        required_skills,
        required_constraints,
        effort,
        priority
    ):
        self.id = id
        self.required_skills = required_skills
        self.required_constraints = required_constraints
        self.effort = effort
        self.priority = priority

move_box = Task(
    id="move_box",
    required_skills={"navigation": 0.6},
    required_constraints={
        "can_move": True
    },
    effort=4,
    priority=2
)

pick_object = Task(
    id="pick_object",
    required_skills={"manipulation": 0.7},
    required_constraints={
        "can_manipulate": True,
        "workspace": "station_A"
    },
    effort=3,
    priority=3
)