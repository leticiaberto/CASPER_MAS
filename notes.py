class Agent():
    def __init__(
        self,
        id,
        skills,
        constraints,
        preferences,
        capacity,
    ):
        self.id = id
        self.skills = skills                  # graded
        self.constraints = constraints        # boolean / ranges
        self.preferences = preferences
        self.capacity = capacity

        self.assigned_tasks = []

        self.partners = []

    def partner_with(self, other_agent):
        self.partners.append(other_agent.id)

    def update_partner_estimated_skills(self, other_agent):
        """ 
        Update estimated skills of a partner agent based on observations 
        Initially set the estimated skills to the actual level skills indicate by the partner and then refine based on task performance over time.
        """
        pass

    def update_trust_level(self, other_agent, success):
        """
        Update trust level of a partner agent based on task outcomes
        Increase trust if tasks are successful, decrease if they fail.
        """
        pass
            
    def trust_propagation(self):
        """
        Propagate trust levels through the network of partners
        If an agent trusts a partner highly, it may also increase trust in that partner's partners.
        """
        pass

    def share_information_start(self, other_agent, info):
        """
        Share information with a partner agent
        This could include sharing observations about tasks, environments, or other agents.
        """
        pass



def satisfies_constraints(agent, task):
    for key, required in task.required_constraints.items():
        agent_value = agent.constraints.get(key, None)

        if agent_value is None:
            return False

        # Boolean constraint
        if isinstance(required, bool):
            if agent_value != required:
                return False

        # Categorical constraint
        if isinstance(required, str):
            if agent_value != required:
                return False

    return True

def skill_match(agent, task):
    score = 0.0
    for skill, required_level in task.required_skills.items():
        agent_level = agent.skills.get(skill, 0.0)
        score += min(agent_level / required_level, 1.0)
    return score / len(task.required_skills)

def compute_utility(agent, task):
    # Arbitration happens only among feasible agents
    if not satisfies_constraints(agent, task):
        return -float("inf")

    skill_score = skill_match(agent, task)
    if skill_score == 0:
        return -float("inf")

    utility = (
        skill_score * task.priority
        - agent.preferences["effort"] * task.effort
    )
    return utility

def allocate_tasks(agents, tasks):
    bids = []

    for agent in agents:
        if len(agent.assigned_tasks) >= agent.capacity:
            continue

        for task in tasks:
            bid = compute_utility(agent, task)
            if bid > 0:
                bids.append((agent, task, bid))

    bids.sort(key=lambda x: x[2], reverse=True)

    assigned_tasks = set()

    for agent, task, bid in bids:
        if task.id in assigned_tasks:
            continue
        if len(agent.assigned_tasks) >= agent.capacity:
            continue

        agent.assigned_tasks.append(task.id)
        assigned_tasks.add(task.id)

    return assigned_tasks