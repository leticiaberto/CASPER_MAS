from collections import defaultdict

class ContextualSkillModel:
    def __init__(self, skill_weights, contexts, default=0.1):
        self.skill_level = defaultdict(dict)        # Stores skill values
        self.skill_preference = defaultdict(dict)    # Stores preference values

        for context, skills in skill_weights.items():
            for skill, weight in skills.items():
                self.skill_level[skill][context] = weight[0]
                self.skill_preference[skill][context] = weight[1]

        # Fill missing contexts with default values
        for skill in self.skill_level:
            for context in contexts:
                if context not in self.skill_level[skill]:
                    self.skill_level[skill][context] = default
                if context not in self.skill_preference[skill]:
                    self.skill_preference[skill][context] = default

    def set_skill(self, skill, context, value):
        """
        Sets the skill level for a given skill in a specific context.

        :param skill: The skill to set the level for.
        :param context: The context in which the skill level is set.
        :param value: The skill level value to set (between 0.0 and 1.0).
        :returns: None
        """
        self.skill_level[skill][context] = value

    def set_preference(self, skill, context, value):
        """
        Sets the skill preference for a given skill in a specific context.

        :param skill: The skill to set the preference for.
        :param context: The context in which the skill preference is set.
        :param value: The skill preference value to set (between 0.0 and 1.0).
        :returns: None
        """
        self.skill_preference[skill][context] = value

    def print_skills(self):
        for skill, contexts in self.skill_level.items():
            print(f"Skill: {skill}")
            for context, level in contexts.items():
                print(f"  Context: {context}, Level: {level}")
                
    def update_skill(self, skill, observed, lr=0.1):
        self.skill_level[skill] += lr * (observed - self.skill_level[skill])

    def contribution(self, skill, context, relevance):
        return (
            self.skill_level[skill][context]
            * self.skill_preference[skill][context]
            * relevance
        )
    
class Context:
    def __init__(self, name, relevance):
        self.name = name
        self.relevance = relevance  # dict[skill] -> R(s|c) = Skill relevance for task: [0.0, 1.0]