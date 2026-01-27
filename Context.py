from collections import defaultdict

class ContextualSkillModel:
    def __init__(self, skill_weights, contexts, ):
        self.skill_level = defaultdict(dict)        # Stores skill values
        self.skill_preference = defaultdict(dict)    # Stores preference values
        self.start_skills_and_preferences(skill_weights, contexts)
        
    def start_skills_and_preferences(self, skill_weights, contexts, default=0.1):
        """
        Sets the skill level and preferences for a given skill in a specific context.

        :param skill_weights: The skill to set the level/preference for. Values to set (between 0.0 and 1.0)
        :param context: The context in which the skill level is set.
        :returns: None
        """
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

    def print_skills_preferences(self):
        for skill, contexts in self.skill_level.items():
            print(f"\nSkill: {skill}")
            for context in contexts:
                print(
                    f"  {context:<10} | "
                    f"Level: {self.skill_level[skill][context]:.2f} | "
                    f"Pref: {self.skill_preference[skill][context]:.2f}"
                )
                
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