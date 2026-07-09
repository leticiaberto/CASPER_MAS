from collections import defaultdict
import csv
import os
from datetime import datetime

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

    def update_skills_and_preferences(self, skill_updates, contexts=None, clamp=True):
        """
        Updates existing skill levels and preferences.

        :param skill_updates: dict in the same format as initialization:
        :param contexts: Optional list of valid contexts (for safety checking).
        :param clamp: If True, ensures values stay between 0.0 and 1.0.
        """

        for context, skills in skill_updates.items():

            if contexts is not None and context not in contexts:
                continue  # ignore invalid contexts safely

            for skill, values in skills.items():
                level, preference = values

                if clamp:
                    level = max(0.0, min(1.0, float(level)))
                    preference = max(0.0, min(1.0, float(preference)))

                # Ensure skill exists
                if skill not in self.skill_level:
                    self.skill_level[skill] = {}
                if skill not in self.skill_preference:
                    self.skill_preference[skill] = {}

                # Update values
                self.skill_level[skill][context] = level
                self.skill_preference[skill][context] = preference


    def print_skills_preferences(self):
        for skill, contexts in self.skill_level.items():
            print(f"\nSkill: {skill}")
            for context in contexts:
                print(
                    f"  {context:<10} | "
                    f"Level: {self.skill_level[skill][context]:.2f} | "
                    f"Pref: {self.skill_preference[skill][context]:.2f}"
                )

    def export_skill_weights(self):
        """
        Export skills in the same format expected by start_skills_and_preferences.
        Returns:
            dict[context][skill] = (level, preference)
        """
        result = {}

        for skill in self.skill_level:
            for context in self.skill_level[skill]:
                if context not in result:
                    result[context] = {}

                level = self.skill_level[skill][context]
                pref  = self.skill_preference[skill][context]

                result[context][skill] = (level, pref)

        return result
                
    def update_skill(self, skill, observed, lr=0.1):
        self.skill_level[skill] += lr * (observed - self.skill_level[skill])

    def contribution(self, skill, context, relevance):
        return (
            self.skill_level[skill][context]
            * self.skill_preference[skill][context]
            * relevance
        )

    def export_skills_preferences_to_CSV(self, agent_id, filename="skills_preferences.csv"):
        # Check if file already exists (to decide whether to write header)
        file_exists = os.path.isfile(filename)

        # Current timestamp
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        with open(filename, mode="a", newline="") as file:
            writer = csv.writer(file)

            # Write header only if file is new
            if not file_exists:
                writer.writerow([
                    "Timestamp", "AgentID", "Skill", "Context", "Level", "Preference"
                ])

            # Append data rows
            for skill, contexts in self.skill_level.items():
                for context in contexts:
                    writer.writerow([
                        timestamp,
                        agent_id,
                        skill,
                        context,
                        f"{self.skill_level[skill][context]:.2f}",
                        f"{self.skill_preference[skill][context]:.2f}"
                    ])

        print(f"Data appended successfully to {filename}")

    def has_context(self, context):
        return any(context in contexts for contexts in self.skill_level.values())
    
    #TODO: Its here to be used as based in the future if using adaptive learning
    def adjust_skills_and_preferences_learning(self, skill_updates, alpha=0.1):
        """
        Incrementally adjusts skills using learning rate alpha.
        new_value = old + alpha * (target - old)
        """

        for context, skills in skill_updates.items():
            for skill, (target_level, target_pref) in skills.items():

                old_level = self.skill_level[skill][context]
                old_pref = self.skill_preference[skill][context]

                self.skill_level[skill][context] = (
                    old_level + alpha * (target_level - old_level)
                )
                self.skill_preference[skill][context] = (
                    old_pref + alpha * (target_pref - old_pref)
                )

class Context:
    def __init__(self, name, relevance):
        self.name = name
        self.relevance = relevance  # dict[skill] -> R(s|c) = Skill relevance for task: [0.0, 1.0]