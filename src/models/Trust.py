from collections import defaultdict

class TrustModel:
    def __init__(self, skill_weights, contexts):
        self.trust_scores = defaultdict(dict)  # Dictionary to hold trust scores for other agents
        self.start_trust(skill_weights, contexts)

    def start_trust(self, skill_weights, contexts, default=0):
        #Wrong for now, it should be retrieving from episodic memory or based on the agent skill level informed
        for context, skills in skill_weights.items():
            for skill, weight in skills.items():
                self.trust_scores[skill][context] = weight[0]

        # Fill missing contexts with default values
        for skill in self.trust_scores:
            for context in contexts:
                if context not in self.trust_scores[skill]:
                    self.trust_scores[skill][context] = default

    def update_trust(self, skill_updates, contexts=None, clamp=True):
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
                    level, _ = values
                    if clamp:
                        level = max(0.0, min(1.0, float(level)))
    
                    # Ensure skill exists
                    if skill not in self.trust_scores:
                        self.trust_scores[skill] = {}
    
                    # Update values
                    self.trust_scores[skill][context] = level

    def print_trust_scores(self):# Not in use anymore, but left in case I change my mind
        for skill, contexts in self.trust_scores.items():
            print(f"\nSkill: {skill}")
            for context in contexts:
                print(
                    f"  {context:<10} | "
                    f"Level: {self.trust_scores[skill][context]:.2f}"
                )

    def export_trust_scores(self):
        """
        Export trust scores in the same format expected by start_trust.
        Returns:
            dict[context][skill] = trust_score
        """
        result = {}

        for skill in self.trust_scores:
            for context in self.trust_scores[skill]:
                if context not in result:
                    result[context] = {}

                trust_score = self.trust_scores[skill][context]
                result[context][skill] = trust_score

        return result
    
    def get(self, skill, context):
        return self.trust_scores.get(skill, {}).get(context)

    def pairs(self):
        return {(s, c) for s, ctxs in self.trust_scores.items() for c in ctxs}