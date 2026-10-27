from src.models.ContextualSkill import ContextualSkillModel
from src.models.Trust import TrustModel

# It is a class because I'm modeling beliefs about other agents, not the agents themselves.
class PartnerAgent:
    def __init__(self, skills, contexts, role):
        self.skills = ContextualSkillModel(skills, contexts)
        self.trust = TrustModel(skills, contexts)
        self.constraints = {} 
        self.workspace = []
        self.role = role
    
    def print_partner_info(self):
        def fmt(value):
            return f"{value:.2f}" if value is not None else "  - "

        all_pairs = self.skills.pairs() | self.trust.pairs()

        current_skill = None
        for skill, context in sorted(all_pairs):
            if skill != current_skill:
                print(f"\nSkill: {skill}")
                current_skill = skill

            level, pref = self.skills.get(skill, context)
            trust = self.trust.get(skill, context)
            print(
                f"  {context:<10} | "
                f"Level: {fmt(level)} | "
                f"Pref: {fmt(pref)} | "
                f"Trust: {fmt(trust)}"
            )

        print("\nConstraints:")
        for constraint in self.constraints.items():
            print(constraint)

        print("\nWorkspace:")
        for item in self.workspace:
            print(f"  {item}")

    def get_partner_info(self):
        info = {
            "skills": self.skills.export_skill_weights(),
            "trust": self.trust.export_trust_scores(),
            "constraints": self.constraints,
            "workspace": self.workspace,
            "role": self.role
        }
        return info
            
    def update_trust_level(self, context, reward, lr=0.05):
        """
        Update trust level of a partner agent based on task outcomes
        Increase trust if tasks are successful, decrease if they fail.
        """
        self.trust[context] += lr * (reward - self.trust[context])

    def update_partner_skills(self, other_agent):
        """ 
        Update estimated skills of a partner agent based on observations 
        Initially set the estimated skills to the actual level skills indicate by the partner and then refine based on task performance over time.
        """
    pass


