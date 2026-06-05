from src.entities.ContextualSkill import ContextualSkillModel

# It is a class because I'm modeling beliefs about other agents, not the agents themselves.
class PartnerAgent:
    def __init__(self, skills, contexts, role, init_trust=0.5):
        self.skills = ContextualSkillModel(skills, contexts)
        self.trust = {c: init_trust for c in contexts}
        self.constraints = {} 
        self.role = role
    
    def print_partner_info(self):
        self.skills.print_skills_preferences()
        print("Trust Levels:")
        for context, trust_level in self.trust.items():
            print(f"  Context: {context}, Trust Level: {trust_level}")

        print("Constraints:")
        for constraint in self.constraints.items():
            print(constraint)
            
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


