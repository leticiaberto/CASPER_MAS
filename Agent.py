from Context import ContextualSkillModel
from Partner import PartnerAgent

class Agent:
    def __init__(self, id, constraints, skills, contexts):
        self.id = id
        self.constraints = constraints  # Task independent      
        self.skills = ContextualSkillModel(skills, contexts)
        
        self.assigned_tasks = []

        self.partners = {}

    def add_partner(self, partner_id, skills, contexts):
        self.partners[partner_id] = PartnerAgent(skills, contexts)

    def print_partners(self):
        print("------\n Partners of ", self.id)
        for pid, partner in self.partners.items():
            print(f"Partner_ID: {pid}")
            partner.print_partner_info()
        print("------")

    def evaluate(self, context):
        return sum(
            self.skills.contribution(
                skill=s,
                context=context.name,
                relevance=context.relevance.get(s, 0.0)
            )
            for s in context.relevance
        )