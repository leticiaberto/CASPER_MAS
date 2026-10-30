from src.models.ContextualSkill import ContextualSkillModel
from src.models.Trust import TrustModel

# It is a class because I'm modeling beliefs about other agents, not the agents themselves.
class PartnerAgent:
    def __init__(self, skills, contexts, role):
        self.skills = ContextualSkillModel(skills, contexts)
        self.trust = TrustModel(contexts)
        self.constraints = {} 
        self.workspace = []
        self.role = role
        self.contexts = contexts

    def update_from(self, episode):
        self.trust.update(episode)
        #self.skills.update(episode)
    
    def restore_from(self, entries):
        """
        Load stored beliefs (chronological) from episodic memory. Works for the initial load and for
        new entries added later, because the evidence count continues from what is already known.
        Skill values are restored only when learned from evidence, so a fresh self-report from the
        partner is not overwritten by an old self-report.
        """
        for e in entries:
            skill, context = e["skill"], e["context"]
            if self.contexts is not None and context not in self.contexts:
                continue
            n = self.trust.get_count(skill, context) + (1 if e["is_evidence"] else 0)
            if n > 0 and e["skill_value"] is not None:
                self.skills.set(skill, context, e["skill_value"], e["preference"])
            if e["trust"] is not None:
                self.trust.restore(skill, context, e["trust"],
                                   source="direct" if n > 0 else "prior", count=n)
                
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

    def update_partner_skills(self, other_agent):
        """ 
        Update estimated skills of a partner agent based on observations 
        Initially set the estimated skills to the actual level skills indicate by the partner and then refine based on task performance over time.
        """
    pass


