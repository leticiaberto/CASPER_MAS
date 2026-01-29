from ContextualSkill import ContextualSkillModel
from Partner import PartnerAgent
from RobotCommunication import RobotComm

class Agent:
    def __init__(self, id, constraints, skills, contexts):
        self.id = id
        self.constraints = constraints  # Task independent      
        self.skills = ContextualSkillModel(skills, contexts)
        
        self.assigned_tasks = []

        self.partners = {}

        # Create communication
        self.comm = RobotComm(self.id)

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

    def export_data(self):
        filename = "data/skills_preferences_" + self.id + ".csv"
        self.skills.export_skills_preferences_to_CSV(self.id, filename)
        for pid, partner in self.partners.items():
            partner.skills.export_skills_preferences_to_CSV(pid, filename)

    # ----------------------------
    # Messaging Protocol
    # ----------------------------
    def send_hello(self):
        self.comm.broadcast("hello", {"Hello from ": self.id})

    def request_skills(self):
        self.comm.broadcast("skills_request", {})

    def send_skills(self, target=None):
        """
        Send skills_update.
        If target is set, only that robot processes it.
        """
        payload = {
            "skill_weights": self.skills.export_skill_weights()
        }

        if target is not None:
            payload["target"] = target

        self.comm.broadcast("skills_update", payload)

    # ----------------------------
    # Listener Callback
    # ----------------------------
    def on_message(self, msg):
        msg_type = msg["type"]
        sender = msg["from"]

        # ----------------------------
        # HELLO
        # ----------------------------
        if msg_type == "hello":
            print(f"[{self.id}] Hello received from {sender}")

            # Reply directly with my skills
            self.send_skills(target=sender)

        # ----------------------------
        # SKILLS REQUEST
        # ----------------------------
        elif msg_type == "skills_request":
            print(f"[{self.id}] Skills requested by {sender}")

            # Reply only to requester
            self.send_skills(target=sender)

        # ----------------------------
        # SKILLS UPDATE
        # ----------------------------
        elif msg_type == "skills_update":

            # Directed update?
            target = msg["data"].get("target", None)

            # Ignore if not meant for me
            if target is not None and target != self.id:
                return

            print(f"[{self.id}] Skills update received from {sender}")

            received_weights = msg["data"]["skill_weights"]
            contexts = list(received_weights.keys())

            # Add partner if not already present
            if sender not in self.partners:
                self.add_partner(sender, received_weights, contexts)
                print(f"Created partner {sender} skills!")
            else:
                #partner_agent = robot.partners[sender] # Could update in case receive new info
                print(f"Updated partner {sender} skills!")


            print(f"[{self.id}] Partner table updated: {list(self.partners.keys())}")

            #self.partners[sender].skills.print_skills_preferences()

    # ----------------------------
    # Startup Procedure
    # ----------------------------
    def startup(self):
        """
        Late join safe startup:
          1. Start listener
          2. Announce hello
          3. Send my skills
          4. Request skills from others 
        """

        self.comm.start_listener(self.on_message)

        # Step 1: announce join
        self.send_hello()

        # Step 2: broadcast my skills once
        self.send_skills()

        # Step 3: request everyone else's skills (optional, because they may have already sent them as a reply to hello)
        #self.request_skills()

    def closeComm(self):
        self.comm.close()