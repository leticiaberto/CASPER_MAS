import time
from src.entities.ContextualSkill import ContextualSkillModel
from src.entities.Partner import PartnerAgent
from src.communication.RobotCommunication import RobotComm
from src.entities.Supervisor import Supervisor
from src.graph.Graph import Graph, GraphVisualizer
from enum import Enum
from src.graph.TaskAssignment import TaskAssignment, TaskStatus


class Roles(Enum):
    SUPERVISOR = "supervisor"
    MEMBER = "member"

class Agent:
    def __init__(self, id, constraints, skills, contexts, role, teamsize):
        self.id = id
        self.constraints = constraints  # Task independent      
        self.skills = ContextualSkillModel(skills, contexts)
        self.supervisor = None
        self.role = Roles(role)
        self.teamSize = teamsize

        if(self.role == Roles.SUPERVISOR):
            self.supervisor = Supervisor(name=f"Supervisor_{self.id}", publish_fn=self.publish)            

        self.assigned_tasks = []

        self.partners = {}

        # Create communication
        self.comm = RobotComm(self.id, self.teamSize)

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
        self.skills.export_skills_preferences_to_CSV(self.id, filename)# Export my own skills
        for pid, partner in self.partners.items():# Export partners skills
            partner.skills.export_skills_preferences_to_CSV(pid, filename)

    def allocate_task(self, agents, mode, top_k, debug=False):
        if(self.role == Roles.SUPERVISOR):
            self.supervisor.assign_agents_to_tasks(self.task_graph.G, agents, mode, top_k, debug)
            self.supervisor.graph_visualizer.export_multiagent_graph(self.task_graph.G, output_name="data/"+self.id, palette_mode="pastel")
        else:
            print("Waiting Supervisor allocate task.")
    
    def print_graph(self):
        graph_visualizer = GraphVisualizer()
        graph_visualizer.export_multiagent_graph(self.task_graph.G,output_name="data/"+self.id, palette_mode="pastel")

    def load_goal(self, task_file):
        # All agents know the task graph, but only the supervisor will score agents (the others can, but will not do it here for simplicity)
        self.task_graph = Graph()
        self.task_graph.load_task_graph(task_file)
    # ----------------------------
    # Messaging Protocol
    # ----------------------------
    def publish(self, msg_type, data, target=None):
        payload = data.copy()

        if target is not None:
            payload["target"] = target

        self.comm.broadcast(msg_type, payload)

    def send_hello(self):
        # self.comm.broadcast("hello", {"Hello from ": self.id})
        self.publish("hello", {"msg": f"Hello from {self.id}"})

    def request_skills(self):
        self.publish("skills_request", {})

    def send_skills(self, target=None):
        """
        Send skills_update.
        If target is set, only that robot processes it.
        """
        self.publish("skills_update", {"skill_weights": self.skills.export_skill_weights()}, target=target)

    # ----------------------------
    # Listener Callback
    # ----------------------------
    def on_message(self, msg):
        msg_type = msg["type"]
        sender = msg["from"]
        """
        request_id = msg.get("data", {}).get("id")
        now = time.time()

        # remove expired
        self.processed_requests = {
            k: v for k, v in self.processed_requests.items()
            if now - v < 30 # Keep this request if it was processed less than 30 seconds ago.
        }

        # Ignore if already handled
        if request_id in self.processed_requests:
            return

        # Mark as processed
        self.processed_requests[request_id] = now
        """
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
                #partner_agent = self.partners[sender] # Could update in case receive new info
                #self.partners[sender].skills.
                print(f"Updated partner {sender} skills!")


            print(f"[{self.id}] Partner table updated: {list(self.partners.keys())}")

            #self.partners[sender].skills.print_skills_preferences()

        elif msg["type"] == "task_assignment_batch":
            for task_id, assignment_data in msg["data"].items():
                assignment = TaskAssignment.deserialize(assignment_data)
                self.task_graph.G.nodes[task_id]["assignment"] = assignment
                #print(f"[{self.id}] Task {task_id} assigned to {assignment.selected_agent}")

            self.print_graph()
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

        # Add partners
        self.comm.start_listener(self.on_message)

        time.sleep(1.0)

        # Step 1: announce join
        self.send_hello()

        # Step 2: broadcast my skills once
        self.send_skills()

        # Step 3: request everyone else's skills (optional, because they may have already sent them as a reply to hello)
        #self.request_skills()

    def closeComm(self):
        self.comm.close()