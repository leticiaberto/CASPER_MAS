import time
from src.entities.ContextualSkill import ContextualSkillModel
from src.entities.Partner import PartnerAgent
from src.communication.RobotCommunication import RobotComm
from src.entities.Supervisor import Supervisor
from src.graph.Global_Graph import GlobalGraph
from src.graph.GraphVisualizer import GraphVisualizer
from src.graph.Local_Graph import LocalGraph
from enum import Enum
from src.graph.TaskAssignment import TaskAssignment, TaskStatus
from abc import abstractmethod

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
            self.supervisor = Supervisor(name=f"Supervisor_{self.id}", agent = self, publish_fn=self.publish)            

        self.assigned_tasks = []

        self.partners = {}

        self.completed_external = set()

        # Create communication
        self.comm = RobotComm(self.id, self.teamSize)

        self.graph_visualizer = GraphVisualizer()

    # ----------------------------
    # Partners
    # ----------------------------
    def add_partner(self, partner_id, skills, contexts):
        self.partners[partner_id] = PartnerAgent(skills, contexts)

    def print_partners(self):
        print("------\n Partners of ", self.id)
        for pid, partner in self.partners.items():
            print(f"\nPartner_ID: {pid}")
            partner.print_partner_info()
        print("------")

    # ----------------------------
    # Load/Save data
    # ----------------------------
    def export_data(self):
        filename = "data/skills_preferences_" + self.id + ".csv"
        self.skills.export_skills_preferences_to_CSV(self.id, filename)# Export my own skills
        for pid, partner in self.partners.items():# Export partners skills
            partner.skills.export_skills_preferences_to_CSV(pid, filename)
    
    def save_graphVisualization(self, graph, output_name, graphType = "global", palette_mode="pastel"):
        if(graphType == "global"):
            self.graph_visualizer.export_multiagent_graph(graph,output_name, palette_mode)
        else:
            self.graph_visualizer.export_agent_task_graph(graph,output_name, palette_mode)
    def load_goal(self, task_file):
        # All agents know the task graph, but only the supervisor will score agents
        self.global_graph = GlobalGraph() # full DAG (read-only knowledge)
        self.global_graph.load_task_graph(task_file)

    # ----------------------------
    # Task allocation/execution
    # ----------------------------
    def allocate_task(self, agents, mode, top_k, debug=False):
        if(self.role == Roles.SUPERVISOR):
            self.supervisor.assign_agents_to_tasks(self.global_graph.G, agents, mode, top_k, debug)
            self.get_assigned_tasks()
            self.save_graphVisualization(self.global_graph.G, output_name="data/Global_"+self.id, palette_mode="pastel")
        else:
            print("Waiting Supervisor allocate task.")

    def get_assigned_tasks(self):
        self.local_graph = LocalGraph(self.id, self.global_graph.G, self.completed_external)
        self.save_graphVisualization(self.local_graph.graph, output_name="data/Local_"+self.id, graphType="local", palette_mode="pastel")
        '''
        for node_id in self.global_graph.G.nodes:
            if(self.global_graph.G.nodes[node_id]["assignment"].selected_agent == "default"):
                self.assigned_tasks.append(self.global_graph.G.nodes[node_id])
        
        for task in self.assigned_tasks:
            print(task["assignment"].task_id, task["assignment"].selected_agent)
        '''
    def step(self):
        self.save_graphVisualization(self.local_graph.graph, output_name="data/Local_"+self.id, palette_mode="pastel")
        ready_tasks = self.local_graph.get_ready_tasks()

        for task in ready_tasks:
            self.local_graph.mark_running(task)
            self.execute_task(task)
            self.local_graph.mark_done(task)
            self.publish_task_completed(task)

    def execute_task(self, task):
        """Template method (common workflow)."""
        self.current_task = task
        
        self.pre_execution(task)
        result = self._execute_task_specific(task)
        self.post_execution(task, result)
        
        return result
    
    def execute_task(self, task):

        # Tell graph we're running
        self.local_graph.mark_running(task)

        # Physical execution
        self.robot.execute(task)

        # Mark done
        self.local_graph.mark_done(task)

        # Notify other agents
        self.publish_task_completed(task)

    def pre_execution(self, task):
        print(f"[{self.id}] Preparing to execute {task.name}")

    def post_execution(self, task, result):
        print(f"[{self.id}] Finished {task.name} with result: {result}")

    @abstractmethod
    def _execute_task_specific(self, task):
        """Robot-specific implementation."""
        pass
    
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

    def send_constraints(self, target=None):
        self.publish("constraints_update", {"constraints":self.constraints}, target=target)
    
    def publish_task_completed(self, task_id):
        msg = {
            "type": "TASK_COMPLETED",
            "task_id": task_id,
            "agent_id": self.id
        }

        self.publish(msg)
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
            self.send_constraints(target=sender)

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
                self.partners[sender].skills.update_skills_and_preferences(received_weights)
                print(f"Updated partner {sender} skills!")


            print(f"[{self.id}] Partner table updated: {list(self.partners.keys())}")

            #self.partners[sender].skills.print_skills_preferences()

        elif msg_type == "task_assignment_batch":
            for task_id, assignment_data in msg["data"].items():
                assignment = TaskAssignment.deserialize(assignment_data)
                self.global_graph.G.nodes[task_id]["assignment"] = assignment
                #print(f"[{self.id}] Task {task_id} assigned to {assignment.selected_agent}")

            self.save_graphVisualization(self.global_graph.G, output_name="data/Global_"+self.id, palette_mode="pastel")
            self.get_assigned_tasks()

        elif msg_type == "constraints_update":
            self.partners[sender].constraints = msg["data"].get("constraints", None)
        
        elif msg_type == "TASK_COMPLETED":
            task_id = msg["task_id"]

            if msg.get("agent_id") != self.id:
                # Register external completion
                self.completed_external.add(task_id)
                if self.local_graph.depends_on_external(task_id):
                    self.local_graph.handle_external_completion(task_id)
        elif msg_type == "Rebuild_GlobalGraph":
            self.global_graph = msg["data"].get("graph", None)
            self.local_graph.rebuild(self.global_graph)

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

        # Step 3: broadcast my constraints once
        self.send_constraints()

        # Step 4: request everyone else's skills (optional, because they may have already sent them as a reply to hello)
        #self.request_skills()

    def closeComm(self):
        self.comm.close()


    # ----------------------------
    # Not used yet
    # ----------------------------
    def evaluate(self, context):
        return sum(
            self.skills.contribution(
                skill=s,
                context=context.name,
                relevance=context.relevance.get(s, 0.0)
            )
            for s in context.relevance
        )