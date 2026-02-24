from src.entities.ContextualSkill import ContextualSkillModel
from src.entities.Partner import PartnerAgent
from src.communication.CommunicationHandler import CommunicationHandler
from src.communication.RobotCommunication import RobotComm
from src.entities.Supervisor import Supervisor
from src.graph.Global_Graph import GlobalGraph
from src.graph.GraphVisualizer import GraphVisualizer
from src.graph.Local_Graph import LocalGraph
from src.graph.TaskAssignment import TaskAssignment, TaskStatus
from abc import abstractmethod
from utils import Roles
import time
class Agent:
    def __init__(self, id, constraints, skills, contexts, role, teamsize):
        self.id = id
        self.constraints = constraints  # Task independent      
        self.skills = ContextualSkillModel(skills, contexts)
        self.supervisor_id = None
        self.role = Roles(role)
        self.teamSize = teamsize

        # Create communication
        self.comm = RobotComm(self.id, self.teamSize)
        self.comm_handler = CommunicationHandler(self, self.comm)
        
        if(self.role == Roles.SUPERVISOR):
            self.supervisor = Supervisor(name=f"Supervisor_{self.id}", agent = self, publish_fn=self.comm_handler.publish)            
        
        self.assigned_tasks = []

        self.partners = {}

        self.graph_visualizer = GraphVisualizer()

        self.supervisor_id = None

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
    
    def save_graphVisualization(self, graph, output_name, palette_mode="pastel"):
        if not graph: # graph is empty
            return
        self.graph_visualizer.export_multiagent_graph(graph,output_name, palette_mode)

    def load_goal(self, task_file):
        # All agents know the task graph, but only the supervisor will score agents
        self.global_graph = GlobalGraph() # full DAG (read-only knowledge)
        self.global_graph.load_task_graph(task_file)

    # ----------------------------
    # Task allocation/execution
    # ----------------------------
    def allocate_task(self, agents, mode, top_k, debug=False):
        if(self.role == Roles.SUPERVISOR):
            self.comm_handler.publish("SUPERVISOR", {"supervisor_id": self.id})
            self.supervisor_id = self.id
            self.supervisor.assign_agents_to_tasks(self.global_graph.G, agents, mode, top_k, debug)
            self.save_graphVisualization(self.global_graph.G, output_name="data/["+self.id+"] Global", palette_mode="pastel")
        else:
            print("Waiting Supervisor allocate task.")

    def get_assigned_tasks(self):
        self.local_graph = LocalGraph(self.id, self.global_graph.G)
        self.graph_visualizer.plot_task_graph(self.local_graph.graph, "data/["+self.id+"] Local_")
        for node_id in self.local_graph.graph.nodes:
            self.publish_task_status_update(node_id, self.local_graph.graph.nodes[node_id]["status"])
            time.sleep(3)

    def step(self):
        ready_tasks = self.local_graph.get_ready_tasks()
        if not ready_tasks:
            pass
        else:
            #print(f"Ready tasks for execution: {ready_tasks}")
            # Inform all the tasks ready to be executed (to improve explanation and trust)
            for task in ready_tasks:
                self.publish_task_status_update(task, TaskStatus.READY) #Do not need to update local because get_ready_tasks() does
                time.sleep(6)
                self.graph_visualizer.plot_task_graph(self.local_graph.graph, "data/["+self.id+"] Local_")
            # Execute each ready task
            for task in ready_tasks:
                self.local_graph.update_status(task, TaskStatus.RUNNING)
                self.publish_task_status_update(task, TaskStatus.RUNNING)
                self.graph_visualizer.plot_task_graph(self.local_graph.graph, "data/["+self.id+"] Local_")
                time.sleep(3)
                self._execute_task_specific(task) # Physical execution
                time.sleep(10)
                self.local_graph.update_status(task, TaskStatus.DONE)
                self.publish_task_status_update(task, TaskStatus.DONE)
                self.graph_visualizer.plot_task_graph(self.local_graph.graph, "data/["+self.id+"] Local_")
                # Supervisor update its local graph and also the global one
                if(self.role == Roles.SUPERVISOR):
                    print("Supervisor updating global graph")
                    self.save_graphVisualization(self.global_graph.G, output_name="data/["+self.id+"] Global_", palette_mode="pastel")
                time.sleep(2)               

    @abstractmethod
    def _execute_task_specific(self, task):
        """Robot-specific implementation."""
        pass
    
    # ----------------------------
    # Graph stuff
    # ----------------------------
    def update_global_graph(self, msg):
        for task_id, assignment_data in msg["data"].items():
            assignment = TaskAssignment.deserialize(assignment_data)
            self.global_graph.G.nodes[task_id]["assignment"] = assignment
            #print(f"[{self.id}] Task {task_id} assigned to {assignment.selected_agent}")

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
    
    # ----------------------------
    # Used in the message protocol
    # ----------------------------
    def partners_skills_update(self, sender, received_weights, contexts):
        # Add partner if not already present
        if sender not in self.partners:
            self.add_partner(sender, received_weights, contexts)
            print(f"Created partner {sender} skills!")
        else:
            self.partners[sender].skills.update_skills_and_preferences(received_weights)
            print(f"Updated partner {sender} skills!")

        print(f"[{self.id}] Partner table updated: {list(self.partners.keys())}")

        #self.partners[sender].skills.print_skills_preferences()

    def partners_constratints_update(self, sender, new):
        self.partners[sender].constraints = new

    def get_task_assignment_batch(self, msg):
            self.update_global_graph(msg)
            self.save_graphVisualization(self.global_graph.G, output_name="data/["+self.id+"] Global_", palette_mode="pastel")
            self.get_assigned_tasks()       

    def set_supervisor(self, supervisor_id):
        print(supervisor_id)
        self.supervisor_id = supervisor_id

    def publish_task_status_update(self, task_id, task_status):
        # Always notify supervisor
        self.comm_handler.publish("task_status_update_supervisor", {"task_id": task_id, "agent_id": self.id, "status":task_status.to_wire()}, self.supervisor_id)
        
        # Notify only agents that depend on this task
        successor_agents = self.local_graph.get_successors(task_id)
        # Send one message per agent
        for agent_id in successor_agents:
            self.comm_handler.publish("task_status_update", {"task_id": task_id, "agent_id": self.id, "status": task_status.to_wire()}, agent_id)
        
    def update_task_status_received_general(self, task_id, task_status):
        #print(f"Received update that task {task_id} is now {task_status.value}")
        if self.local_graph.depends_on_external(task_id):
            self.local_graph.handle_external_completion(task_id, task_status)

    def update_task_status_received_supervisor(self, task_id, task_status):
            #print("Supervisor updating global graph")
            self.global_graph.update_status(task_id, task_status)
            self.save_graphVisualization(self.global_graph.G, output_name="data/["+self.id+"] Global_", palette_mode="pastel")


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
        self.comm.start_listener(self.comm_handler.on_message)

        time.sleep(1.0)

        # Step 1: announce join
        self.comm_handler.send_hello()

        # Step 2: broadcast my skills once
        self.comm_handler.send_skills()

        # Step 3: broadcast my constraints once
        self.comm_handler.send_constraints()

        # Step 4: request everyone else's skills (optional, because they may have already sent them as a reply to hello)
        #self.comm_handler.request_skills()

    
    def closeComm(self):
        self.comm.close()