from src.entities.ContextualSkill import ContextualSkillModel
from src.entities.Partner import PartnerAgent
from src.communication.CommunicationHandler import CommunicationHandler
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

        # Create communication
        self.comm_handler = CommunicationHandler(self, teamsize)
        
        if(self.role == Roles.SUPERVISOR):
            self.supervisor = Supervisor(name=f"Supervisor_{self.id}", agent = self, publish_fn=self.comm_handler.publish)            

        self.assigned_tasks = []

        self.partners = {}

        self.completed_external = set()

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
    # Graph stuff
    # ----------------------------
    def update_global_graph(self,msg):
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