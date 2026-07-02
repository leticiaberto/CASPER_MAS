import queue
from src.graph.RealTimeGraphVisualizer import RealTimeGraphVisualizer
from src.entities.ContextualSkill import ContextualSkillModel
from src.entities.Partner import PartnerAgent
from src.communication.CommunicationHandler import CommunicationHandler
from src.communication.RobotCommunication import RobotComm
from src.entities.Supervisor import Supervisor
from src.graph.Global_Graph import GlobalGraph
from src.graph.GraphVisualizer import GraphVisualizer
from src.graph.Local_Graph import LocalGraph
from src.graph.TaskAssignment import TaskAssignment, TaskStatus
from src.analysis.ExperimentLogger import ExperimentLogger
from abc import abstractmethod
from utils import Roles
import time

class Agent:
    def __init__(self, id, constraints, skills, contexts, role, teamsize, party_duration, guests, run_id, log_dir="experiment_logs"):
        self.id = id
        self.constraints = constraints  # Task independent      
        self.skills = ContextualSkillModel(skills, contexts)
        self.supervisor_id = None
        self.role = Roles(role)
        self.teamSize = teamsize
        self.party_duration = party_duration
        self.guests = guests

        # Create communication
        self.comm = RobotComm(self.id, self.teamSize)
        self.comm_handler = CommunicationHandler(self, self.comm)

        # Experiment analytics: every agent gets its own logger pointed at
        # the same run_id/output_dir so per-agent execution-order rows from
        # every team member land in the same shared CSV (task_execution.csv)
        # and can be told apart by run_id when comparing experiments.
        # run_id should be passed in (e.g. derived once per experiment and
        # shared across all agents + the supervisor) so all rows from the
        # same run carry a matching identifier; if omitted, each agent
        # would otherwise generate its own random run_id.
        self.experiment_logger = ExperimentLogger(run_id=run_id, output_dir=log_dir)
        
        if(self.role == Roles.SUPERVISOR):
            self.supervisor = Supervisor(name=f"Supervisor_{self.id}", agent = self, publish_fn=self.comm_handler.publish, run_id=self.experiment_logger.run_id, log_dir=log_dir)
            # Reuse the supervisor's logger (same run_id) as this agent's
            # own, so the supervisor's own task executions land under one
            # consistent run_id instead of two separate ExperimentLogger
            # instances that happen to share an output_dir.
            self.experiment_logger = self.supervisor.experiment_logger           
            
        self.assigned_tasks = []

        self.partners = {}

        self.graph_visualizer = GraphVisualizer()

        self.supervisor_id = None

        self.goal_finished = False

        self.shutdown_reason = None

        # Message queue: listener thread enqueues, main thread drains in step()
        self._msg_queue = queue.Queue()

    # ----------------------------
    # Partners
    # ----------------------------
    def add_partner(self, partner_id, skills, contexts, role):
        self.partners[partner_id] = PartnerAgent(skills, contexts, role)

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
        filename = "skills_preferences_" + self.id + ".csv"
        self.skills.export_skills_preferences_to_CSV(self.id, filename)# Export my own skills
        for pid, partner in self.partners.items():# Export partners skills
            partner.skills.export_skills_preferences_to_CSV(pid, filename)
    
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
            self.supervisor.assign_agents_to_tasks(self.global_graph.G, agents, mode, top_k, debug, self.id)
            self.graph_visualizer.export_multiagent_graph(self.global_graph.G, output_name=f"["+self.id+"] Global_Supervisor_Allocation", palette_mode="pastel")
            self.vis_queue = queue.Queue()
            visualizer = RealTimeGraphVisualizer(
                self.global_graph.G,
                self.vis_queue
            )
            visualizer.start()
        else:
            print("Waiting Supervisor allocate task.")

    def get_assigned_tasks(self):
        self.local_graph = LocalGraph(self.id, self.global_graph.G)
        self.graph_visualizer.plot_task_graph(self.local_graph.graph, "["+self.id+"] Local_")
        for node_id in self.local_graph.graph.nodes:
            self.publish_task_status_update(node_id, self.local_graph.graph.nodes[node_id]["status"])
            print(f"Published initial status of task {node_id} as {self.local_graph.graph.nodes[node_id]['status'].value}")
            time.sleep(10)
        print("Assigned tasks received and local graph initialized.")

    def spin_once(self):
        """
            Drain every queued message on the main thread.
            Call this anywhere the main thread needs to process incoming
            messages: at the top of step(), and in the Robot.py startup
            polling loop while waiting for partners to announce themselves.
        """
        while not self._msg_queue.empty():
            try:
                msg = self._msg_queue.get_nowait()
                self.comm_handler.on_message(msg)
            except queue.Empty:
                break

    def step(self):
        self.spin_once()  # process all pending messages before acting

        if not hasattr(self, "local_graph"):
            return
        
        ready_tasks = self.local_graph.get_ready_tasks()

        if ready_tasks:
            #  READY announce used for explainability/trust on the receiving end (other agents)
            for task in ready_tasks:
                #print(f"Ready task: {t} with priority {self.local_graph.graph.nodes[t].get('priority', 'N/A')}")
                self.task_update_status_and_publish(task, TaskStatus.READY, ignore=True)# Do not need to update local because get_ready_tasks() does
                time.sleep(5)

            # Pick the single highest-priority (lowest number) ready task
            task = min(
                ready_tasks,
                key=lambda t: self.local_graph.graph.nodes[t].get("priority", float('inf'))
            )

            task_started_at = time.time()
            self.task_update_status_and_publish(task, TaskStatus.RUNNING)
            time.sleep(5)
            self._execute_task_specific(task)# Physical execution
            time.sleep(5)
            task_finished_at = time.time()
            self.task_update_status_and_publish(task, TaskStatus.DONE)

            # Experiment analytics: record the order in which THIS agent
            # executed its tasks, plus a "supervisor view" snapshot of the
            # task's position in the global DAG (predecessors/successors,
            # bottleneck/source/leaf, how many downstream tasks it just
            # unblocked) -- so later analysis can relate per-agent execution
            # order to each task's actual importance to the team, not just
            # to this agent. self.global_graph.G is the full DAG every
            # agent holds read-only (see load_goal()); on non-supervisor
            # agents its status fields reflect the last
            # task_assignment_batch / task_status_update_supervisor
            # messages received, so they may lag slightly behind the
            # supervisor's own copy -- structure (predecessors/successors)
            # is always accurate, status-derived fields (e.g.
            # successors_unblocked_by_this) are best-effort.
            #
            # started_at/finished_at (unix timestamps) are what make a
            # Gantt chart possible later: one bar per task, from started_at
            # to finished_at, grouped by agent_id.
            self.experiment_logger.log_task_execution(
                self.id, task, self.global_graph.G,
                extra_fields={"started_at": task_started_at, "finished_at": task_finished_at},
            )

            time.sleep(10)
        else:
            print("No Tasks READY to execute")
            
        if self.role == Roles.SUPERVISOR:
            # Release any time_to_clean tasks once all subgoal_level==1 tasks are done
            subgoal_level_1_nodes = [
                node_id for node_id in self.global_graph.G.nodes
                if self.global_graph.G.nodes[node_id].get("subgoal_level") == 1
                and not self.global_graph.G.nodes[node_id].get("at_end", False)
            ]
            all_subgoals_done = bool(subgoal_level_1_nodes) and all(
                self.global_graph.G.nodes[node_id].get("assignment") is not None
                and self.global_graph.G.nodes[node_id]["assignment"].status == TaskStatus.DONE
                for node_id in subgoal_level_1_nodes
            )
            if all_subgoals_done and not self.supervisor.release_clean_msg:
                print(f"[{self.id}] All subgoal_level 1 tasks are done. Releasing time_to_clean tasks.")
                for node_id in self.global_graph.G.nodes:
                    node_data = self.global_graph.G.nodes[node_id]
                    assignment = node_data.get("assignment")
                    if (
                        node_data.get("time_to_clean") is False
                        and assignment.status != TaskStatus.NOT_ASSIGNED
                        and assignment.selected_agent is not None
                        and assignment.status not in (TaskStatus.RUNNING, TaskStatus.DONE)
                    ):
                        print(f"[{self.id}] All subgoal_level 1 tasks done. Releasing clean task '{node_id}'.")
                        self.release_task(node_id)
                        # Flip the flag on the global graph so we don't re-release next step
                        self.global_graph.G.nodes[node_id]["time_to_clean"] = True
                self.supervisor.release_clean_msg = True
                
                # The supervisor doesn't necessarily know (or need to know) which
                # agent is the host — broadcast so every agent gets a chance to react.
                # Only an agent that overrides _on_party_ending() (e.g. Human) will
                # actually do anything with it.
                self.comm_handler.publish("party_over", {"supervisor_id": self.id})
                # Broadcasts aren't delivered back to the sender (same convention as
                # the "SUPERVISOR" / "all_tasks_done" messages above), so also fire
                # the hook locally in case this agent is itself the host.
                self._on_party_ending()
 
            if self.check_all_tasks_done():# Check everytime in case one can change the status back
                print("All tasks are done!")
                self.comm_handler.publish("all_tasks_done", {"agent_id": self.id})
                time.sleep(5)
                self.goal_finished = True

    def task_update_status_and_publish(self, task, new_status, ignore=False):
        if not ignore:
            self.local_graph.update_status(task, new_status)
        self.publish_task_status_update(task, new_status)
        self.graph_visualizer.plot_task_graph(self.local_graph.graph, "["+self.id+"] Local_")
                
    def check_all_tasks_done(self):
        all_done = all(
            self.global_graph.G.nodes[n]["assignment"].status == TaskStatus.DONE
            for n in self.global_graph.G.nodes
        )
        return all_done

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

    # ----------------------------
    # Used in the message protocol
    # ----------------------------
    def partners_skills_update(self, sender, received_weights, contexts, role, constraints):
        # Add partner if not already present
        if sender not in self.partners:
            self.add_partner(sender, received_weights, contexts, role)
            print(f"Created partner {sender} skills!")
        else:
            self.partners[sender].skills.update_skills_and_preferences(received_weights)
            print(f"Updated partner {sender} skills!")
        
        self.partners[sender].constraints = constraints  # <-- always apply
        print(f"[{self.id}] Partner table updated: {list(self.partners.keys())}")

        self.partners[sender].skills.print_skills_preferences()

    def partners_constratints_update(self, sender, new):
        self.partners[sender].constraints = new

    def get_task_assignment_batch(self, msg):
            self.update_global_graph(msg)
            self.graph_visualizer.export_multiagent_graph(self.global_graph.G,output_name="["+self.id+"] Global_Member_Allocation", palette_mode="pastel")
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
            self.vis_queue.put(("refresh",))
            self.global_graph.update_status(task_id, task_status)
            self.graph_visualizer.export_multiagent_graph(self.global_graph.G,output_name="["+self.id+"] Global_Supervisor_StatusUpdated", palette_mode="pastel", graphType="updated")

            # Runtime idle check: every time any task's status changes
            # anywhere on the team, re-evaluate whether some agent now has
            # nothing READY/RUNNING to do while at least one other agent
            # does. Only meaningful on the supervisor (it's the only agent
            # whose global_graph reflects everyone's live status), and
            # logs to idle_events.csv via self.experiment_logger.
            if(task_status == TaskStatus.RUNNING): # it makes sense only check for other idle agent if the message is that someone is running something
                self.experiment_logger.check_and_log_idle_agents(self.global_graph.G, trigger_task_id=task_id, trigger_agent_id=self.id)

    def handle_task_clearance(self, task_id):
        """
        Called by CommunicationHandler when a task_ready_clearance message arrives.
        Flips time_to_clean on the local graph and, if all other deps are met,
        marks the task READY and publishes the status update.
        """
        became_ready = self.local_graph.receive_clearance(task_id)
        if became_ready:
            print(f"[{self.id}] Task '{task_id}' cleared by supervisor and is now READY.")
            self.publish_task_status_update(task_id, TaskStatus.READY)
            self.graph_visualizer.plot_task_graph(self.local_graph.graph, "[" + self.id + "] Local_")
        else:
            print(f"[{self.id}] Task '{task_id}' received clearance but is not yet READY (deps still pending).")
 
    def handle_party_over(self):
        """
        Called by CommunicationHandler when a 'party_over' message arrives
        from the supervisor. Any agent may receive this broadcast; it's
        forwarded to _on_party_ending() so only agents that override that
        hook (e.g. Human, as the host) actually react to it.
        """
        self._on_party_ending()

    def handle_shutdown(self, reason="unknown", unassigned_tasks=None):
        """
        Called by CommunicationHandler when a 'shutdown' message arrives from
        the supervisor (e.g. allocation failed because some task had no
        capable agent). Reuses goal_finished so the existing Robot.py
        loop-exit / cleanup path fires unchanged, but records *why* we
        stopped so logs/callers can tell an abort apart from a normal
        "all_tasks_done" completion.
        """
        self.shutdown_reason = reason
        self.shutdown_unassigned_tasks = unassigned_tasks or []
        self.goal_finished = True

    def release_task(self, task_id):
        """
        Supervisor-side convenience: look up the assigned agent and send clearance.
        Only callable when this agent holds the supervisor role.
        """
        if self.role != Roles.SUPERVISOR:
            raise RuntimeError("Only the supervisor agent can release tasks.")
 
        assignment = self.global_graph.G.nodes[task_id].get("assignment")
        if assignment is None or assignment.selected_agent is None:
            raise ValueError(f"Task '{task_id}' has no assigned agent.")
 
        if assignment.selected_agent != self.id:
            self.supervisor.release_task(task_id, assignment.selected_agent)
        else:
            self.task_update_status_and_publish(task_id, TaskStatus.PENDING)
            time.sleep(5)

    def _on_party_ending(self):
        """
        Hook called exactly once, on the supervisor, the moment the party
        duration elapses. No-op by default; subclasses (e.g. Human) can
        override to trigger their own behavior (e.g. guests leaving).
        """
        pass

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

        # Add partners — listener thread only enqueues; main thread drains
        self.comm.start_listener(self._msg_queue)

        time.sleep(1.0)

        # Step 1: announce join
        self.comm_handler.send_hello()

        # Step 2: broadcast my skills and constraints once
        self.comm_handler.send_skills()

        # Step 4: request everyone else's skills (optional, because they may have already sent them as a reply to hello)
        #self.comm_handler.request_skills()

        # Start the party timer so step() can release time_to_clean tasks after party_duration seconds
        self._start_time = time.time()

    
    def closeComm(self):
        self.comm.close()