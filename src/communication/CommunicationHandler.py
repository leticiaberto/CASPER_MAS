from utils import Roles
from src.graph.TaskAssignment import TaskStatus

class CommunicationHandler:
    def __init__(self, agent, comm):
        self.agent = agent
        # Create communication
        self.comm = comm
    
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
        self.publish("hello", {"msg": f"Hello from {self.agent.id}"})

    def request_skills(self):
        self.publish("skills_request", {})

    def send_skills(self, target=None):
        """
        Send skills_update.
        If target is set, only that robot processes it.
        """
        self.publish("skills_update", {
            "skill_weights": self.agent.skills.export_skill_weights(),
            "role": self.agent.role.value,
            "constraints": self.agent.constraints
        }, target=target)
        
    def send_constraints(self, target=None):
        self.publish("constraints_update", {"constraints": self.agent.constraints}, target=target)
    
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
            print(f"[{self.agent.id}] Hello received from {sender}")

            # Reply directly with my skills
            self.send_skills(target=sender)
            #self.send_constraints(target=sender) # Now everything is inside send_skills

        # ----------------------------
        # SKILLS REQUEST
        # ----------------------------
        elif msg_type == "skills_request":
            print(f"[{self.agent.id}] Skills requested by {sender}")

            # Reply only to requester
            self.send_skills(target=sender)

        # ----------------------------
        # SKILLS UPDATE
        # ----------------------------
        elif msg_type == "skills_update":

            # Directed update?
            target = msg["data"].get("target", None)

            # Ignore if not meant for me
            if target is not None and target != self.agent.id:
                return

            print(f"[{self.agent.id}] Skills update received from {sender}")

            received_weights = msg["data"]["skill_weights"]
            contexts = list(received_weights.keys())
            role = msg["data"].get("role", None)

            if role is not None:
                role = Roles(role)  # deserialize back to enum

            constraints = msg["data"].get("constraints", {})
            
            self.agent.partners_skills_update(sender, received_weights, contexts, role, constraints)

        elif msg_type == "task_assignment_batch":
            self.agent.get_task_assignment_batch(msg)

        elif msg_type == "constraints_update":
            new = msg["data"].get("constraints", None)
            self.agent.partners_constratints_update(sender, new)
        
        elif msg_type == "task_status_update":
            task_id = msg["data"]["task_id"]
            target = msg["data"]["target"]
            task_status = TaskStatus.from_wire(msg["data"]["status"])

            # Ignore if not meant for me
            if target is not None and target != self.agent.id:
                return

            self.agent.update_task_status_received_general(task_id, task_status)

        elif msg_type == "task_status_update_supervisor":
            task_id = msg["data"]["task_id"]
            target = msg["data"]["target"]
            task_status = TaskStatus.from_wire(msg["data"]["status"])
            
             # Ignore if not meant for me
            if target is not None and target != self.agent.id:
                return
            self.agent.update_task_status_received_supervisor(task_id, task_status)

        elif msg_type == "task_ready_clearance":
            task_id = msg["data"]["task_id"]
            target = msg["data"].get("target", None)
 
            # Ignore if not meant for me
            if target is not None and target != self.agent.id:
                return
 
            self.agent.handle_task_clearance(task_id)

            
        elif msg_type == "Rebuild_GlobalGraph":
            self.agent.update_global_graph(msg)
            self.agent.local_graph.rebuild(self.agent.global_graph)

        elif msg_type == "SUPERVISOR":
            self.agent.set_supervisor(msg["data"]["supervisor_id"])

        elif msg_type == "all_tasks_done": 
            if sender == self.agent.supervisor_id:
                print(f"[{self.agent.id}] Received all_tasks_done message. Shutting down.")
                self.agent.goal_finished = True
