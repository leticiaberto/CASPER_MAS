import time
from src.communication.RobotCommunication import RobotComm

class CommunicationHandler:
    def __init__(self, agent, teamSize):
        self.agent = agent
        # Create communication
        self.comm = RobotComm(self.agent.id, teamSize)
    
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
        self.publish("skills_update", {"skill_weights": self.agent.skills.export_skill_weights()}, target=target)

    def send_constraints(self, target=None):
        self.publish("constraints_update", {"constraints":self.agent.constraints}, target=target)
    
    def publish_task_completed(self, task_id):
        msg = {
            "type": "TASK_COMPLETED",
            "task_id": task_id,
            "agent_id": self.agent.id
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
            print(f"[{self.agent.id}] Hello received from {sender}")

            # Reply directly with my skills
            self.send_skills(target=sender)
            self.send_constraints(target=sender)

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

            # Add partner if not already present
            if sender not in self.agent.partners:
                self.agent.add_partner(sender, received_weights, contexts)
                print(f"Created partner {sender} skills!")
            else:
                self.agent.partners[sender].skills.update_skills_and_preferences(received_weights)
                print(f"Updated partner {sender} skills!")


            print(f"[{self.agent.id}] Partner table updated: {list(self.agent.partners.keys())}")

            #self.partners[sender].skills.print_skills_preferences()

        elif msg_type == "task_assignment_batch":
            self.agent.update_global_graph(msg)
            self.agent.save_graphVisualization(self.agent.global_graph.G, output_name="data/Global_"+self.agent.id, palette_mode="pastel")
            self.agent.get_assigned_tasks()

        elif msg_type == "constraints_update":
            self.agent.partners[sender].constraints = msg["data"].get("constraints", None)
        
        elif msg_type == "TASK_COMPLETED":
            task_id = msg["task_id"]

            if msg.get("agent_id") != self.agent.id:
                # Register external completion
                self.agent.completed_external.add(task_id)
                if self.agent.local_graph.depends_on_external(task_id):
                    self.agent.local_graph.handle_external_completion(task_id)
        
        elif msg_type == "Rebuild_GlobalGraph":
            self.agent.update_global_graph(msg)
            self.agent.local_graph.rebuild(self.agent.global_graph)

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