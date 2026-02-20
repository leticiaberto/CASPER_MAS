import zmq
import time
import threading
import yaml


def load_network_config(path="configs/network.yaml"):
    with open(path, "r") as f:
        config = yaml.safe_load(f)
    hub_ip = config.get("hub_ip", "127.0.0.1")
    pub_port = config.get("pub_port", 5555)
    sub_port = config.get("sub_port", 5556)
    return hub_ip, pub_port, sub_port


class RobotComm:
    """
    RobotComm: broadcast communication with selective multi-ACK support
    for 'task_assign' and 'task_update' message types.
    """

    ACK_TYPES = {"_task_assignment_batch", "task_update"}  # Only these trigger ACKs

    def __init__(self, robot_id: str, team_size: int = 1, ignore_self: bool = True):
        self.robot_id = robot_id
        self.ignore_self = ignore_self
        self.team_size = team_size

        hub_ip, pub_port, sub_port = load_network_config()

        self.ctx = zmq.Context()
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.connect(f"tcp://{hub_ip}:{pub_port}")

        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect(f"tcp://{hub_ip}:{sub_port}")
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")

        time.sleep(0.5)
        print(f"[{self.robot_id}] Connected to hub at {hub_ip} (team_size={self.team_size})")

        self.processed_requests = {}

    # ----------------------------------------------------
    # BROADCAST WITH MULTI-ACK AND RESEND (for selective types)
    # ----------------------------------------------------
    def broadcast(self, msg_type: str, data: dict, wait_ack=True, ack_timeout=3.0, max_retries=3):
        """
        Broadcast a message and wait for ACKs only if type is in ACK_TYPES.
        Resends message for missing ACKs up to max_retries.
        """
        message_id = f"{self.robot_id}_{msg_type}_{time.time()}"
        message = {
            "from": self.robot_id,
            "type": msg_type,
            "data": data,
            "time": time.time(),
            "id": message_id,
        }

        # If message does not require ACK, just send once
        if msg_type not in self.ACK_TYPES or not wait_ack or self.team_size <= 1:
            self.pub.send_json(message)
            return True

        retries = 0
        received_acks = set()

        while retries <= max_retries:
            self.pub.send_json(message)
            print(f"[{self.robot_id}] Sent '{msg_type}', attempt {retries + 1}")

            start_time = time.time()
            while time.time() - start_time < ack_timeout:
                ack_msg = self.receive(blocking=False)
                if ack_msg:
                    if (
                        ack_msg.get("type") == f"ack_{msg_type}"
                        and ack_msg.get("to") == self.robot_id
                    ):
                        received_acks.add(ack_msg.get("from"))
                        if len(received_acks) >= self.team_size - 1:
                            return True

            missing = self.team_size - 1 - len(received_acks)
            if missing > 0:
                print(f"[{self.robot_id}] Missing ACKs from {missing} robots. Resending...")
                retries += 1
            else:
                break

        print(f"[{self.robot_id}] Could not get all ACKs after {max_retries} retries.")
        return False

    # ----------------------------------------------------
    # RECEIVE
    # ----------------------------------------------------
    def receive(self, blocking=True, timeout=0.0):
        """
        Receive the next message.

        Automatically ignores self messages if enabled.

        Returns:
            dict message or None if no message is available in non-blocking mode
        """
        while True:
            try:
                if not blocking:
                    # Set timeout for non-blocking mode
                    self.sub.setsockopt(zmq.RCVTIMEO, int(timeout * 1000))
                else:
                    # Blocking mode: wait indefinitely
                    self.sub.setsockopt(zmq.RCVTIMEO, -1)

                msg = self.sub.recv_json()
            except zmq.Again:
                # Non-blocking timeout reached
                return None
            #print(msg)
            # Ignore self messages except 'task_status_update' to update the global graph
            if self.ignore_self and msg.get("from") == self.robot_id:
                if msg.get("type") != "task_status_update_supervisor" and msg.get("type") != "task_assignment_batch":
                    continue

            
            # Avoid duplicate processing
            msg_id = msg.get("id")
            now = time.time()

            # remove expired
            self.processed_requests = {
                k: v for k, v in self.processed_requests.items()
                if now - v < 30 # Keep this request if it was processed less than 30 seconds ago.
            }

            # Ignore if already handled
            if msg_id and msg_id in self.processed_requests:
                continue
            if msg_id:
                # Mark as processed
                self.processed_requests[msg_id] = now

            return msg

    # ----------------------------------------------------
    # BACKGROUND LISTENER WITH SELECTIVE ACK
    # ----------------------------------------------------
    def start_listener(self, callback):
        def loop():
            while True:
                msg = self.receive()
                if msg:
                    # Only send ACK for selected message types
                    if "type" in msg and "from" in msg and msg["type"] in self.ACK_TYPES:
                        ack = {
                            "from": self.robot_id,
                            "to": msg["from"],
                            "type": f"ack_{msg['type']}",
                            "time": time.time(),
                        }
                        self.pub.send_json(ack)
                    callback(msg)

        thread = threading.Thread(target=loop, daemon=True)
        thread.start()

    # ----------------------------------------------------
    # CLEANUP
    # ----------------------------------------------------
    def close(self):
        self.pub.close()
        self.sub.close()
        self.ctx.term()
        print(f"[{self.robot_id}] Communication closed.")
