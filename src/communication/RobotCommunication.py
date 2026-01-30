import zmq
import time
import threading
import yaml

def load_network_config(path="configs/network.yaml"):
    """
    Loads the network configuration for the robot communication hub.

    Expected YAML format:

    hub_ip: 192.168.1.50
    pub_port: 5555
    sub_port: 5556
    """

    with open(path, "r") as f:
        config = yaml.safe_load(f)

    hub_ip = config.get("hub_ip", "127.0.0.1")
    pub_port = config.get("pub_port", 5555)
    sub_port = config.get("sub_port", 5556)

    return hub_ip, pub_port, sub_port

class RobotComm:
    """
    RobotComm: broadcast communication for multi-robot systems.

    Each robot can:
        - broadcast messages to all robots
        - receive messages from all other robots
        - automatically ignore its own messages by default

    Requires:
        A ZeroMQ relay hub running in the network.
    """

    def __init__(
        self,
        robot_id: str,
        ignore_self: bool = True,
    ):
        self.robot_id = robot_id

        # Load hub settings from YAML
        hub_ip, pub_port, sub_port = load_network_config()

        # Default behavior: ignore messages sent by itself
        self.ignore_self = ignore_self

        self.ctx = zmq.Context()

        # Publisher socket (robot → hub)
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.connect(f"tcp://{hub_ip}:{pub_port}")

        # Subscriber socket (hub → robot)
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect(f"tcp://{hub_ip}:{sub_port}")

        # Subscribe to all messages
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")

        # Allow sockets to connect properly
        time.sleep(0.5)

        print(
            f"[{self.robot_id}] Connected to hub at {hub_ip} "
            f"(ignore_self={self.ignore_self})"
        )

    # ----------------------------------------------------
    # BROADCAST
    # ----------------------------------------------------
    def broadcast(self, msg_type: str, data: dict):
        """
        Broadcast a message to all robots.

        Example:
            comm.broadcast("task_done", {"task": "pickup"})
        """
        message = {
            "from": self.robot_id,
            "type": msg_type,
            "data": data,
            "time": time.time(),
        }

        self.pub.send_json(message)

    # ----------------------------------------------------
    # RECEIVE
    # ----------------------------------------------------
    def receive(self, blocking=True, timeout=0.0):
        """
        Receive the next message.

        Automatically ignores self messages if enabled.

        Returns:
            dict message or None
        """

        while True:
            # --- Non-blocking mode ---
            if not blocking:
                self.sub.setsockopt(zmq.RCVTIMEO, int(timeout * 1000))
                try:
                    msg = self.sub.recv_json()
                except zmq.Again:
                    return None

            # --- Blocking mode ---
            else:
                msg = self.sub.recv_json()

            # Ignore own messages
            if self.ignore_self and msg.get("from") == self.robot_id:
                continue

            return msg

    # ----------------------------------------------------
    # BACKGROUND LISTENER
    # ----------------------------------------------------
    def start_listener(self, callback):
        """
        Start a background thread that listens continuously.

        callback(msg) is called whenever another robot sends a message.
        """

        def loop():
            while True:
                msg = self.receive()
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
