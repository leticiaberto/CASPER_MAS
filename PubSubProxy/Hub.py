import yaml
import zmq

def load_network_config(path="../configs/network.yaml"):
    """
    Loads the network configuration for the robot communication hub.

    Expected YAML format:

    hub_ip: 192.168.1.50
    pub_port: 5555
    sub_port: 5556
    """

    with open(path, "r") as f:
        config = yaml.safe_load(f)

    pub_port = config.get("pub_port", 5555)
    sub_port = config.get("sub_port", 5556)

    return pub_port, sub_port

def run_hub():
    ctx = zmq.Context()

    pub_port, sub_port = load_network_config()

    frontend = ctx.socket(zmq.XSUB)
    frontend.bind(f"tcp://*:{pub_port}")

    backend = ctx.socket(zmq.XPUB)
    backend.bind(f"tcp://*:{sub_port}")

    print("=== Robot Relay Hub Running ===")
    print(f"PUB robots → hub: {pub_port}")
    print(f"HUB → SUB robots: {sub_port}")

    zmq.proxy(frontend, backend)


if __name__ == "__main__":
    run_hub()