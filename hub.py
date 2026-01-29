import zmq


def run_hub(pub_port=5555, sub_port=5556):
    ctx = zmq.Context()

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