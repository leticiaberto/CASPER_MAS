import yaml
import socket

def get_network_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def update_yaml_config(filename="../configs/network.yaml"):
    """
    Get the current machine's IP address and update the 'hub_ip' field in the specified YAML configuration file.
    """
    current_ip = get_network_ip()
    
    # 1. Read existing data
    try:
        with open(filename, 'r') as file:
            config = yaml.safe_load(file) or {}
    except FileNotFoundError:
        config = {}

    # 2. Update the hub_ip field
    config['hub_ip'] = current_ip

    # 3. Write back to file
    with open(filename, 'w') as file:
        yaml.dump(config, file, default_flow_style=False, sort_keys=False)
    
    print(f"Updated {filename} with hub_ip: {current_ip}")

if __name__ == "__main__":
    update_yaml_config()