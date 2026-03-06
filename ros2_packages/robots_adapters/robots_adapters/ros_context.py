import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

class ROSContextManager:
    _ref_count = 0
    _executor = None
    _thread = None
    _lock = threading.Lock()

    @classmethod
    def acquire(cls):
        with cls._lock:
            if cls._ref_count == 0:
                rclpy.init()
                cls._executor = MultiThreadedExecutor()
                cls._thread = threading.Thread(
                    target=cls._executor.spin,
                    daemon=True
                )
                cls._thread.start()
            cls._ref_count += 1

    @classmethod
    def release(cls):
        with cls._lock:
            cls._ref_count -= 1
            if cls._ref_count == 0:
                cls._executor.shutdown()
                rclpy.shutdown()

    @classmethod
    def add_node(cls, node):
        cls._executor.add_node(node)

    @classmethod
    def remove_node(cls, node):
        cls._executor.remove_node(node)