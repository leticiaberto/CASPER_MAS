import rclpy

class ROSContextManager:
    _instance = None
    _ref_count = 0

    @classmethod
    def acquire(cls):
        if cls._instance is None:
            if not rclpy.ok():
                rclpy.init()
            cls._instance = True
            print("[ROSContext] rclpy initialized")

        cls._ref_count += 1
        print(f"[ROSContext] ref_count={cls._ref_count}")

    @classmethod
    def release(cls):
        cls._ref_count -= 1
        print(f"[ROSContext] ref_count={cls._ref_count}")
        if cls._ref_count <= 0:
            rclpy.shutdown()
            cls._instance = None
            cls._ref_count = 0
            print("[ROSContext] rclpy shutdown")