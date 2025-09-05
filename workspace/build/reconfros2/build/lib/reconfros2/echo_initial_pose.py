import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import PoseWithCovarianceStamped  # Adjust this import to your topic's message type

class EchoInitialPose(Node):
    def __init__(self):
        super().__init__('echo_initial_pose')
        self.topic_name = '/initialpose'
        self.subscription = None
        self.msg_received = False

    def wait_for_topic(self, timeout=None):
        start_time = time.time()
        while rclpy.ok():
            topic_list = self.get_topic_names_and_types()
            topics = [name for name, _ in topic_list]
            if self.topic_name in topics:
                self.get_logger().info(f"Topic {self.topic_name} is now being published.")
                self.create_subscription(PoseWithCovarianceStamped, self.topic_name, self.callback, 10)
                return True
            if timeout and (time.time() - start_time) > timeout:
                self.get_logger().warn(f"Timeout of {timeout} seconds reached. Topic {self.topic_name} not found.")
                return False
            time.sleep(0.5)

    def callback(self, msg):
        self.get_logger().info(f"Received message: {msg}")
        self.msg_received = True

def main(args=None):
    rclpy.init(args=args)
    node = EchoInitialPose()
    
    try:
        if node.wait_for_topic(timeout=30):  # Wait for up to 30 seconds
            while rclpy.ok() and not node.msg_received:
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
