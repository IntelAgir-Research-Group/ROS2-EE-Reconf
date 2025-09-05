import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def wait_for_sim_time(self, timeout=10.0):
        # If sim time is used, wait until /clock is active
        if not self.get_clock().ros_time_is_active:
            start = time.time()
            self.get_logger().info("Waiting for /clock (use_sim_time) ...")
            while not self.get_clock().ros_time_is_active:
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - start > timeout:
                    self.get_logger().warn("Timed out waiting for /clock; using wall time.")
                    return False
        return True

    def wait_for_subscriber(self, timeout=10.0):
        start = time.time()
        self.get_logger().info("Waiting for a subscriber on /initialpose ...")
        while self.publisher_.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().warn("No subscriber on /initialpose within timeout.")
                return False
        return True

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # <-- current time (not a fixed old stamp!)
        msg.header.frame_id = 'map'

        # Pose
        msg.pose = PoseWithCovariance()
        msg.pose.pose = Pose()
        msg.pose.pose.position = Point(x=0.0, y=5.0, z=0.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=-2.108866730152919e-06, w=0.9999999999977763)

        # Covariance (non-zero yaw variance helps AMCL)
        msg.pose.covariance = [
            0.25, 0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.25, 0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.06853892
        ]

        # Publish a few times to be robust
        for i in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published initial pose ({i+1}/5)')
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        node.wait_for_sim_time(timeout=10.0)         # don’t publish with stale/zero time
        node.wait_for_subscriber(timeout=10.0)       # avoid publishing to nobody
        node.publish_initial_pose()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
