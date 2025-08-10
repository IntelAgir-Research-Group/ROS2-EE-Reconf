import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
import numpy as np
import time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(1.0, self.publish_initial_pose)

    def publish_initial_pose(self):
        # Create and populate the PoseWithCovarianceStamped message
        msg = PoseWithCovarianceStamped()

        # Set the header
        msg.header = Header()
        msg.header.stamp = Time(sec=1714778807, nanosec=917830182)
        msg.header.frame_id = 'map'

        # Set the pose with covariance
        msg.pose = PoseWithCovariance()
        msg.pose.pose = Pose()
        msg.pose.pose.position = Point(x=1.0, y=1.9, z=0.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=-2.108866730152919e-06, w=0.9999999999977763)

        # Set the covariance (as a flat list)
        covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892
        ]
        msg.pose.covariance = covariance

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published message: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()

    try:
        #rclpy.spin(node)
        for i in range(2): # making sure the message is read
            rclpy.spin_once(node)
            time.sleep(5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
