import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import action_msgs.msg


class NavigateToPoseWithFeedback(Node):

    def __init__(self):
        super().__init__('navigate_to_pose_with_feedback')
        self.action_client = self.create_action_client(NavigateToPose, 'navigate_to_pose')
        self.feedback_received = False

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback: {}'.format(feedback))
        self.feedback_received = True

    def send_goal(self, pose):
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting...')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.action_client.wait_for_server()
        self.action_client.send_goal(goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Goal sent: {}'.format(goal_msg))

    async def main(self):
        pose = PoseStamped()
        # Set your desired pose here
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.orientation.w = 1.0  # Facing forward
        self.send_goal(pose)
        while rclpy.ok() and not self.feedback_received:
            await self.action_client.wait_for_feedback()

def main(args=None):
    rclpy.init(args=args)
    navigate_to_pose_with_feedback = NavigateToPoseWithFeedback()
    rclpy.spin(navigate_to_pose_with_feedback.main())
    navigate_to_pose_with_feedback.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
