import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import csv
import time

class NavigateToPoseActionClient(Node):

    feedback = None
    x_pos = None
    y_pos = None

    def __init__(self):
        super().__init__('rl4greenros_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Navigation timeout in seconds
        self.timeout_sec = 60  
        self.goal_start_time = None
        self.goal_handle = None

        # Getting the parameter from the ROS2 run command 
        self.declare_parameter('nav_goal', 0)
        nav_goal = self.get_parameter('nav_goal').get_parameter_value().integer_value
        self.get_logger().info(f'nav_goal value: {nav_goal}')

        switcher = {
            1: (-2.50, 1.5),
            2: (-2.50, -1.25),
            3: (-0.35, -0.80),
            4: (1.50, -1.00),
            5: (2.50, 0.50),
            6: (0.6, 0.50)
        }

        navigation_coords = switcher.get(nav_goal, "Invalid position")
        self.x_pos = navigation_coords[0]
        self.y_pos = navigation_coords[1]
        
    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Record start time for timeout check
        self.goal_start_time = time.time()

        # Start a timer to monitor timeout
        self.create_timer(1.0, self.check_timeout)

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def check_timeout(self):
        """Check if the navigation exceeded the timeout."""
        if self.goal_start_time and (time.time() - self.goal_start_time) > self.timeout_sec:
            self.get_logger().warn(f"Navigation timeout exceeded ({self.timeout_sec} sec). Canceling goal...")
            if self.goal_handle:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda _: self.get_logger().info("Goal canceled due to timeout"))
            self.goal_start_time = None  # Stop checking after cancel
    
    def feedback_callback(self, feedback):
        self.feedback = feedback

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        success = 0
        navigation_time = 0
        recoveries = 0
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            self.get_logger().info('Navigation time: {0}'.format(self.feedback.feedback.navigation_time.sec))
            self.get_logger().info('Recoveries: {0}'.format(self.feedback.feedback.number_of_recoveries))
            success = 1
        else:
            self.get_logger().info('Goal failed or canceled')
        
        if self.feedback:
            navigation_time = self.feedback.feedback.navigation_time.sec
            recoveries = self.feedback.feedback.number_of_recoveries
        
        # Save the variables as a CSV file
        data = [[success, navigation_time, recoveries]]
        filename = '/data/nav2_performance.csv'
        self.get_logger().info(f'Results saved to {filename}')

        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['success', 'navigation_time', 'recoveries'])
            writer.writerows(data)

        rclpy.shutdown()
        
    def send_goal(self, x_pos, y_pos):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal request...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x_pos
        goal_msg.pose.pose.position.y = y_pos
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
            )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

def main(args=None):
    rclpy.init(args=args)

    action_client = NavigateToPoseActionClient()
    action_client.send_goal(action_client.x_pos, action_client.y_pos)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
