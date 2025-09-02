import sys, time, csv, os, math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Path

class NavigateToPoseActionClient(Node):
    feedback = None
    x_pos = None
    y_pos = None

    def __init__(self):
        super().__init__('rl4greenros_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Params
        self.declare_parameter('nav_goal', 0)
        self.declare_parameter('timeout_sec', 180)
        self.declare_parameter('plan_topic', '/plan')  # change to '/planner_server/plan' if needed
        self.timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().integer_value
        self.plan_topic = self.get_parameter('plan_topic').get_parameter_value().string_value

        nav_goal = self.get_parameter('nav_goal').get_parameter_value().integer_value
        self.get_logger().info(f'nav_goal value: {nav_goal} | timeout: {self.timeout_sec}s')

        switcher = {
            1: (-2.50, 1.5),
            2: (-2.50, -1.25),
            3: (-0.35, -0.80),
            4: (1.50, -1.00),
            5: (2.50, 0.50),
            6: (0.6, 0.50)
        }
        navigation_coords = switcher.get(nav_goal, "Invalid position")
        if navigation_coords == "Invalid position":
            self.get_logger().error('Invalid nav_goal value')
            rclpy.shutdown()
            return
        self.x_pos, self.y_pos = navigation_coords

        # Timeout state
        self.goal_handle = None
        self.goal_start_time = None
        self.timeout_timer = None
        self.timed_out = False

        # Planned path length (first plan only)
        self.planned_distance_m = None   # None until captured
        self.awaiting_plan = False       # flip to True when goal accepted
        self.create_subscription(Path, self.plan_topic, self._on_plan, 10)

        self.csv_path = '/data/nav2_performance.csv'

    def _on_plan(self, msg: Path):
        # Only record the FIRST plan after the goal is accepted
        if not self.awaiting_plan:
            return
        if not msg.poses:
            return
        d = 0.0
        for i in range(1, len(msg.poses)):
            x0, y0 = msg.poses[i-1].pose.position.x, msg.poses[i-1].pose.position.y
            x1, y1 = msg.poses[i].pose.position.x, msg.poses[i].pose.position.y
            d += math.hypot(x1 - x0, y1 - y0)
        self.planned_distance_m = d
        self.awaiting_plan = False  # lock it: ignore replans
        self.get_logger().info(f"[FIRST PLAN] planned_distance_m={self.planned_distance_m:.3f} m")

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_done)
            return

        self.get_logger().info('Goal accepted :)')
        self.goal_start_time = time.time()

        # Start capturing ONLY the first plan for this goal
        self.awaiting_plan = True

        # Start timeout watchdog
        self.timeout_timer = self.create_timer(1.0, self.check_timeout)

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def check_timeout(self):
        if self.goal_start_time is None or self.timed_out:
            return
        elapsed = time.time() - self.goal_start_time
        if elapsed > self.timeout_sec:
            self.timed_out = True
            self.get_logger().warn(f"Timeout after {self.timeout_sec}s. Canceling goal and logging as failure…")
            if self.goal_handle:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._on_cancel_done)
            if self.timeout_timer is not None:
                self.timeout_timer.cancel()

    def _on_cancel_done(self, _):
        if self.feedback is not None:
            navigation_time = int(self.feedback.feedback.navigation_time.sec)
            recoveries = int(self.feedback.feedback.number_of_recoveries)
        else:
            navigation_time = int(time.time() - self.goal_start_time) if self.goal_start_time else 0
            recoveries = 0

        success = 0
        self._write_results(success, navigation_time, recoveries, note='timeout')
        self.get_logger().info("Results saved (timeout). Shutting down.")
        rclpy.shutdown()

    def feedback_callback(self, feedback):
        self.feedback = feedback

    def get_result_callback(self, future):
        if self.timed_out:
            return
        status = future.result().status
        success = 1 if status == GoalStatus.STATUS_SUCCEEDED else 0

        if self.feedback is not None:
            navigation_time = int(self.feedback.feedback.navigation_time.sec)
            recoveries = int(self.feedback.feedback.number_of_recoveries)
        else:
            navigation_time = int(time.time() - self.goal_start_time) if self.goal_start_time else 0
            recoveries = 0

        self._write_results(success, navigation_time, recoveries, note='result')
        rclpy.shutdown()

    def _write_results(self, success, navigation_time, recoveries, note=''):
        header = ['success', 'navigation_time', 'recoveries', 'planned_distance_m']
        value = round(self.planned_distance_m, 4) if self.planned_distance_m is not None else -1.0
        new_file = not os.path.exists(self.csv_path) or os.path.getsize(self.csv_path) == 0
        with open(self.csv_path, 'a', newline='') as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(header)
            w.writerow([success, navigation_time, recoveries, value])
        self.get_logger().info(f"Results ({note}) saved to {self.csv_path}")

    def send_goal(self, x_pos, y_pos):
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPose action server not available after waiting")
            self.destroy_node()
            rclpy.shutdown()
            return

        # Reset the stored planned distance for THIS goal
        self.planned_distance_m = None
        self.awaiting_plan = False  # will flip to True once goal is accepted

        self.get_logger().info('Sending goal request...')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x_pos
        goal_msg.pose.pose.position.y = y_pos
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToPoseActionClient()
    node.send_goal(node.x_pos, node.y_pos)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
