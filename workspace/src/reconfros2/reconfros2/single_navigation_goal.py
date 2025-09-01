import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion

class NavigationGoalPublisher(Node):
    def __init__(self):
        super().__init__('single_navigation_goal')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.navigation_goals_ = []  # List to hold navigation goals
        self.current_goal_index_ = 0  # Index of the current goal
        self.timer_ = self.create_timer(1, self.publish_next_goal)

        # Example navigation goals
        # -1.97, -1.76 <-- starting position
        self.navigation_goals_.append((0.57, 0.60))  # Example goal 1

    def publish_next_goal(self):
        if self.current_goal_index_ < len(self.navigation_goals_):
            goal = self.navigation_goals_[self.current_goal_index_]
            goal_msg = PoseStamped()
            goal_msg.pose.position.x = goal[0]
            goal_msg.pose.position.y = goal[1]
            goal_msg.pose.position.z = 0.0  # Assuming z=0 for simplicity
            goal_msg.pose.orientation = Quaternion(w=1.0)  # Default orientation
            goal_msg.header.frame_id = "map"  # Set the frame ID to "map"
            self.publisher_.publish(goal_msg)
            self.current_goal_index_ += 1
        else:
            self.get_logger().info('All navigation goals sent.')
            # Stop publishing goals or handle the end of navigation goals

def main(args=None):
    rclpy.init(args=args)
    navigation_goal_publisher = NavigationGoalPublisher()
    rclpy.spin(navigation_goal_publisher)
    navigation_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
