import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import random
from nav_msgs.msg import Odometry
import math
import time

class NavigationGoalPublisher(Node):
    def __init__(self):
        super().__init__('multi_navigation_goals')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            1
        )
        self.navigation_goals_ = []  # List to hold navigation goals
        self.current_goal_index_ = 0  # Index of the current goal
        self.goal_reached_ = False  # Flag to indicate if the current goal is reached
        self.distance_treshold_ = 0.5
        self.first_goal_ = True

        # Example navigation goals
        # -1.97, -1.76 <-- starting position
        #self.navigation_goals_.append((-2.50, 1.5))  # Pos 1
        #self.navigation_goals_.append((-2.50, -1.25))  # Pos 2
        #self.navigation_goals_.append((-0.35, -0.80))  # Pos 3
        #self.navigation_goals_.append((1.50, -1.00))  # Pos 4
        #self.navigation_goals_.append((2.50, 0.50))  # Pos 5
        self.navigation_goals_.append((0.6, 0.50))  # Pos 6

    def odometry_callback(self, msg):
        if self.goal_reached_:  # Define a threshold for reaching the goal
            self.get_logger().info('Reached destination')
            self.get_logger().info('Sleeping for 30 seconds')
            time.sleep(30)
            self.current_goal_index_ += 1
            self.publish_next_goal()
            self.goal_reached_ = False
        else:
            if self.first_goal_:
                self.first_goal_ = False
                self.publish_next_goal()
            else:
                if self.current_goal_index_ < len(self.navigation_goals_):
                    current_goal = self.navigation_goals_[self.current_goal_index_]
                    current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
                    self.get_logger().info(str(current_pose[0]))
                    self.get_logger().info(str(current_goal[0]))
                    self.get_logger().info(str(current_pose[1]))
                    self.get_logger().info(str(current_goal[1]))
                    self.distance_to_goal_ = math.sqrt((current_pose[0] - current_goal[0]) ** 2 + (current_pose[1] - current_goal[1]) ** 2)
                    self.get_logger().info(str(self.distance_to_goal_))      
                    self.get_logger().info('------')
                    if self.distance_to_goal_ <= self.distance_treshold_:
                        self.goal_reached_=True
            # self.get_logger().info('killing odometry callback')

    def publish_next_goal(self):
        if self.current_goal_index_ < len(self.navigation_goals_):
            self.get_logger().info('Navigating to the goal: ')
            self.get_logger().info(str(self.current_goal_index_))
            goal = self.navigation_goals_[self.current_goal_index_]
            goal_msg = PoseStamped()
            goal_msg.pose.position.x = goal[0]
            goal_msg.pose.position.y = goal[1]
            goal_msg.pose.position.z = 0.0  # Assuming z=0 for simplicity
            goal_msg.pose.orientation = Quaternion(w=1.0)  # Default orientation
            goal_msg.header.frame_id = "map"  # Set the frame ID to "map"
            self.publisher_.publish(goal_msg)
        else:
            self.get_logger().info('All navigation goals sent.')

def main(args=None):
    rclpy.init(args=args)
    navigation_goal_publisher = NavigationGoalPublisher()
    rclpy.spin(navigation_goal_publisher)
    navigation_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
