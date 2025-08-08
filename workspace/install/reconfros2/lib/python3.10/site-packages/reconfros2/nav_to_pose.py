import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import random
from nav_msgs.msg import Odometry
import math
import time
import sys

class NavToPose(Node):
    def __init__(self):
        super().__init__('nave_to_pose')

        self.navigation_goals_ = []  # List to hold navigation goals
        self.current_goal_index_ = 0  # Index of the current goal
        self.goal_reached_ = False  # Flag to indicate if the current goal is reached
        self.distance_treshold_ = 0.5
        self.first_goal_ = True

        self.goal_start_time_ = None  # Track when the goal was issued
        self.goal_timeout_ = 90.0     # Timeout in seconds

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

        self.navigation_goals_.append(switcher.get(nav_goal, "Invalid position"))

        # Setting up the publisher and subscriber topics
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription_ = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            1
        )

        # Making sure the topic is published
        for i in range(5):
            self.publish_next_goal()
            time.sleep(1)

    def odometry_callback(self, msg):
        if self.current_goal_index_ < len(self.navigation_goals_):
            current_goal = self.navigation_goals_[self.current_goal_index_]
            current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            self.distance_to_goal_ = math.sqrt(
                (current_pose[0] - current_goal[0]) ** 2 +
                (current_pose[1] - current_goal[1]) ** 2
            )

            if self.distance_to_goal_ <= self.distance_treshold_:
                self.get_logger().info('Reached destination')
                self.goal_reached_ = True

            # --- TIMEOUT LOGIC ---
            elif self.goal_start_time_ is not None:
                elapsed_time = time.time() - self.goal_start_time_
                if elapsed_time > self.goal_timeout_:
                    self.get_logger().warn(f"Goal {self.current_goal_index_} timed out after {elapsed_time:.1f}s.")
                    self.goal_reached_ = True  # Treat as reached to proceed to next goal

            # Move to next goal if current was reached or timed out
            if self.goal_reached_:
                self.goal_reached_ = False
                self.current_goal_index_ += 1
                self.publish_next_goal()


    def publish_next_goal(self):
        if self.current_goal_index_ < len(self.navigation_goals_):
            self.get_logger().info('Navigating to the goal.')
            self.get_logger().info(str(self.current_goal_index_))
            goal = self.navigation_goals_[self.current_goal_index_]
            goal_msg = PoseStamped()
            goal_msg.pose.position.x = goal[0]
            goal_msg.pose.position.y = goal[1]
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation = Quaternion(w=1.0)
            goal_msg.header.frame_id = "map"
            self.publisher_.publish(goal_msg)

            # Save the time the goal was sent
            self.goal_start_time_ = time.time()

        else:
            self.get_logger().info('All navigation goals sent.')
            sys.exit()
    
def main(args=None):
    rclpy.init(args=args)
    navigation_goal_publisher = NavToPose()
    try:
        rclpy.spin(navigation_goal_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    navigation_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
