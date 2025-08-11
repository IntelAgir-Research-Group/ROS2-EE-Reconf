import rclpy
from rclpy.node import Node
import time

# Import tf2 and the necessary utilities
import tf2_ros
import math

# Import the subprocess module to run the actual_pose script
import subprocess

# Import the exception for external shutdown
from rclpy.executors import ExternalShutdownException

import sys
class LocalisationErrorListener(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('localization_error_listener')

        # Set up the list to store distances
        self.distances = []
        self.num_samples = 0
        self.num_resets = 0
        self.average_distance = 0
        self.reset_cooldown = 0

        # Create a buffer and listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # CSV style header
        print('timestamp,localization_error,avg_loc_error,num_loc_resets')

        # Set a timer to periodically check for the transform
        self.create_timer(0.1, self.check_transform)  # Check every 1 second

    def check_transform(self):
        # Get the transform from /odom to /map
        try:
            base_map = self.tf_buffer.lookup_transform('base_footprint', 'map', rclpy.time.Time())
            base_odom = self.tf_buffer.lookup_transform('base_footprint', 'odom', rclpy.time.Time())
            
            # Calculate the position distance between the two
            distance_to_map = math.sqrt((base_map.transform.translation.x - base_odom.transform.translation.x) ** 2 + 
                                         (base_map.transform.translation.y - base_odom.transform.translation.y) ** 2)

            # Calculate the orientation (heading) in radians
            map_yaw = self.get_yaw_from_quaternion(base_map.transform.rotation)
            odom_yaw = self.get_yaw_from_quaternion(base_odom.transform.rotation)

            # Calculate the angle difference
            angle_difference = self.normalize_angle(map_yaw - odom_yaw)

            # Calculate the combined localization error
            combined_error = math.sqrt(distance_to_map**2 + angle_difference**2)
            
            # Update the rolling average
            self.average_distance = (self.average_distance * self.num_samples + combined_error) / (self.num_samples + 1)
            self.num_samples += 1

            if combined_error > 2.0 and self.reset_cooldown == 0:
                # Reset the localization
                subprocess.Popen(['bash', '-c', 'ros2 run rl4greenros actual_pose' '> /dev/null 2>&1'])
                self.reset_cooldown = 50
                self.num_resets += 1

            self.reset_cooldown = max(0, self.reset_cooldown - 1)

            # Append to the list
            self.distances.append(combined_error)
            timestamp = int(time.time())
            print(f'{timestamp},{combined_error},{self.average_distance},{self.num_resets}')
            
            # flush the output
            sys.stdout.flush()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # ignore if the transform is not available
            pass 


    def get_yaw_from_quaternion(self, quaternion):
        """Convert quaternion to yaw angle (z-axis rotation) manually."""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        # Calculate yaw (phi)
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(sin_yaw, cos_yaw)
        
        return yaw  # yaw in radians

    def normalize_angle(self, angle):
        """Normalize angle to be within [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = LocalisationErrorListener()
    try:
        rclpy.spin(node)
    except:
        pass
    node.destroy_node()

    try:
        rclpy.shutdown()
    except:
        pass

if __name__ == '__main__':
    main()
