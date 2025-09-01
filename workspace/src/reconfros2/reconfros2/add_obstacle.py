import time
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class AddObstacle(Node):

    def __init__(self):
        super().__init__('add_obstacle_node')
        self.spawn_entity = self.create_client(SpawnEntity, '/spawn_entity')

        # Getting the parameter from the ROS2 run command 
        self.declare_parameter('num_obstacles', 0)
        self.num_obstacles = self.get_parameter('num_obstacles').get_parameter_value().integer_value
        self.get_logger().info(f'num_obstacles value: {self.num_obstacles}')

        self.declare_parameter('nav_goal', 0)
        self.nav_goal = self.get_parameter('nav_goal').get_parameter_value().integer_value
        self.get_logger().info(f'nav_goal value: {self.nav_goal}')

        # Declare the 'use_sim_time' parameter with a default value
        if not self.has_parameter('use_sim_time'):
          self.declare_parameter('use_sim_time', True)

        # Get the value of the 'use_sim_time' parameter
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        if use_sim_time:
            self.get_logger().info('Using simulation time (use_sim_time is set to True)')
        else:
            self.get_logger().info('Using system time (use_sim_time is set to False)')


    def add_obstacle(self, num_obstacles, nav_goal):
      if num_obstacles not in [1, 2]:
          self.get_logger().error("Number of obstacles should be 1 or 2")
          return
      elif nav_goal not in range(1,6):
          self.get_logger().error("Nav goal should be between be 1 and 5")
          return
      
      print(f'Obstacles: {num_obstacles}, Nav Goal: {nav_goal}')

      obstacles = {
        1: [(-0.6, 1.5), (-1.5, 1.0)],
        2: [(-0.9, 1.1), (-0.75, -0.5)],
        3: [(-4.5, 3.8), (-2.6, 1.2)],
        4: [(1.0, 0.1), (1.5, -0.6)],
        5: [(1.7, 1.5), (2.5, 1.0)]
      }

      positions = obstacles.get(nav_goal)

      for i in range(num_obstacles):
          
          time.sleep(2.5*(i+1))
          
          position = positions[i]

          print(f'Adding obstacle to the posisiton {position}')

          self.spawn_entity.wait_for_service()
          request = SpawnEntity.Request()
          request.name = f"obstacle_cylinder_{nav_goal}_{i}"
          request.xml = """
          <sdf version='1.4'>
              <model name='obstacle_cylinder'>
                <static>0</static>
                <link name='link'>
                  <pose>0 0 1 0 0 0</pose>
                  <collision name='collision'>
                    <geometry>
                      <cylinder>
                        <radius>0.1</radius> <!-- Set a smaller radius -->
                        <length>0.2</length>  <!-- Set a smaller length -->
                      </cylinder>
                    </geometry>
                  </collision>
                  <visual name='visual'>
                    <geometry>
                      <cylinder>
                        <radius>0.1</radius> <!-- Match the smaller radius -->
                        <length>0.2</length>  <!-- Match the smaller length -->
                      </cylinder>
                    </geometry>
                  </visual>
                </link>
              </model>
            </sdf>
          """

          request.initial_pose = Pose()
          request.initial_pose.position.x = position[0]
          request.initial_pose.position.y = position[1]
          request.initial_pose.position.z = 0.0

          future = self.spawn_entity.call_async(request)
          rclpy.spin_until_future_complete(self, future)    

def main(args=None):
    rclpy.init(args=args)
    add_obstacle_node = AddObstacle()

    add_obstacle_node.add_obstacle(add_obstacle_node.num_obstacles, add_obstacle_node.nav_goal)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
