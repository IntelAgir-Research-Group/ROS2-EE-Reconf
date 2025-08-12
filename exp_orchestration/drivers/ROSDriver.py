import os
import subprocess
import sys
import threading
from ament_index_python.packages import get_package_share_directory
import time

rr_path = os.getenv('RR_PATH')
sys.path.append(rr_path)

# Used to set Robot Runner Context
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext

class ROSDriver:
    configuration_root_dir = '/orchestrator/configs/'
    configuration_sufix = '.yaml'
    container_name = 'robot-runner_container'

    # position_goals = [(1, [-2.50, 1.5]), (2, [-2.50, -1.25]),
    #                 (3, [-0.35, -0.80]), (4, [1.50, -1.00]),
    #                 (5, [2.50, 0.50]), (6, [0.6, 0.50])]

    def exec_docker_command(self, command: str):
        docker_command = f"docker exec {self.container_name} bash -c \"{command}\""
        result = subprocess.run(docker_command, shell=True, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Error: {result.stderr}")
        else:
            print(f"Success: {result.stdout}")        
        time.sleep(5)
        return result
    
    def run_command_in_thread(self, command: str) -> threading.Thread:
        thread = threading.Thread(target=self.exec_docker_command, args=(command,))
        return thread

    def set_initial_position(self):
        command = "source /opt/ros/humble/setup.bash && source /packages/setup.bash && ros2 run reconfros2 initial_pose"
        self.exec_docker_command(command)
        return

    def next_configuration(self, context: RobotRunnerContext): # implement the move to position logic
        
        variation = context.run_variation
        # Getting Configuration
        configuration_number = variation['configuration']
        global_enabled = False
        if variation['global']:
            global_enabled = True
        local_enabled = False
        if variation['local']:
            local_enabled = True

        print(f"Loading configuration '{configuration_number}'...")
        
        local_configuation = 'local/config_' + str(configuration_number) + self.configuration_sufix
        global_configuration = 'global/config_' + str(configuration_number) + self.configuration_sufix
        local_configuration_path = self.configuration_root_dir + local_configuation
        global_configuration_path = self.configuration_root_dir + global_configuration

        if global_enabled:
            command = "source /opt/ros/humble/setup.bash && source /packages/setup.bash && ros2 param load /global_costmap/global_costmap " + global_configuration_path
            self.exec_docker_command(command)
        
        if local_enabled:
            command = "source /opt/ros/humble/setup.bash && source /packages/setup.bash && ros2 param load /local_costmap/local_costmap " + local_configuration_path
            self.exec_docker_command(command)

        return
    
    # def next_position(self, context: RobotRunnerContext):
    #     variation = context.run_variation

    #     # Getting next position
    #     next_position = int(variation['position_goal'])
    #     # command = f"source /opt/ros/humble/setup.bash && source /packages/setup.bash && ros2 run reconfros2 nav_to_pose --ros-args -p nav_goal:={next_position}"
    #     command = f"source /opt/ros/humble/setup.bash && source /packages/setup.bash && ros2 run reconfros2 nav_to_pose_action --ros-args -p nav_goal:={next_position} -p timeout_sec:=200"
    #     self.exec_docker_command(command)

    def next_position(self, context: RobotRunnerContext) -> threading.Thread:
        variation = context.run_variation

        # Getting next position
        next_position = int(variation['position_goal'])
        command = f"source /opt/ros/humble/setup.bash && source /packages/setup.bash && ros2 run reconfros2 nav_to_pose_action --ros-args -p nav_goal:={next_position} -p timeout_sec:=200"
        #self.exec_docker_command(command)
        return self.run_command_in_thread(command)

    def next_obstacle(self, context: RobotRunnerContext): # implement obstacles at the preset position
        variation = context.run_variation

        # number obstacles
        number_obstacles = variation['number_obstacles']
        nav_goal = variation['position_goal']

        if number_obstacles > 0:
            print(f"Adding '{number_obstacles}' obstacles.")
            command = f"source /opt/ros/humble/setup.bash && source /packages/setup.bash && ros2 run reconfros2 add_obstacle --ros-args -p num_obstacles:={number_obstacles} -p nav_goal:={nav_goal}"
            #self.exec_docker_command(command)
            return self.run_command_in_thread(command)
        else:
            print("No obstacles!")
            return threading.Thread()