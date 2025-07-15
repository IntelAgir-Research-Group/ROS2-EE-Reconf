import os
import signal
from threading import Thread
import subprocess

from EventManager.Models.RobotRunnerEvents import RobotRunnerEvents
from EventManager.EventSubscriptionController import EventSubscriptionController
from ConfigValidator.Config.Models.RunTableModel import RunTableModel
from ConfigValidator.Config.Models.FactorModel import FactorModel
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext
from ConfigValidator.Config.Models.OperationType import OperationType

# System Runners
from Plugins.Systems.Docker.DockerRunner import DockerRunner
from Plugins.Systems.RL4GreenROS.ROSDriver import ROSDriver

# Resource Profilers
from Plugins.Profilers.CPUMemProfiler import CPUMemProfiler
#from Plugins.Profilers.DockerCPUProfiler import DockerCPUProfiler
#from Plugins.Profilers.DockerMemProfiler import DockerMemProfiler
#from Plugins.Profilers.DockerPowerProfiler import DockerPowerProfiler
#from Plugins.Profilers.Nav2PerformanceProfiler import Nav2PerformanceProfiler

from typing import Dict, List
from pathlib import Path
import time

rl4greenros_path = os.getenv('RL4GreenROS_PATH')

class RobotRunnerConfig:
    name:                       str             = "rl4greenros_experiment"
    required_ros_version:       int             = 2
    required_ros_distro:        str             = any
    operation_type:             OperationType   = OperationType.AUTO
    time_between_runs_in_ms:    int             = 1000
    results_output_path:        Path            = Path("~/Documents/experiments")
    distributed_clients:        int             = 1
    
    # =================================================USER SPECIFIC UNNECESSARY CONFIG===============================================

    docker_runner: DockerRunner
    ros_driver: ROSDriver
    cpu_mem_profiler: CPUMemProfiler
    discovery_server_pid: int

    # Dynamic configurations can be one-time satisfied here before the program takes the config as-is
    # NOTE: Setting some variable based on some criteria
    def __init__(self):
        """Executes immediately after program start, on config load"""
        if not rl4greenros_path:
            print('Please, set the RL4GreenROS path: RL4GreenROS_PATH environment variable.')
            os._exit(0)
        else:
            self.docker_runner = DockerRunner()
            self.ros_driver = ROSDriver()
            self.cpu_mem_profiler = CPUMemProfiler()

            EventSubscriptionController.subscribe_to_multiple_events_multiple_callbacks([ 
                (RobotRunnerEvents.BEFORE_EXPERIMENT,   [self.clean_local_docker]),
                (RobotRunnerEvents.BEFORE_RUN,          [self.start_discovery, self.start_gazebo, self.start_rviz, self.start_rr]),
                (RobotRunnerEvents.START_RUN,           [self.set_robot_initial_position, self.set_up_experimental_world]),
                (RobotRunnerEvents.START_MEASUREMENT,   [self.start_measurement]),
                (RobotRunnerEvents.LAUNCH_MISSION,      [self.drive_robot]),
                (RobotRunnerEvents.STOP_MEASUREMENT,    [self.stop_measurement]),
                (RobotRunnerEvents.STOP_RUN,            [self.stop_run]),
            ])

            EventSubscriptionController.subscribe_to_multiple_events_multiple_remote_calls([
                (RobotRunnerEvents.BEFORE_EXPERIMENT, [('clean_docker', ['nav2'])]),
                (RobotRunnerEvents.BEFORE_RUN,        [('start_remote_nav2_container', ['nav2'])]),
                (RobotRunnerEvents.STOP_RUN,          [('clean_docker', ['nav2'])]),
            ])
            
            print("Custom config loaded")

    def create_run_table(self) -> List[Dict]:
        run_table = RunTableModel(
            factors = [
                FactorModel("round", range(0,6)),
                FactorModel("configuration", range(1,26)),
                FactorModel("position_goal", [2]),
                FactorModel("number_obstacles", [0])
            ]
        )
        run_table.create_experiment_run_table()
        return run_table.get_experiment_run_table()

    def clean_local_docker(self) -> None:
        print("Cleaning up local Docker environment")
        self.docker_runner.set_containers(['gazebo', 'rviz', 'robot-runner', 'discovery'])
        self.docker_runner.remove_containers()

    def start_gazebo(self) -> None:
        print("Starting Gazebo")
        self.docker_runner.start_container("gazebo")

    def start_discovery(self) -> None:
        print("Starting DDS Discovery")
        self.docker_runner.start_container("discovery")

    def start_rviz(self) -> None:
        print("Starting Rviz")
        self.docker_runner.start_container("rviz")
    
    def start_rr(self) -> None:
        print("Starting Robot Runner")
        self.docker_runner.start_container("robot-runner")

    def set_robot_initial_position(self, context: RobotRunnerContext) -> None:
        print("Setting robot initial position on Gazebo")
        self.ros_driver.set_initial_position() # get the initial position from the context
        
    def set_up_experimental_world(self, context: RobotRunnerContext) -> None:
        print("Setting up the configuration and obstacles!")
        self.ros_driver.next_configuration(context)
        self.ros_driver.next_obstacle(context)

    def start_measurement(self, context: RobotRunnerContext) -> None:
        print("Start measuring resource usage!")
        # self.cpu_mem_profiler.start_profiler("nav2_container","cpu-mem.csv", context)

    def drive_robot(self, context: RobotRunnerContext) -> None:
        print("Driving the robot to the next position!")
        self.ros_driver.next_position(context)

    def stop_measurement(self, context: RobotRunnerContext) -> None:
        print("Stopping measurements!")
        # self.cpu_mem_profiler.stop_profiler()
        """
        
            Dumping measurements: Improve the code later
        
        """
        print("Copying the power consumption data to the results folder")
        # variation = context.run_variation
        # run_id = variation['__run_id']
        # dest_rapl_file = f"~/Documents/experiments/rl4greenros_experiment/{run_id}/rapl.csv"
        # dest_perf_file = f"~/Documents/experiments/rl4greenros_experiment/{run_id}/performance.csv"
        # dest_cpu_file = f"~/Documents/experiments/rl4greenros_experiment/{run_id}/cpu_mem.csv"
        # command_rapl = f"cp {rl4greenros_path}/docker/data/rapl.csv {dest_rapl_file}"
        # subprocess.run(command_rapl, shell=True)
        # command_perf = f"cp {rl4greenros_path}/docker/data/nav2_performance.csv {dest_perf_file}"
        # subprocess.run(command_perf, shell=True)
        # command_cpu = f"cp {rl4greenros_path}/cpu-mem.csv {dest_cpu_file}"
        # subprocess.run(command_cpu, shell=True)

    def stop_run(self, context: RobotRunnerContext) -> None:
        print("Stop run!")
        self.docker_runner.set_containers(['gazebo', 'rviz', 'robot-runner', 'discovery'])
        self.docker_runner.remove_containers()
        print("Cooling down period of 30 seconds...")
        time.sleep(30)
        print("----------------- Run Finished -----------------\n\n")
    
    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
