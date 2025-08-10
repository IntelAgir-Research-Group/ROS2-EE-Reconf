import os
from threading import Thread
import subprocess
import sys

rr_path = os.getenv('RR_PATH')
sys.path.append(rr_path)

from EventManager.Models.RobotRunnerEvents import RobotRunnerEvents
from EventManager.EventSubscriptionController import EventSubscriptionController
from ConfigValidator.Config.Models.RunTableModel import RunTableModel
from ConfigValidator.Config.Models.FactorModel import FactorModel
from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext
from ConfigValidator.Config.Models.OperationType import OperationType

# System Runners
from exp_orchestration.drivers.DockerRunner import DockerRunner
from exp_orchestration.drivers.ROSDriver import ROSDriver

# Resource Profilers
from exp_orchestration.profilers.ROS2CPUMemProfiler import ROS2CPUMemProfiler
#from Plugins.Profilers.DockerCPUProfiler import DockerCPUProfiler
#from Plugins.Profilers.DockerMemProfiler import DockerMemProfiler
#from Plugins.Profilers.DockerPowerProfiler import DockerPowerProfiler
#from Plugins.Profilers.Nav2PerformanceProfiler import Nav2PerformanceProfiler

from typing import Dict, List
from pathlib import Path
import time

rl4greenros_path = os.getenv('RL4GreenROS_PATH')

class RobotRunnerConfig:
    name:                       str             = "greenros_reconf_world_small"
    required_ros_version:       int             = 2
    required_ros_distro:        str             = any
    operation_type:             OperationType   = OperationType.AUTO
    time_between_runs_in_ms:    int             = 1000
    results_output_path:        Path             = Path("~/Documents/experiments")
    
    # =================================================USER SPECIFIC UNNECESSARY CONFIG===============================================

    docker_runner: DockerRunner
    ros_driver: ROSDriver
    cpu_mem_profiler: ROS2CPUMemProfiler
    nav2_profiler: None # TO BE IMPLEMENTED

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
            self.cpu_mem_profiler = ROS2CPUMemProfiler('nav2_container','docker/data/cpu-mem.csv')

            EventSubscriptionController.subscribe_to_multiple_events([ 
                (RobotRunnerEvents.BEFORE_EXPERIMENT,   self.before_experiment), 
                (RobotRunnerEvents.BEFORE_RUN,          self.before_run),
                (RobotRunnerEvents.START_RUN,           self.start_run),
                (RobotRunnerEvents.START_MEASUREMENT,   self.start_measurement),
                (RobotRunnerEvents.LAUNCH_MISSION,      self.launch_mission),
                (RobotRunnerEvents.STOP_MEASUREMENT,    self.stop_measurement),
                (RobotRunnerEvents.STOP_RUN,            self.stop_run),
                (RobotRunnerEvents.POPULATE_RUN_DATA,   self.populate_run_data),
                (RobotRunnerEvents.AFTER_EXPERIMENT,    self.after_experiment)
            ])
            
            print("Custom config loaded")

    def create_run_table(self) -> List[Dict]:
        """Create and return the run_table here. A run_table is a List (rows) of tuples (columns), 
        representing each run robot-runner must perform"""
        run_table = RunTableModel(
            factors = [
                FactorModel("round", range(0,5)),
                FactorModel("configuration", range(0,20)),
                FactorModel("position_goal", [2]),
                # FactorModel("number_obstacles", [0,1,2]), # Only implemented in 1 map
                # FactorModel("map", ['small', 'medium', 'large']) # Not implemented
            ]
            # ,
            # exclude_variations = [
            #     {"example_treatment1"},     # all runs having treatment example_treatment1 will be excluded
            #     {"example_treatment1", "example_treatment2"} # all runs having the combination <treatment1, treatment2> will be excluded
            # ] sys.path.append(rr_path)
        )
        run_table.create_experiment_run_table()
        return run_table.get_experiment_run_table()

    def before_experiment(self) -> None:
        """Perform any activity required before starting the experiment here"""
        print("before_experiment() called: cleaning and bringing up")
        self.docker_runner.clean_docker_environment()

    def before_run(self) -> None:
        """Perform any activity required before starting a run, no context is available 
        here as the run is not yet active (BEFORE RUN)"""
        print("before_run() called: setting up the Docker environment")
        self.docker_runner.start_container("gazebo", 1)
        self.docker_runner.start_container("rviz", 1)
        self.docker_runner.start_container("robot-runner", 1)

    def start_run(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for starting the run here. 
        Activities before and after starting the run should also be performed here."""
        print("Config.start_run() called!")

        variation = context.run_variation
 
        print('Creating Nav2 configuration file...')

        # number obstacles
        configuration = variation['configuration']
        print(f'Configuration: {configuration}')
        project_folder = os.getenv("RL4GreenROS_PATH")
        nav2_param_file = f"{project_folder}/current_config/nav2_params.yaml"
        commands = [
            f"cat {project_folder}/current_config/default_params.yaml > {nav2_param_file}",
            f"cat {project_folder}/config/gen_configs/local/config_{configuration}.yaml >> {nav2_param_file}",
            f"cat {project_folder}/config/gen_configs/global/config_{configuration}.yaml >> {nav2_param_file}",
        ]

        for cmd in commands:
            subprocess.run(cmd, shell=True, check=True)

        print('Starting Nav2 with customized configuration')
        self.docker_runner.start_container("nav2", 1)

        print("Setting robot initial position on Gazebo")
        self.ros_driver.set_initial_position() # get the initial position from the context
        
        ## FIXED UNTIL ALL PROFILERS ARE RUNNING
        # Setting up obstacles
        # self.ros_driver.next_obstacle(context)

    def start_measurement(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for starting measurements."""
        print("Config.start_measurement called!")
        self.cpu_mem_profiler.start_profiler(context)

    def launch_mission(self, context: RobotRunnerContext) -> None:
        """Perform any activity interacting with the robotic
        system in question (simulated or real-life) here."""

        print("Config.launch_mission() called!")
        self.ros_driver.next_position(context)

    def stop_measurement(self, context: RobotRunnerContext) -> None:
        """Perform any activity here required for stopping measurements."""
        print("Config.stop_measurement called!")
        self.cpu_mem_profiler.stop_profiler()
        """
            Dumping measurements: Improve the code later
        """
        print("Copying the power consumption data to the results folder")
        variation = context.run_variation
        run_id = variation['__run_id']
        # Nav2
        dest_nav2_file = f"~/Documents/experiments/{self.name}/{run_id}/nav2_performance.csv"
        command_nav2 = f"cp {rl4greenros_path}/docker/data/nav2_performance.csv {dest_nav2_file}"
        subprocess.run(command_nav2, shell=True)
        # CPU/MEM
        dest_cpu_file = f"~/Documents/experiments/{self.name}/{run_id}/cpu-mem.csv"
        command_cpu = f"cp {rl4greenros_path}/docker/data/cpu-mem.csv {dest_cpu_file}"
        subprocess.run(command_cpu, shell=True)

    def stop_run(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for stopping the run here.
        Activities before and after stopping the run should also be performed here."""
        
        print("Config.stop_run() called!")
        self.docker_runner.clean_docker_environment()

        ## bash clean-containers


        print("Cooling down period of 30 seconds...")
        time.sleep(30)
        print("----------------- Run Finished -----------------\n\n")
    
    def populate_run_data(self, context: RobotRunnerContext) -> tuple:
        """Return the run data as a row for the output manager represented as a tuple"""
        return None

    def after_experiment(self) -> None:
        """Perform any activity required after stopping the experiment here"""
        print("after_experiment() called: cleaning the experiment environment")

    # ===============================================DO NOT ALTER BELOW THIS LINE=================================================
    # NOTE: Do not alter these values
    experiment_path:            Path             = None
