import os
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
from Plugins.Profilers.ProcessResProfiler import ProcessResProfiler

from typing import Dict, List
from pathlib import Path
import time

rl4greenros_path = os.getenv('RL4GreenROS_PATH')

class RobotRunnerConfig:
    name:                       str             = "rl4greenros_multiproc_3"
    required_ros_version:       int             = 2
    required_ros_distro:        str             = any
    operation_type:             OperationType   = OperationType.AUTO
    time_between_runs_in_ms:    int             = 1000
    results_output_path:        Path            = Path("~/Documents/experiments")
    distributed_clients:        int             = 0
    
    # =================================================USER SPECIFIC UNNECESSARY CONFIG===============================================

    docker_runner: DockerRunner
    ros_driver: ROSDriver
    cpu_mem_profiler: CPUMemProfiler
    controller_profile: ProcessResProfiler
    planner_profile: ProcessResProfiler

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
            self.cpu_mem_profiler = CPUMemProfiler('nav2_container','cpu_mem.csv')
            self.controller_profile = ProcessResProfiler('controller_server', 'nav2_controller')
            self.planner_profile = ProcessResProfiler('planner_server', 'nav2_planner')
            self.smoother_profile = ProcessResProfiler('smoother_server', 'nav2_smoother')
            self.behavior_profile = ProcessResProfiler('behavior_server', 'nav2_behavior')
            self.btnav_profile = ProcessResProfiler(' bt_navigator', 'nav2_bt')
            self.waypoint_profile = ProcessResProfiler('waypoint_follower', 'nav2_waypoint')
            self.velocity_profile = ProcessResProfiler('velocity_smoother', 'nav2_velocity')
            self.lifecycle_profile = ProcessResProfiler('lifecycle_manager', 'nav2_lifecycle')
            self.rstate_profile = ProcessResProfiler('robot_state_publisher', 'nav2_rstate')
            self.map_server = ProcessResProfiler('map_server', 'nav2_map')
            self.gzserver = ProcessResProfiler('gzserver', 'nav2_gzserver')
            self.gzclient = ProcessResProfiler('gzclient', 'nav2_gzclient')
            self.rviz = ProcessResProfiler('rviz2', 'nav2_rviz')

            EventSubscriptionController.subscribe_to_multiple_events_multiple_callbacks([ 
                (RobotRunnerEvents.BEFORE_EXPERIMENT,   [self.before_experiment]), 
                (RobotRunnerEvents.BEFORE_RUN,          [self.before_run]),
                (RobotRunnerEvents.START_RUN,           [self.start_run]),
                (RobotRunnerEvents.START_MEASUREMENT,   [self.start_measurement]),
                (RobotRunnerEvents.LAUNCH_MISSION,      [self.launch_mission]),
                (RobotRunnerEvents.STOP_MEASUREMENT,    [self.stop_measurement]),
                (RobotRunnerEvents.STOP_RUN,            [self.stop_run]),
                (RobotRunnerEvents.POPULATE_RUN_DATA,   [self.populate_run_data]),
                (RobotRunnerEvents.AFTER_EXPERIMENT,    [self.after_experiment])
            ])
            
            print("Custom config loaded")

        self.containers = ['gazebo', 'nav2', 'rviz', 'robot-runner']

    def create_run_table(self) -> List[Dict]:
        """Create and return the run_table here. A run_table is a List (rows) of tuples (columns), 
        representing each run robot-runner must perform"""
        run_table = RunTableModel(
            factors = [
                FactorModel("round", [1]),
                FactorModel("configuration", range(1,21)),
                FactorModel("position_goal", [1]),
                FactorModel("number_obstacles", [0])
            ] 
        )
        run_table.create_experiment_run_table()
        return run_table.get_experiment_run_table()

    def before_experiment(self) -> None:
        """Perform any activity required before starting the experiment here"""
        print("before_experiment() called: cleaning and bringing up")
        self.docker_runner.clean_containers(self.containers)

    def before_run(self) -> None:
        """Perform any activity required before starting a run, no context is available 
        here as the run is not yet active (BEFORE RUN)"""
        print("before_run() called: setting up the Docker environment")
        for c in self.containers:
            self.docker_runner.start_container(c)

        print("Setting robot initial position on Gazebo")
        self.ros_driver.set_initial_position()
        
    def start_run(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for starting the run here. 
        Activities before and after starting the run should also be performed here."""
        print("Config.start_run() called!")
        self.ros_driver.next_configuration(context)
        self.ros_driver.next_obstacle(context)
        
    def start_measurement(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for starting measurements."""
        print("Config.start_measurement called!")
        self.cpu_mem_profiler.start_profiler(context)
        self.controller_profile.start()
        self.planner_profile.start()
        self.smoother_profile.start()
        self.behavior_profile.start()
        self.btnav_profile.start()
        self.waypoint_profile.start()
        self.velocity_profile.start()
        self.lifecycle_profile.start()
        self.rstate_profile.start()
        self.map_server.start()
        self.gzserver.start()
        self.gzclient.start()
        self.rviz.start()

    def launch_mission(self, context: RobotRunnerContext) -> None:
        """Perform any activity interacting with the robotic
        system in question (simulated or real-life) here."""

        print("Config.launch_mission() called!")
        self.ros_driver.next_position(context)

    def stop_measurement(self, context: RobotRunnerContext) -> None:
        """Perform any activity here required for stopping measurements."""
        print("Config.stop_measurement called!")
        self.cpu_mem_profiler.stop_profiler()
        self.controller_profile.stop()
        self.planner_profile.stop()
        self.smoother_profile.stop()
        self.behavior_profile.stop()
        self.btnav_profile.stop()
        self.waypoint_profile.stop()
        self.velocity_profile.stop()
        self.lifecycle_profile.stop()
        self.rstate_profile.stop()
        self.map_server.stop()
        self.gzserver.stop()
        self.gzclient.stop()
        self.rviz.stop()
        """
        
            Dumping measurements: Improve the code later
        
        """
        print("Copying the power consumption data to the results folder")
        variation = context.run_variation
        run_id = variation['__run_id']
        dest_folder = f"~/Documents/experiments/{self.name}/{run_id}/"
        dest_perf_file = f"~/Documents/experiments/{self.name}/{run_id}/performance.csv"
        dest_cpu_file = f"~/Documents/experiments/{self.name}/{run_id}/cpu_mem.csv"
        command_rapl_1 = f"cp {rl4greenros_path}/nav2_*.csv {dest_folder}"
        subprocess.run(command_rapl_1, shell=True)
        command_perf = f"cp {rl4greenros_path}/docker/data/nav2_performance.csv {dest_perf_file}"
        subprocess.run(command_perf, shell=True)
        command_cpu = f"cp {rl4greenros_path}/cpu-mem.csv {dest_cpu_file}"
        subprocess.run(command_cpu, shell=True)
        rm_nav2_profiling=f"sudo rm -f ~/Documents/experiments/{self.name}/nav2*"
        subprocess.run(rm_nav2_profiling, shell=True)

    def stop_run(self, context: RobotRunnerContext) -> None:
        """Perform any activity required for stopping the run here.
        Activities before and after stopping the run should also be performed here."""
        
        print("Config.stop_run() called!")
        self.docker_runner.clean_containers(self.containers)
        command_cleaning = "./clean-containers.sh"
        subprocess.run(command_cleaning, shell=True)
        rm_nav2_profiling=f"sudo rm -f {rl4greenros_path}/nav2*.csv" 
        subprocess.run(rm_nav2_profiling, shell=True)
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
