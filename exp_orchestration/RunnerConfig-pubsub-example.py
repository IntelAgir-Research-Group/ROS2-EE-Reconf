import os
import time
import sys
experiment_path = os.getenv('EXPERIMENT_PATH')
sys.path.append(experiment_path)

from EventManager.Models.RunnerEvents import RunnerEvents
from EventManager.EventSubscriptionController import EventSubscriptionController
from ConfigValidator.Config.Models.RunTableModel import RunTableModel
from ConfigValidator.Config.Models.FactorModel import FactorModel
from ConfigValidator.Config.Models.RunnerContext import RunnerContext
from ConfigValidator.Config.Models.OperationType import OperationType
from ProgressManager.Output.OutputProcedure import OutputProcedure as output
from exp_runners.drivers.DockerRunner import DockerRunner
from exp_runners.profilers.CPUMemProfiler import CPUMemProfiler
from exp_runners.profilers.ProcessResProfiler import ProcessResProfiler

from typing import Dict, List, Any, Optional
from pathlib import Path
from os.path import dirname, realpath

import subprocess

class RunnerConfig:
    ROOT_DIR = Path(dirname(realpath(__file__)))

    name:                       str             = "cpp_py_ros2_pub_sub"
    results_output_path:        Path             = ROOT_DIR / 'experiments'
    operation_type:             OperationType   = OperationType.AUTO
    time_between_runs_in_ms:    int             = 1000
    docker_runner: DockerRunner

    # CPU and Memory
    cpu_mem_profiler_server: CPUMemProfiler
    cpu_mem_profiler_client: CPUMemProfiler

    # Energy
    energy_profiler_server: ProcessResProfiler
    energy_profiler_client: ProcessResProfiler

    global commands_to_run

    def __init__(self):
        if not experiment_path:
            print('Please, set the experiemtn path: EXPERIMENT_PATH environment variable.')
            os._exit(0)
        else:
            self.docker_runner = DockerRunner()

            EventSubscriptionController.subscribe_to_multiple_events([
                (RunnerEvents.BEFORE_EXPERIMENT, self.before_experiment),
                (RunnerEvents.BEFORE_RUN       , self.before_run       ),
                (RunnerEvents.START_MEASUREMENT, self.start_measurement),
                (RunnerEvents.START_RUN        , self.start_run        ),
                (RunnerEvents.STOP_RUN         , self.stop_run         ),
                (RunnerEvents.STOP_MEASUREMENT , self.stop_measurement ),
                (RunnerEvents.POPULATE_RUN_DATA, self.populate_run_data),
                (RunnerEvents.AFTER_EXPERIMENT , self.after_experiment )
            ])
            self.run_table_model = None  # Initialized later

            # default duration
            self.duration = 30

            # default publisher and subscribers
            self.__pub = 'talker'
            self.__sub = 'listener'

            output.console_log("Custom config loaded")

    def create_run_table_model(self) -> RunTableModel:
        package = FactorModel("ros_package", ['simple_publisher_subscriber'])
        interval = FactorModel("msg_interval", [0.05, 0.1, 0.25, 0.5, 1.0]) # JoyStick, then doubling until a good rate for logging that do not stress the system
        num_clients = FactorModel("num_clients", [1, 2, 3])
        language = FactorModel("language", ['py', 'cpp'])
        exec_time = FactorModel("exec_time", [180])
        self.run_table_model = RunTableModel(
            factors=[package, interval, language, exec_time, num_clients],
            repetitions = 20
        )
        return self.run_table_model

    def before_experiment(self) -> None:
        output.console_log("Config.before_experiment() called!")
        output.console_log("Cleaning Docker...")
        command_cleaning = "./clean-containers.sh"
        subprocess.run(command_cleaning, shell=True)

    def before_run(self) -> None:
        output.console_log("Config.before_run() called!")

    def start_run(self, context: RunnerContext) -> None:
        output.console_log("Config.start_run() called!")

        output.console_log("Starting Docker containers...")
        output.console_log("Starting Server/Pub...")
        self.docker_runner.start_container('server', 1)

        variation = context.run_variation
        clients = variation['num_clients']
        self.duration = variation['exec_time']
        output.console_log(f"Starting Clients from 1 to {clients}...")
        self.docker_runner.start_container('client', clients)

        package=variation['ros_package']
        match package:
            case 'simple_publisher_subscriber':
                self.__pub='talker'
                self.__sub='listener'
            case 'simple_service_client':
                self.__pub='server'
                self.__sub='client'
            case 'action_tutorials':
                self.__pub='fibonacci_action_server'
                self.__sub='fibonacci_action_client'
        interval=variation['msg_interval']
        language=variation['language']
        exec_time=variation['exec_time']
        
        containers = []
        commands = []

        output.console_log("Running Server/Pub command...")
        server_command = f"source /opt/ros/humble/setup.bash && source /projeto/install/setup.bash && ros2 run {package}_{language} {self.__pub} {exec_time} {interval}"
        #threads.append = self.docker_runner.run_in_thread('docker_server_1',server_command)
        containers.append('docker_server_1')
        commands.append(server_command)

        output.console_log("Running Client(s)/Sub(s) command...")
        if variation['ros_package'] == 'simple_service_client':
            client_command = f"source /opt/ros/humble/setup.bash && source /projeto/install/setup.bash && ros2 run {package}_{language} {self.__sub} 10 10 {exec_time} {interval}"
        else:
            client_command = f"source /opt/ros/humble/setup.bash && source /projeto/install/setup.bash && ros2 run {package}_{language} {self.__sub} {exec_time} {interval}"
        
        for c in range(clients):
            containers.append(f'docker_client_{c+1}')
            commands.append(client_command)

        self.commands_to_run = list(zip(containers, commands))

        # Execute the commands only after starting measurement.
        self.docker_runner.execute_commands_in_parallel(self.commands_to_run)

        time.sleep(1)

    def start_measurement(self, context: RunnerContext) -> None:
        output.console_log("Config.start_measurement() called!")
        output.console_log("Starting measuaring, then starting the ROS 2 commands...")

        # CPU and Memory
        self.cpu_mem_profiler_server = CPUMemProfiler(self.__pub, 'cpu-mem-server.csv')
        self.cpu_mem_profiler_client = CPUMemProfiler(self.__sub, 'cpu-mem-client.csv')
        
        factors_keys = ['package', 'interval', 'language', 'exec_time', 'num_clients']
        self.cpu_mem_profiler_server.start_profiler(context, factors_keys)
        self.cpu_mem_profiler_client.start_profiler(context, factors_keys)
        
        server_pid = None
        client_pid = None
        while server_pid == None or client_pid == None:
            got_pid_server = self.cpu_mem_profiler_server.get_pid_by_name('talker')
            got_pid_client = self.cpu_mem_profiler_client.get_pid_by_name('listener')
            if got_pid_server != None and got_pid_client != None:
                server_pid = str(got_pid_server)
                client_pid = str(got_pid_client)

        # Energy
        self.energy_profiler_server = ProcessResProfiler(server_pid, 'energy-server')
        time.sleep(0.1)
        self.energy_profiler_client = ProcessResProfiler(client_pid, 'energy-client')
        time.sleep(0.1)

        self.energy_profiler_server.start()
        time.sleep(0.1)
        self.energy_profiler_client.start()
        time.sleep(0.1)

        # Waiting for the duration
        start_time = time.time()
        while time.time() - start_time < self.duration:
            time.sleep(1)

    def stop_measurement(self, context: RunnerContext) -> None:
        # CPU and Memory
        self.cpu_mem_profiler_server.stop_profiler()
        self.cpu_mem_profiler_client.stop_profiler()

        # Energy
        self.energy_profiler_server.stop()
        self.energy_profiler_client.stop()

        output.console_log("Config.stop_measurement called!")

    def stop_run(self, context: RunnerContext) -> None:
        output.console_log("Config.stop_run() called!")

        variation = context.run_variation
        run_id = variation['__run_id']

        # All CSV
        dest_files = f"{self.experiment_path}/{run_id}/"
        files_cp = f"cp -f {experiment_path}/*.csv {dest_files}"
        subprocess.run(files_cp, shell=True)

        # Energy Server
        dest_files = f"{self.experiment_path}/{run_id}/"
        files_cp = f"cp -f {experiment_path}/energy-* {dest_files}"
        subprocess.run(files_cp, shell=True)

        # Clean Up
        rm_files_command = f"rm -f {experiment_path}/*.csv"
        subprocess.run(rm_files_command, shell=True)

        rm_energy_files_command = f"rm -f {experiment_path}/energy-*"
        subprocess.run(rm_energy_files_command, shell=True)

        output.console_log("Measurement data copied!")

        return None

    def populate_run_data(self, context: RunnerContext) -> Optional[Dict[str, Any]]:
        output.console_log("Config.populate_run_data() called!")
        
    def after_experiment(self) -> None:
        output.console_log("Config.after_experiment() called!")
        output.console_log("Cooling down for 10 seconds...")
        time.sleep(30)

    # ================================ DO NOT ALTER BELOW THIS LINE ================================
    experiment_path:            Path             = None
