import os
from pathlib import Path
import sys
import docker
import psutil
import time
import csv
from datetime import datetime
from threading import Thread, Event
# Experiment Runner Plugins/Drivers
import subprocess

rr_path = os.getenv('RR_PATH')
rr_project=rr_path+'robot-runner/'
sys.path.append(rr_project)

from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext

class ROS2CPUMemProfiler:

    stop_event = Event()
    __pid = None
    factors = []

    def __init__(self, name, file):
        self.file_name = file
        self.__pid = self.get_pid_by_name(name)

    def get_pid(self):
        return self.__pid
    
    def set_pid(self, new_pid: str):
        self.__pid = new_pid

    def get_pid_by_name(self, name: str):
        for proc in psutil.process_iter(['pid', 'name']):
            tries = 5
            try:
                if proc.info['name'] == name:
                    print(f"Process found with PID {proc.info['pid']}")
                    return proc.info['pid']
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return None

    def get_cpu_usage(self):
        try:
            process = psutil.Process(self.__pid)
            cpu_usage = process.cpu_percent(interval=0.1)
            children = process.children(recursive=True)
            for child in children:
                cpu_usage += child.cpu_percent(interval=0.1)
            return cpu_usage
        except psutil.NoSuchProcess:
            return None

    def get_memory_usage(self):
        try:
            process = psutil.Process(self.__pid)
            memory_usage = process.memory_info().rss
            children = process.children(recursive=True)
            for child in children:
                memory_usage += child.memory_info().rss
            return memory_usage
        except psutil.NoSuchProcess:
            return None

    def start_profiler(self, context):
        thread = Thread(target=self.profiling, args=(context,))
        thread.start()

    def get_variation(self, context: RobotRunnerContext):
        if self.__pid is None:
            print(f"Process {self.__pid} is not running.")
            return ()

        variation = context.run_variation

        variation_factor_values: list = list(map(
            variation.get, ['__run_id']
        ))

        return (variation_factor_values)
        
    def profiling(self, context: RobotRunnerContext):
        variation_factor_values: list = self.get_variation(context)

        print(f"Monitoring CPU and memory usage for PID {self.__pid})...")
        sequence = 0

        with open(self.file_name, mode='w', newline='') as file:
            writer = csv.writer(file)

            columns = ['run', *tuple(self.factors), 'cpu_percentage', 'memory_usage']
            writer.writerow(columns)
            
            while not self.stop_event.is_set():
                cpu_usage = self.get_cpu_usage()
                memory_usage = self.get_memory_usage()
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                if cpu_usage is not None and memory_usage is not None:
                    writer.writerow([timestamp,*tuple(variation_factor_values), cpu_usage, memory_usage])
                sequence += 1
                time.sleep(0.1)

    def stop_profiler(self):
        self.stop_event.set()

