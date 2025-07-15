import sys
import docker
import os
import psutil
import time

from ConfigValidator.Config.Models.RunnerContext import RunnerContext

rr_path = os.getenv('RR_PATH')
sys.path.append(rr_path)

class DockerCPUProfiler:
    def __init__(self, context: RunnerContext):
        self.context = context

    def get_container_pid(container_name):
        client = docker.from_env()
        container = client.containers.get(container_name)
        return container.attrs['State']['Pid']

    def measure_cpu_metrics(duration, container_name):
        
        # Getting the Docker Container PID
        pid = get_container_pid(container_name)

        # Attach to the process
        process = psutil.Process(pid)
        
        # Initial CPU times
        start_cycles = process.cpu_times().user + process.cpu_times().system
        start_percent = process.cpu_percent(interval=None)
        
        # Wait for the specified duration
        time.sleep(duration)
        
        # Final CPU times
        end_cycles = process.cpu_times().user + process.cpu_times().system
        end_percent = process.cpu_percent(interval=None)
        
        # Calculate metrics
        cpu_cycles = end_cycles - start_cycles
        cpu_percent = (end_percent - start_percent) / duration
        
        return cpu_cycles, cpu_percent