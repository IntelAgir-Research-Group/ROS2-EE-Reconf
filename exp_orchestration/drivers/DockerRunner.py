
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
import threading
import docker
import os
import subprocess
import time

volumes = ['packages', 'nav2_data', 'nav2_profiler', 'exp_orchestration', 'maps', 'robot-runner']
rr_path = os.getenv('RR_PATH')
experiment_path = os.getenv('RL4GreenROS_PATH')
compose_file_path = None
processes = []

class DockerRunner:
    containers = ['gazebo', 'nav2', 'rviz', 'robot-runner']

    def kill_processes():
        for p in processes:
            try:
                p.terminate()
                p.wait()
            except:
                continue

    def volume_exists(self, volume_name):
        try:
            subprocess.run(["docker", "volume", "inspect", volume_name], check=True, stdout=subprocess.PIPE)
            return True  # Volume exists
        except subprocess.CalledProcessError:
            return False  # Volume does not exist

    def container_exists(self, container_name):
        try:
            subprocess.run(["docker", "container", "inspect", container_name], check=True, stdout=subprocess.PIPE)
            return True
        except subprocess.CalledProcessError:
            return False
    
    def set_containers(self, containers):
        self.containers = containers
        print('List of containers is set!')

    def remove_containers(self):
        for container in self.containers:
            container_name = container+'_container'
            if self.container_exists(container_name):
                try:
                    subprocess.run(["docker", "kill", container_name], check=True)
                    subprocess.run(["docker", "rm", "-f", container_name], check=True)
                    print(f"Container '{container_name}' removed successfully.")
                except subprocess.CalledProcessError as e:
                    print(f"Error removing container '{container_name}': {e}")
        return True

    def remove_volumes(self):
        # Removing volumes
        for volume in volumes:
            if self.volume_exists(volume):
                try:
                    subprocess.run(["docker", "volume", "rm", volume], check=True)
                    print(f"Volume '{volume}' removed successfully.")
                except subprocess.CalledProcessError as e:
                    print(f"Error removing volume '{volume}': {e}")

    def clean_docker_environment(self):
        self.remove_containers()
        self.remove_volumes()

    def clean_containers(self, containers):
        for c in containers:
            container_name = c+'_container'
            if self.container_exists(container_name):
                try:
                    subprocess.run(["docker", "rm", "-f", container_name], check=True)
                    print(f"Container '{container_name}' removed successfully.")
                except subprocess.CalledProcessError as e:
                    print(f"Error removing container '{container_name}': {e}")
        self.remove_volumes()
        return True    

    def check_compose_file(self):
        global compose_file_path
        compose_file_path = experiment_path+'/docker/compose/docker-compose.yml'
        if not os.path.exists(compose_file_path):
            print(f"docker-compose.yml file not found: {compose_file_path}")
            return False
        else:
            return True
            
    def docker_command(self, container, command):
    
        container_name = container
        if self.container_exists(container_name):
            try:
                # subprocess.run(["docker", "exec", "-it", container_name, '/bin/bash', '-c', command], check=True)
                subprocess.Popen(["docker", "exec", "-it", container_name, '/bin/bash', '-c', command], stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
                print(f"Command successfull in container '{container_name}'")
            except subprocess.CalledProcessError as e:
                print(f"Error executing command in container '{container_name}': {e}")
        return True    
    
    def execute_commands_in_parallel(self, commands):
        time.sleep(0.1)
        #with ProcessPoolExecutor() as executor:
        with ThreadPoolExecutor() as executor:
            futures = {executor.submit(self.docker_command, container, command): container for container, command in commands}
            print('All Docker commands have been instantiated...')

    def run_in_thread(self, container, command):
        thread = threading.Thread(target=self.docker_command, args=(container, command))
        thread.start()
        return thread

    def start_container(self, container, scale):
        if self.check_compose_file():
            try:
                if scale == 1:
                    subprocess.run(["docker", "compose", "-f", compose_file_path, "up", "-d", container], check=True)
                else:
                    subprocess.run(["docker", "compose", "-f", compose_file_path, "up", "-d", "--scale", f"{container}={scale}"], check=True)
                print(f"Container for service '{container}' started successfully.")
                print('Wainting for the container to warm up... ')
                time.sleep(20)
                return True
            except subprocess.CalledProcessError as e:
                print(f"Error starting container for service '{container}': {e}")
                return False