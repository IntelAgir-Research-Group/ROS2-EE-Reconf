import os
import subprocess
import sys
from threading import Thread, Event
import time

rr_path = os.getenv('RR_PATH')
sys.path.append(rr_path)

class ProcessResProfiler:

    def __init__(self, pid: str, file: str):
        try:
            print(f"Starting the energy profiler for PID {pid} and saving the measurements into {file}")
            self.stop_event = Event()
            self.process_pid = pid
            self.file_name = file  # Ensure attribute is correctly set
            print("Setting up the energy measurement command...")
            self.command = ["/usr/bin/powerjoular", "-p", self.process_pid, "-f", self.file_name]
        except Exception as e:
            print(f"Error during initialization: {e}")
            raise

    def run_subprocess(self):
        print(f"[{self.process_pid}]  Monitoring resource consumption.")
        command_str = subprocess.list2cmdline(self.command)
        try:
            process = subprocess.Popen(["sudo", "bash", "-c", command_str], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except Exception as e:
            print(f"[{self.process_pid}] Error running subprocess: {e}")
            return
        
        print(f"[{self.process_pid}] Resource profiler started.")
        while not self.stop_event.is_set():
            time.sleep(0.1)

        process.terminate()
        stdout, stderr = process.communicate()
        process.wait()
        if stderr:
            print(f"[{self.process_pid}] STDERR: {stderr.decode()}")
        else:
            print(f"[{self.process_pid}] Subprocess terminated.")

    def stop(self):
        time.sleep(10)  # Optional: wait before stopping
        print(f"[{self.process_pid}] Received stop event!")
        print("Stopping energy profiler...")
        self.stop_event.set()

    def start(self):
        print("Starting energy profiler...")
        self.subprocess_thread = Thread(target=self.run_subprocess)
        self.subprocess_thread.start()

