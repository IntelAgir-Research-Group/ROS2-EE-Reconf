import os
from pathlib import Path
import sys
import psutil
import time
import csv
from datetime import datetime
from threading import Thread, Event
import subprocess
from typing import List, Tuple, Optional

# Optional: add robot-runner to PYTHONPATH if present
rr_path = os.getenv('RR_PATH')
project_path = os.getenv('RL4GreenROS_PATH')
rr_project = rr_path + 'robot-runner/' if rr_path else ''
if rr_project:
    sys.path.append(rr_project)

try:
    from ConfigValidator.Config.Models.RobotRunnerContext import RobotRunnerContext
except Exception:
    class RobotRunnerContext:
        run_variation = {}

class ROS2CPUMemProfiler:

    stop_event = Event()
    __pid = None
    factors = []

    # ---- PowerJoular config ----
    pj_cmd = "powerjoular"          # binary name; change if needed
    use_sudo = True                 # set False if running as root or no sudo needed
    pj_overwrite = False             # keep a single-line CSV that we re-read
    pj_timestamp_ms = True          # ask PJ to output ms timestamps if supported
    SAMPLE_PERIOD = 0.1             # seconds

    # ---- Internal PowerJoular state ----
    _pj_proc: Optional[subprocess.Popen] = None
    _pj_file: Optional[str] = None
    _pj_energy_j: float = 0.0
    _pj_last_t: Optional[float] = None
    _pj_active: int = 0

    def __init__(self, name, file):
        self.file_name = file
        self.name = name
        self.__pid = self.get_pid_by_name(name)

    # ---------- PID helpers ----------
    def get_pid(self):
        return self.__pid

    def set_pid(self, new_pid: int):
        self.__pid = new_pid

    def get_pid_by_name(self, name: str):
        for proc in psutil.process_iter(['pid', 'name']):
            try:
                if proc.info['name'] == name:
                    print(f"Process found with PID {proc.info['pid']}")
                    return proc.info['pid']
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return None

    # ---------- CPU & memory ----------
    def get_cpu_usage(self):
        try:
            process = psutil.Process(self.__pid)
            cpu_usage = process.cpu_percent(interval=0.05)  # quick prime
            for child in process.children(recursive=True):
                try:
                    cpu_usage += child.cpu_percent(interval=0.0)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            return cpu_usage
        except psutil.NoSuchProcess:
            return None

    def get_cpu_cycles_estimate(self):
        """
        Estimate total CPU cycles used by the process and its children so far.
        (user+system) CPU time [s] * current CPU frequency [Hz].
        """
        if self.__pid is None:
            return None
        try:
            process = psutil.Process(self.__pid)

            def _cpu_seconds(p: psutil.Process) -> float:
                t = p.cpu_times()
                return (t.user or 0.0) + (t.system or 0.0)

            total_seconds = _cpu_seconds(process)
            for child in process.children(recursive=True):
                try:
                    total_seconds += _cpu_seconds(child)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

            freq = psutil.cpu_freq()
            if not freq or not freq.current:
                return None
            cycles = total_seconds * (freq.current * 1_000_000)  # MHz -> Hz
            return int(cycles)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            return None

    def get_memory_usage(self):
        try:
            process = psutil.Process(self.__pid)
            memory_usage = process.memory_info().rss
            for child in process.children(recursive=True):
                try:
                    memory_usage += child.memory_info().rss
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            return memory_usage
        except psutil.NoSuchProcess:
            return None

    # ---------- PowerJoular integration ----------
    def _start_powerjoular(self):
        """Spawn PowerJoular and verify the output file is actually created."""
        self._pj_active = 0
        self._pj_energy_j = 0.0
        self._pj_last_t = None

        if self.__pid is None or not psutil.pid_exists(self.__pid):
            print("[powerjoular] PID is None or not alive.")
            return

        self._pj_file = f"{project_path}/docker/data/pj_{self.name}.csv"
        self._pj_log  = f"/tmp/pj_{self.__pid}.log"

        # Make sure directory exists
        try:
            os.makedirs(os.path.dirname(self._pj_file), exist_ok=True)
        except Exception as e:
            print(f"[powerjoular] cannot create dir: {e}")
            return

        # Build command
        cmd = []
        if self.use_sudo and os.geteuid() != 0:
            cmd += ["sudo", "-n"]  # non-interactive; will fail fast if not permitted
        cmd += [self.pj_cmd, "-p", str(self.__pid)]

        # Prefer overwrite; some builds only support -f (append)
        out_flag = "-o" if self.pj_overwrite else "-f"
        cmd += [out_flag, self._pj_file]
        # if self.pj_timestamp_ms:
        #     cmd += ["-c"]
        # Optional: set sampling interval if your PJ supports it (uncomment if needed)
        # cmd += ["-i", "100"]  # 100 ms

        print(f"Powerjoular command: {cmd}")

        try:
            self._pj_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=open(self._pj_log, "wb"),
                start_new_session=True
            )
        except FileNotFoundError:
            print("[powerjoular] binary not found in PATH.")
            self._pj_proc = None
            return
        except Exception as e:
            print(f"[powerjoular] failed to start: {e}")
            self._pj_proc = None
            return

        # Wait up to 2s for the file to appear & have at least 1 non-empty line
        t0 = time.time()
        while time.time() - t0 < 2.0:
            # Died already?
            rc = self._pj_proc.poll()
            if rc is not None:
                print(f"[powerjoular] exited immediately with code {rc}. See log: {self._pj_log}")
                # If sudo -n denied, stderr will say "a password is required"
                return
            if os.path.exists(self._pj_file):
                try:
                    with open(self._pj_file, "rb") as f:
                        data = f.read().strip()
                    if data:
                        self._pj_active = 1
                        break
                except Exception:
                    pass
            time.sleep(0.1)

        if not self._pj_active:
            # Try append mode fallback once if overwrite didn’t work
            if out_flag == "-o":
                try:
                    if self._pj_proc and self._pj_proc.poll() is None:
                        self._pj_proc.terminate()
                except Exception:
                    pass
                try:
                    os.remove(self._pj_file)
                except Exception:
                    pass

                print("[powerjoular] overwrite file not created; retrying with append mode (-f).")
                self.pj_overwrite = False
                return self._start_powerjoular()

            print(f"[powerjoular] file not created/filled after timeout. See {self._pj_log}.")


    # def stop_profiler(self):
    #     self.stop_event.set()
    #     try:
    #         if self._pj_proc and self._pj_proc.poll() is None:
    #             self._pj_proc.terminate()
    #     except Exception:
    #         pass
    #     finally:
    #         self._pj_proc = None

    def _stop_powerjoular(self):
        if self._pj_proc:
            try:
                self._pj_proc.terminate()
            except Exception:
                pass
            self._pj_proc = None
        self._pj_active = 0

    def _read_pj_latest(self) -> tuple[Optional[float], Optional[float]]:
        """
        Return (timestamp_seconds, watts) from PowerJoular CSV.
        PID CSV: Date,CPU Utilization,CPU Power
        System CSV: Date,CPU Utilization,Total Power,CPU Power,GPU Power
        """
        path = self._pj_file
        if not path or not os.path.exists(path):
            return (None, None)
        try:
            with open(path, "rb") as f:
                data = f.read()
            if not data:
                return (None, None)
            last_line = data.rsplit(b"\n", 1)[-1].decode("utf-8").strip()
            if not last_line:
                return (None, None)
            parts = [p.strip() for p in last_line.split(",")]
            watts = None
            if len(parts) >= 5:       # system CSV
                watts = float(parts[3])  # CPU Power
            elif len(parts) == 3:     # PID CSV
                watts = float(parts[2])  # CPU Power
            else:
                return (None, None)
            return (time.time(), watts)  # use wall-time; PJ "Date" is a string
        except Exception:
            return (None, None)


    # ---------- Orchestration ----------
    def start_profiler(self, context):
        self.stop_event.clear()
        self._start_powerjoular()
        thread = Thread(target=self.profiling, args=(context,), daemon=True)
        thread.start()

    def get_variation(self, context: RobotRunnerContext):
        if self.__pid is None:
            print(f"Process {self.__pid} is not running.")
            return ()
        variation = context.run_variation if hasattr(context, "run_variation") else {}
        variation_factor_values: list = list(map(variation.get, ['__run_id']))
        return tuple(variation_factor_values)

    def profiling(self, context: RobotRunnerContext):
        variation_factor_values: list = list(self.get_variation(context))
        print(f"Monitoring CPU, memory, and PowerJoular power for PID {self.__pid}...")

        with open(self.file_name, mode='w', newline='') as file:
            writer = csv.writer(file)
            columns = [
                'timestamp',
                *tuple(self.factors),
                'cpu_percentage',
                'memory_rss_bytes',
                'cpu_cycles_est',
                'pj_power_w',
                'pj_energy_j',
                'pj_active'
            ]
            writer.writerow(columns)

            while not self.stop_event.is_set():
                t_loop_start = time.time()

                cpu_usage = self.get_cpu_usage()
                memory_usage = self.get_memory_usage()
                cpu_cycles_est = self.get_cpu_cycles_estimate()

                # --- PowerJoular sampling & energy integration ---
                pj_power_w = ''
                if self._pj_active and self.__pid and psutil.pid_exists(self.__pid):
                    ts, watts = self._read_pj_latest()
                    if watts is not None:
                        pj_power_w = watts  # <-- stays instantaneous (W)
                        now = ts if ts is not None else time.time()
                        if self._pj_last_t is not None and now >= self._pj_last_t:
                            dt = max(0.0, now - self._pj_last_t)
                            self._pj_energy_j += watts * dt  # <-- only energy accumulates
                        self._pj_last_t = now
                else:
                    # If PJ process died, mark inactive once
                    if self._pj_active:
                        self._pj_active = 0

                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

                if (cpu_usage is not None) and (memory_usage is not None):
                    writer.writerow([
                        timestamp,
                        *tuple(variation_factor_values),
                        cpu_usage,
                        memory_usage,
                        cpu_cycles_est if cpu_cycles_est is not None else '',
                        pj_power_w,
                        self._pj_energy_j if self._pj_energy_j else '',
                        self._pj_active
                    ])
                    file.flush()

                # Pace loop
                elapsed = time.time() - t_loop_start
                sleep_left = self.SAMPLE_PERIOD - elapsed
                if sleep_left > 0:
                    time.sleep(sleep_left)

    def stop_profiler(self):
        self.stop_event.set()
        self._stop_powerjoular()
