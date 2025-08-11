# Orchestrating the Experiments via Robot Runner

This is a README file for the orchestration of RL4GreenROS experiments with [Robot Runner](https://github.com/IntelAgir-Research-Group/robot-runner/tree/distributed-rr) (RR). 

## Prerequisites/Environment Setup

Before running this package, make sure you have the following prerequisites installed:

- ROS 2
- Docker
- Python >3 (It has been validated on `Python 3.10`)
- [PowerJoular](https://github.com/joular/powerjoular) -- Check the documentation for installation.


### PowerJoular

For energy measurements, we use **PowerJoular**, which must be set to run as root without password.

For that, first put the rule in a drop-in with visudo and correct perms:

```bash
sudo visudo -f /etc/sudoers.d/powerjoular-michel
```

Add the following content (replacing user with your username) into the file:

```bash
Cmnd_Alias POWERJOULAR = /usr/local/bin/powerjoular, /usr/bin/powerjoular
user ALL=(root) NOPASSWD: POWERJOULAR
```

bash
Copy
Edit
sudo chown root:root /etc/sudoers.d/powerjoular-michel
sudo chmod 0440 /etc/sudoers.d/powerjoular-michel
Ensure /etc/sudoers has #includedir /etc/sudoers.d (not commented).


### Running the Experiments

You must set path to your `ROS2-ee-reconf` folder:

```bash
export RL4GreenROS_PATH="<path here>"
```

You must also source your ROS 2 and RL4GreenROS packages:

```bash
source /opt/ros/humble/setup.bash
source $RL4GreenROS_PATH/docker/install/setup.bash
```

Check [here](../../#running-gui-from-docker) how to configure your Linux for running GUI applications on Docker.

## Usage

Please, clone the robot-runner repository and set its path to `RR_path`:

```bash
export RR_PATH="<PATH>"
export PYTHONPATH=$PYTHONPATH:$RL4GreenROS_PATH:$RR_PATH
```

With everything set, you must run the orchestrator with the following command:

```bash
python3 $RR_PATH/robot-runner/ $RL4GreenROS_PATH/exp_orchestration/GreenROS-RR-reconf.py
```