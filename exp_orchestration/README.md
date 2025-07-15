# Orchestrating the Experiments via Robot Runner

This is a README file for the orchestration of RL4GreenROS experiments with [Robot Runner](https://github.com/IntelAgir-Research-Group/robot-runner/tree/distributed-rr) (RR). 

## Prerequisites/Environment Setup

Before running this package, make sure you have the following prerequisites installed:

- ROS 2
- Docker
- Python >3 (It has been validated on `Python 3.10`)

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