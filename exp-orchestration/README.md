# Orchestrating the Experiments via Robot Runner

This is a README file for the orchestration of RL4GreenROS experiments with [Robot Runner](https://github.com/IntelAgir-Research-Group/robot-runner/tree/distributed-rr) (RR). 

## Prerequisites/Environment Setup

Before running this package, make sure you have the following prerequisites installed:

- ROS 2
- Docker
- Python >3 (It has been validated on `Python 3.10`)

You must set path to your RL4GreenROS folder:

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

First of all, let us set up the hosts that we are going to use in our experiment. Follow [this section](https://github.com/IntelAgir-Research-Group/robot-runner/tree/distributed-rr?tab=readme-ov-file#performing-a-simple-distributed-experiment) of Robot Runner documentation.

With everything set, you must run the orchestrator with the following command:

```bash
python3 $RL4GreenROS_PATH/docker/robot-runner/ $RL4GreenROS_PATH/docker/exp-orchestration/RL4GreenROS-RR-new.py
```

Note that in the current configuration, only 1 RR client is requested (Nav2). Gazebo and RViz are launched on the same machine as RR. The execution only begins when the RR client is connected. You can launch it by using the following command:

```bash
PYTHONPATH=$RL4GreenROS_PATH/docker/workspace/robot-runner/ python3 $RL4GreenROS_PATH/docker/exp-orchestration/RL4GreenROS-RR-Nav2.py
```

Now, your exeperiment must have been started.


## Customization

If you want to change something in the experiment, edit the `RL4GreenROS-RR.py` file. First, take a look at `Robot Runner (RR)` [tutorial](https://github.com/IntelAgir-Research-Group/robot-runner/tree/distributed-rr) to understand how it works.