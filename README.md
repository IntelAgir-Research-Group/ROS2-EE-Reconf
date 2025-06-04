# Reconfiguring ROS 2 for Energy Efficiency

This is the replication package of the paper `TITLE` on the reconfiguration of ROS 2 packages for energy efficiency.

## Table of Contents

- [Workspace](#workspace)
- [Configurations](#configurations)
- [Docker Setup](#docker-setup)
  - [Running GUI from Docker](#running-gui-from-docker)
- [Running the Experiments](#runnin-the-experiments)
- [Data Analysis](#data-analysis)

### Workspace

Here is where we develop customized ROS 2 package (`reconfros2`) for orchestration and profiling.

For building the package, we use a `Docker` container:

```bash
docker-compose -f docker/compose/compose-ros-humble-nav2.yml up reconfros2
docker exec -it reconfros2_container bash
cd /ros-ws
source /opt/ros/humble/setup.bash
colcon build
```

You can also simply push the changes to GitHub, which automatically builds the `package`. Then, you must pull the changes locally.

TBA -- GitHub Action

### Configurations

Setup a virtual environment for Python:

```bash
python3 -m venv ./venv
source .venv/bin/activate
```

Then, install the dependencies:

```bash
pip3 install -r requirements.txt
```

And generate the configurations from the list of possible parameter values:

```bash
cd config/
python3 gen_pairwise_confs.py
```

Then, the `yaml` files can be generated:

```bash
python3 
```

### Docker Setup

All the computation runs on Docker containers. We keep services in a single compose files in the `docker/compose` folder.

To run a specific service:

```bash
$ sudo docker-compose -f <file.yml> up SERVICE
```

Example:

```bash
$ sudo docker-compose -f docker/compose/compose-ros-humble.yml up ros-humble
```

#### Running GUI from Docker

Unfortunately, so far we could not work by using the graphic card from the kernel module. :(

However, there is a work around that helps us to run GUI from Docker containers. From Ubuntu 19, Xorg does not allow TCP connections by default, so we must enable it. For this, follow the next steps:

1. Edit the `/etc/X11/Xwrapper.config` as the following:

```
#allowed_users=console
allowed_users=anybody
```

2. In the `/usr/bin/Xorg` file, Xwrapper every commands should be like this:

```
exec "$basedir"/Xorg.wrap "$@" -listen tcp
```

3. Restart Xorg:

```bash
sudo systemctl restart gdm
```

4. Allow any host to connect to Xorg (you can disable after experiments):

```bash
xhost +
xhost +si:localuser:root
```

Now, you can start GUI apps from your Docker containers. This is important to visually check everything is working accordingly.

For the experiments, we set GUI mode off.

## Running the Experiments

For running the experiments, you must have the `robot-runner` repository clonned and set the environment variable:

```bash
export RR_PATH='../robot-runner'
```

We have some customized plugins in ...

The configurations are generated in folder `config` as YAML files.

All the experiment is set on the RR configuration file: `./exp-orchestration/RR-ReconfROS.py`. The results are saved in the `exp-orchestration/data` folder.'

Before starting the experiments, both computers must have the exact same copy this repository.

#### On Computer 1 (Gazebo environment)

Start the Gazebo environment with the following command:

```bash
...
```

#### On Computer 2 (Nav2 Stack)

You must source this repository:

```bash
source setup.bash
```

Then, the following command runs all the experiment trials (if interruppted, RR restarts from the point where the previous exection stopped):

```bash
...
```

## Data Analysis

TBA
