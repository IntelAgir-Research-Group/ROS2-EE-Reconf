# Reconfiguring ROS 2 for Energy Efficiency

This is the replication package of the paper `Tuning ROS 2 for Greener Navigation: Experimental Insights from Costmap 2D Configurations` submitted to `ICRA 2026`.

## Table of Contents

- [Workspace](#workspace)
- [Configurations](#configurations)
- [Docker Setup](#docker-setup)
  - [Running GUI from Docker](#running-gui-from-docker)
- [Running the Experiments](#runnin-the-experiments)
- [Data Analysis](#data-analysis)

### Workspace

For building the package (ONLY WHEN SOMETHING IS CHANGED), we use a `Docker` container:

```bash
docker-compose -f docker/compose/docker-compose.yml up robot-runner
docker exec -it robot-runner_container bash
cd /ros-ws
source /opt/ros/humble/setup.bash
colcon build
```

You can also simply push the changes to GitHub, which automatically builds the `package`. Then, you must pull the changes locally. `GitHub Action TBA`

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

And generate the configurations from the list of possible parameter values (ONLY FIRST TIME):

```bash
cd config/
python3 gen_pairwise_confs.py
```

Then, the `yaml` files can be generated (ONLY FIRST TIME):

```bash
TBA
```

### Docker Setup

All the computation runs on Docker containers. Please, install `Docker` and `Docker Compose`. It is also important that you set you user to run `docker` [rootless](https://docs.docker.com/engine/security/rootless/) commands. 

We keep services in separate compose files in the `docker/compose` folder.

To run a specific service:

```bash
$ docker-compose -f <file.yml> up SERVICE
```

Example of a complete run (run in three separate terminals):

Gazebo:
```bash
$ docker-compose -f docker/compose/compose-gazebo.yml up gazebo
```

Nav2:
```bash
$ docker-compose -f docker/compose/compose-nav2.yml up nav2
```

RVIZ:
```bash
$ docker-compose -f docker/compose/compose-rviz.yml up rviz
```

#### Running GUI from Docker

This is a work around that helps you to run GUI from Docker containers. From Ubuntu 19, Xorg does not allow TCP connections by default, so we must enable it. Another point, when logging in, be sure to choose Xorg as window manager. 

Then, follow the next steps:

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

Now, you can start GUI apps from your Docker containers. This is important to visually check everything is working accordingly. *For the experiments, we set GUI mode off*.

## Running the Experiments

For running the experiments, you must have the `robot-runner` [repository](https://github.com/S2-group/robot-runner) cloned and set the environment variable:

```bash
export RR_PATH='../robot-runner'
```

Then, move to [this](./exp-orchestration/) tutorial/documentation.

## Data Analysis

The data analysis scripts, graphs, and logs can be found in [this](./analysis/) folder.
