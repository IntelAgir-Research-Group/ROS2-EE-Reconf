# GPU Accelerated Simulation Machine at ISW

## 1. Accessing the Machine

### Local access

The machine is located at ISW in the Student-Lab. To get the Passowrd please contact @github/michelalbonico. Usually the machine is left with a turned off screen (more information below). So if you want to use it, make sure tu turn it on first.

<a name="leaving-the-machine"></a>
Idle state specifications for the machine:

- Since we set up remote access via TeamViewer, make sure **TeamViewer is up and running** bevore leaving.
- **Lock the PC** with `WIN + L` (don't sleep or shut down).
- **Turn of the screen** so if someone else accedentally moves the mouse or touches the keyboard, they don't see that the PC is running. This mitigates the risk of them shutting down the PC.
- Idealy you should **leave a note on the keyboard** that says somathing link: `Simulation running - Do not turn off`

### Remote Access
We set up TeamViewer for remote controll. Currently @github/michelalbonico has access to it. Please contact him for further information about this.

> Sometimes TeamViewer fails. In this case, just restart the machine. TeamViewer is set as a start programm but please make sure that it's actutally started and running before leaving. Also make sure you leave the machine in it's [idle state specifications](#leaving-the-machine).

## 2. Running Simulations

When running the project on a machine for the first time there are a couple of things, that have to be setup **in advance**. Please refere to [this](https://github.com/IntelAgir-Research-Group/ROS2-EE-Reconf/blob/main/README.md) documentation for details. The standard procedure to start the simulation after setup is running the `GreenROS-RR-reconf.py` file.

```bash
python3 $RR_PATH/robot-runner/ $RL4GreenROS_PATH/exp_orchestration/GreenROS-RR-reconf.py
```

### Navigation Fail

In the past the Robot sometimes got stuck on a walls and therefore the simulation didn't continue to run. This issue is already handled but in case the simulation fails, just closing the windows of RViz and Nav2 is not enough. There are still leftovers in Docker that have to be removed befor a successfull restart. This is easily done by a script:

```bash
bash $RL4GreenROS_PATH/clean-containers.sh
```

> Tipp: If you need to check a single Docker container (e.g. this helped troubleshooting the setup for the tool: nvidia container toolkit), they can be started independently:\
`sudo docker-compose -f docker/compose/compose-gazebo.yml up gazebo`,\
`sudo docker-compose -f docker/compose/compose-nav2.yml up nav2` or\
`sudo docker-compose -f docker/compose/compose-rviz.yml up rviz`

## 3. Using a Nvidia GPU to Accelerate Docker Container for Sumulation

One of the callanges we faced, was to use the GPUs memory to help calculate the simulations. Therefore a tool called Nvidia Container Toolkit is utilized which enables Docker containers to be configured through yaml to use a Nvidia GPU. To install the tool, follow the steps of the [official installation documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) for Ubuntu, Debian using `apt`.


Once installed activate the tool:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Test if the tool is properly installed:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

Next, the tool has to be called from the docker container its supposed to run in. You can use `--gpus` to start containers with the tool from the comand line. We usually use docker compose (`.yml` or `.yaml`) files to provide replication. A basic setup only includes the `capabilities` value which **has to be set** in every container for the tool to work.

```yaml
services:
  <your_service_name>:
    image: <your_image>
    # the following is the importent part
    command: nvidia-smi
    deploy:
      resources:
        reservations:
          devices:
            - count: all
              capabilities: [gpu]
```

This way the machine uses all available grafics cards and their idle drivers. To test if it works, start your Docker container and execute `nvidia-smi` in the terminal to see all graphic card activities. Down in the list of processes should be an entry of `gzserver` and `gzserver`. This validates that the crafics card is rendering the simulation.

> If you need more information about configuration of the `.yml` or `.yaml` and usage see take a look at [this website](https://docs.docker.com/compose/how-tos/gpu-support/).

## 4. Machine Configuration

### Hardware Components

- RAM: 16 GB (DDR4?)
- CPU: 12th Gen Intel Core i7-12700F
- GPU: Nvidia GeForce RTX 3060 

### Software Versions

- Docker version 27.4.0
- Gazebo
- Linux Ubuntu 22.04` (kernel 6.8.0-65-generic)
- Nav2
- Nvidia Container Toolkit
- Power Jolar
- Python 3
- Robot Runner
- ROS 2 Humble
- RViz
- Team Viewer