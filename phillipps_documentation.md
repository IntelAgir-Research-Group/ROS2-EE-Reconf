# Utilize Nvidia GPU in Docker Containers

## Installation

Followed the steps under "With apt: Ubuntu, Debian"

https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

## Coniguration

(Also in the installation guid)
Activate the tool:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Test if it works:

```bash
docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

## Usage in Containers

You can use `--gpus` to start containers with the tool from the comand line. We usually use docker compose (`*.yml`) files to provide replication so thats what will be shown here. A basic setup only includes the `capabilities` value which has to be set, otherwise it produces an error.

```yaml
services:
  <service_name>:
    image: <image>
    command: nvidia-smi
    deploy:
      resources:
        reservations:
          devices:
            - count: all
              capabilities: [gpu]
```

This way the machine uses all available grafics cards and the idle drivers.

Test if it works:

```bash
nvidia-smi
```

Down in the list of processes should be an entry of `gzserver` and `gzserver`.

If you need more information about configuration and usage see [this link](https://docs.docker.com/compose/how-tos/gpu-support/)