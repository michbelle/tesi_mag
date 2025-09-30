### launch docker

```bash
xhost local:root
```

```bash
docker run -it \
    -v .:/root/openRMF_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro --device=/dev/dri:/dev/dri -e DISPLAY=$DISPLAY \
    --privileged \
    --network host \
    --name serverNavRos \
    tesi_image \
    bash
```

```bash
docker run -it \
    -v .:/root/openRMF_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro --device=/dev/dri:/dev/dri -e DISPLAY=$DISPLAY \
    --runtime=nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
    --privileged \
    --network host \
    --name serverNavRos \
    tesi_image \
    bash
```

```bash
docker run -it \
        --network=host \
        --cap-add=SYS_PTRACE \
        --security-opt=seccomp:unconfined \
        --security-opt=apparmor:unconfined \
        --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
        --volume=/mnt/wslg:/mnt/wslg \
        --ipc=host \
        --volume=/run/user/1000:/run/user/1000 \
        -e DISPLAY=$DISPLAY \
        -e XAUTHORITY=$XAUTHORITY \
        -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
        -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        -e PULSE_SERVER=$PULSE_SERVER \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        --runtime=nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
        --user ros \
        -v .:/openRMF_ws \
        --name serverNavRos \
        tesi_image_v3 \
        bash
```

## build comand

```bash
colcon build --symlink-install
```


### launch simul world

```bash
ros2 launch tesi_code 0s_ign_spawn_world_gazebo.launch.py
```

#### add mini with ROS_ID 10

```bash
ros2 launch tesi_code 1s_ign_spawn_mini_gazebo.launch.py
```

#### add jobot with ROS_ID 20

```bash
ros2 launch tesi_code 1s_ign_spawn_jobot_gazebo.launch.py
```

#### launch mini navigation stack 
```bash
ros2 launch mini_launchpad Snav2_global.launch.py
```

#### launch jobot navigation stack 
```bash
ros2 launch jobot_launchpad Snav2_global.launch.py
```

## Zenoh communication

### launch zenoh server
```bash
zenohd
```

### launch zenoh bridge server 
```bash
/openRMF_ws/src/tesi_code/zenoh/simul_server_rmf_zenoh.sh
```

### launch zenoh bridge mini
```bash
/openRMF_ws/src/tesi_code/zenoh/simul_mini_zenoh.sh
```

### launch zenoh bridge jobot
```bash
/openRMF_ws/src/tesi_code/zenoh/simul_jobot_zenoh.sh
```

## rmf server

### rmf core

```bash
export ROS_DOMAIN_ID=30
ros2 launch rmf_server_elettra 0_rmf_core.launch.xml
```

### rmf api and dashboard

to launch on server **outside of docker**

```bash
docker compose -f <(curl -s https://raw.githubusercontent.com/michbelle/RMF_server/cb467cfdddeabfc3a877e32934d323016416cd2b/Docker_f/dockerCompose_api_dashboard/compose.yml) up
```

### rmf fleet adapter for mini
```bash
export ROS_DOMAIN_ID=30
ros2 launch rmf_server_elettra 1rmf_mini_fleet_adapter.launch.xml server_uri:="ws://localhost:8000/_internal"
```

### rmf fleet adapter for mini
```bash
export ROS_DOMAIN_ID=30
ros2 launch rmf_server_elettra 1rmf_jobot_fleet_adapter.launch.xml server_uri:="ws://localhost:8000/_internal"
```
