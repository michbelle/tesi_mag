# Tesi laurea magistrale

## Mini

basato su humble

## resto basato su jazzy

### git dowload

if the first time you need to launch

```bash
git submodule update --init --recursive
```

else

```bash
git submodule update --recursive --remote
```

```bash
git pull --recurse-submodules
```


## build comand

```bash
colcon build --symlink-install
```

### devcontainer

start ssh agent
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ecdsa
```

then run
```bash
DOCKER_BUILDKIT=1 docker build -t tesi_image . --ssh default
```

then launch the image with all build in with

share x with:
```bash
xhost local:root
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
        -v .:/home/ros/openRMF_ws \
        --user ros \
        tesi_image \
        bash
```

```bash
        --runtime=nvidia
        --runtime=nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \

```


# temp
```bash
ros2 run rviz2 rviz2 -d /openRMF_ws/src/mini_rover_code/src/mini_launchpad/rviz/mini_nav.rviz

ros2 bag play rosbags/mini/mini_record_007 --clock

ros2 launch mini_launchpad Snav2_global.launch.py

ros2 run tesi_code record_data.py
```