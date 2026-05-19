# my master's thesis

Creation of two AMRs using ROS (one in Humble and the other in Jazzy) for moving objects within Elettra Sincrotrone Trieste using the fleet manager Open-RMF

### Folder structure

```bash
.
├── .devcontainer # docker instruction 
├── 000_sequence # sequence of commands to launch 
├── rosbags # information of how the data were recorder 
│   ├── jobot
│   ├── mini
│   └── results #TODO
├── src # each code used in this thesis
│   ├── DStar_Trajectory_Planner # planner ported from ROS 1 to ROS 2
│   ├── jobot_driver_ros2 # used to control for the jobot AMR in jazzy ported from ROS 1 to ROS 2 
│   ├── mini_rover_code # used to control of the Mini AMR in humble (simulation available in jazzy)
│   ├── rmf_server # to control the fleets of the AMR  based on Open-RMF
│   └── tesi_code # that launch each part of the other folders
└── testo # "documentation"
```


### Download

```bash
git clone --recurse-submodules 
```

if downloaded without recursive the first time you need to launch

```bash
git submodule update --init --recursive
```

else

```bash
git submodule update --recursive --remote
```

## Build code

```bash
colcon build --symlink-install
```

### Build docker image

This build a docker image with all the code, ready to play:

if need ssh agent
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

if launch without VScode extensions

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

If the PC has an nvidia GPU, it can be connected to the docker image

```bash
        --runtime=nvidia
        --runtime=nvidia --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \

```