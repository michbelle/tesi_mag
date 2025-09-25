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
docker run -it --rm  \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro --device=/dev/dri:/dev/dri -e DISPLAY=$DISPLAY \
    --privileged \
    --network host \
    --name serverNavRos \
    tesi_image \
    bash
```


### tmp commands
# [submodule "src/jobot_driver_ros2"]
# 	path = src/jobot_driver_ros2
# 	url = git@github.com:michbelle/jobot_driver_ros2.git
# 	branch = jazzy
# [submodule "src/DStar_Trajectory_Planner"]
# 	path = src/DStar_Trajectory_Planner
# 	url = git@github.com:ElettraSciComp/DStar-Trajectory-Planner.git
# 	branch = jazzy
git submodule add -b jazzy git@github.com:michbelle/jobot_driver_ros2.git src/jobot_driver_ros2
git submodule add -b jazzy git@github.com:ElettraSciComp/DStar-Trajectory-Planner.git src/DStar_Trajectory_Planner
