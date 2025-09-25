### launch docker

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

### launch zenoh server bridge
```bash
zenohd
```

### launch zenoh server
```bash
zenohd
```

### launch zenoh server
```bash
zenohd
```