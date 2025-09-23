save topic to simulate beahaviours


avaible topics
```bash
/cmd_vel
/imu/data
/joint_states
/magnetometer
/odometry/wheels
/orientation
/parameter_events
/robot_description
/robot_info
/robot_status
/rosout
/rover_mini/battery_status
/scan
/soft_estop/reset
/soft_estop/trigger
/temperature
/tf
/tf_static
/trim_event
```


record
```bash
ros2 bag record \
    -o record_007 \
    /imu/data \
    /joint_states \
    /magnetometer \
    /odometry/wheels \
    /orientation \
    /parameter_events \
    /robot_description \
    /robot_info \
    /robot_status \
    /rover_mini/battery_status \
    /scan \
    /temperature \
    /tf \
    /tf_static \
    /trim_event 
```


play
```bash
ros2 bag play \
    record_001 \
    --topics \
    <topic 1> \ ...
    --clock
    -r 1
```

playing with simulation time
```
ros2 bag play record_001 --clock -r 5
```

### Dowload 

https://drive.elettra.eu/d/572e23d488d842128720/