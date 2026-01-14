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
    -b 1000000000 \
    --compression-mode file \
    -o jobot_record_002 \
    /bond \
    /serial_status \
    /diagnostics \
    /cmd_vel \
    /parameter_events \
    /joint_states \
    /uls_debug \
    /tf \
    /arduino/imu_data_raw \
    /scan \
    /jobot_driver_ros2/transition_event \
    /robot_description \
    /tf_static \
    /covariance \
    /odom \
    /battery_capacity \
    /current
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

https://drive.elettra.eu/d/b906f8bf0e054df18998/
