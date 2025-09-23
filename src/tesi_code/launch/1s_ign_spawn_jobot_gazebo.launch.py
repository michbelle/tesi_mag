import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
os.environ["ROS_DOMAIN_ID"] = "20"

def generate_launch_description():
    
    robot_name="jobot"
    
    
    # set_env_id_jobot=SetEnvironmentVariable('ROS_DOMAIN_ID', '20'),
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration("namespace")
    
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    declare_namespace_LA = DeclareLaunchArgument(
        "namespace",
        default_value="jobot")
    
    # Spawn Rover Robot
    
    
    # gz_spawn_entity_jobot=IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_spawn_model.launch.py")),
    #         launch_arguments={
    #             "topic":"/jobot/robot_description",
    #             "entity_name":"jobot"
    #         }.items()
    # )
    
    gz_spawn_entity_jobot = Node(
        package="ros_gz_sim",
        executable="create",
        name=f'{robot_name}_gz_create',
        arguments=[
            "-world", "",
            # "-topic", f"/{robot_name}/robot_description",
            "-topic", f"/robot_description",
            "-name", f"{robot_name}",
            # "-allow_renaming", "true",
            "-x", "43.02",
            "-y","-81.63",
            "-z", "0.1",
        ]
    )
    
    gz_ros2_bridge_jobot = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f'{robot_name}_gz_bridge',
        arguments=[
            f"/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            f"/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            f"/{robot_name}/odometry/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            f"/{robot_name}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            f'/{robot_name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'/{robot_name}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            f'/{robot_name}/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        remappings=[
            ("/"+robot_name+"/cmd_vel","/cmd_vel"),
            ("/"+robot_name+"/odometry/wheels","/odom"),
            ("/"+robot_name+"/tf","/tf"),
            ("/"+robot_name+"/joint_states","/joint_states"),
            ("/"+robot_name+"/scan","/scan"),
            ("/"+robot_name+"/imu/data","/arduino/imu_data_raw"),
        ],
    )

    # # Robot state publisher
        
    urdf = os.path.join(get_package_share_directory(
        'jobot_simulation'), "model",'urdf', 'jobot_robot.urdf')
    robot_desc = ParameterValue(Command(['xacro ', urdf]),
                                       value_type=str)
    params = {'use_sim_time': use_sim_time, 'robot_description': robot_desc}
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=f'{robot_name}_state_publisher',
            output='screen',
            parameters=[params],
            # namespace=f"{robot_name}",
            # remappings=[
            #     ('/robot_description', '/{robot_name}/robot_description'),
            # ],
            arguments=[])
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # ld.add_action(set_env_id_jobot)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    #add robot
    ld.add_action(gz_spawn_entity_jobot)
    ld.add_action(gz_ros2_bridge_jobot)

    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
