import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.logging import get_logger

import os
os.environ["ROS_DOMAIN_ID"] = "10"

def generate_launch_description():
    # Create the launch configuration variables
    logger = get_logger('launch_logger')
    logger.info('launching simulation with ROS_DOMAIN_ID : 10')
    
    robot_name="mini"

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true")

    declare_namespace_LA = DeclareLaunchArgument(
        "namespace",
        default_value="mini")
    
    gz_spawn_entity_rover_mini = Node(
        package="ros_gz_sim",
        executable="create",
        name="mini_gz_create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "rover_"+robot_name+"",
            "-allow_renaming", "true",
            "-x", "53.84",
            "-y","-62.60",
            "-z", "0.1",
        ],
    )

    gz_ros2_bridge_rover_mini = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="mini_gz_bridge",
        arguments=[
            "/"+robot_name+"/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/"+robot_name+"/odometry/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/"+robot_name+"/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/"+robot_name+"/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/"+robot_name+"/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/"+robot_name+"/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        remappings=[
            ("/"+robot_name+"/cmd_vel","/cmd_vel"),
            ("/"+robot_name+"/odometry/wheels","/odometry/wheels"),
            ("/"+robot_name+"/tf","/tf"),
            ("/"+robot_name+"/joint_states","/joint_states"),
            ("/"+robot_name+"/scan","/scan"),
            ("/"+robot_name+"/imu/data","/imu/data"),
        ],
    )

    # Robot state publisher
    urdf = os.path.join(get_package_share_directory(
        "mini_simulation"), "urdf", "mini.urdf")
    
    robot_desc = ParameterValue(Command(["xacro ", urdf]),
                                       value_type=str)
    
    params = {"use_sim_time": use_sim_time, "robot_description": robot_desc}
    
    start_robot_state_publisher_cmd = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name=robot_name+"_state_publisher",
            output="screen",
            # namespace=robot_name,
            parameters=[params],
            # remappings=[
            #     ("/tf", "/mini/tf"),
            #     ("/tf_static", "/mini/tf_static"),
            # ],
            arguments=[])

    # Create the launch description and populate
    ld = LaunchDescription()

    # ld.add_action(set_env_id_mini)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_LA)

    #add robot
    ld.add_action(gz_spawn_entity_rover_mini)
    ld.add_action(gz_ros2_bridge_rover_mini)

    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
