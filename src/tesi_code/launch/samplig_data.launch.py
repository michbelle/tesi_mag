import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import SetEnvironmentVariable
from pathlib import Path

import os
os.environ["ROS_DOMAIN_ID"] = "10"

def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')

    # info_domain_id=LogInfo(msg='launching simulation on ROS_DOMAIN_ID : 10'),

    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Include the gz sim launch file  
    launch_folder = get_package_share_directory("mini_launchpad")

    launch_nav2_simul = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_folder, "launch", "Snav2_global.launch.py")),
        launch_arguments={
        }.items()
    )


    config_path = Path(get_package_share_directory('tesi_code'), 'config', 'utility', 'odom_config.yaml')
    record_data_node = Node(
        package='tesi_code',
        executable='record_data.py',
        name='odom_print',
        output='screen',
        parameters=[config_path]
        )
    

    ld = LaunchDescription()
    # ld.add_action(info_domain_id)
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Launch Gazebo
    ld.add_action(launch_nav2_simul)
    ld.add_action(record_data_node)    

    return ld