import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch.conditions import IfCondition

from launch.actions import ExecuteProcess

def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    world = LaunchConfiguration('world')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='elettra',
        description='World dir to use in Gazebo')
    
    world_name = LaunchConfiguration('world_name')
    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name',
        default_value='ign-elettra-map.world',
        description='World file to use in Gazebo')
    
    headless = LaunchConfiguration("headless")
    headless_dcl = DeclareLaunchArgument(
        'headless',
        default_value='False')
    
    #add general path for simulation 
    
    new_path = PathJoinSubstitution([get_package_share_directory("tesi_code"), 'ign_world', world, "modelOUT"])
    set_env_cmd=AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_path)
    
    
    
    gz_world_arg = PathJoinSubstitution([get_package_share_directory("tesi_code"), 'ign_world', world, world_name])
    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', gz_world_arg],
        output='screen',
    )
    
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        condition=IfCondition(PythonExpression(["not ", headless])),
        launch_arguments={
            "gz_args" : ["-v4 -g "],
        }.items(),
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    #set env variable
    # Declare the launch options
    ld.add_action(headless_dcl)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(set_env_cmd)
    ld.add_action(declare_world_name_cmd)

    # Launch Gazebo
    ld.add_action(gz_sim)
    ld.add_action(gz_sim_gui)

    return ld