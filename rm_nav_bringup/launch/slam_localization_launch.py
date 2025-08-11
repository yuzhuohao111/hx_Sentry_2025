
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world') 
    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_toolbox_map_dir = PathJoinSubstitution([get_package_share_directory("rm_nav_bringup"), 'map',world])

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("rm_nav_bringup"),
                                   'config', 'mapper_params_localization.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='5',
        description='Select world (map file, pcd file, world file share the same name prefix as the this parameter)')
    
    start_localization_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time,
           'map_file_name': slam_toolbox_map_dir}
        ],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_world_cmd) 
    ld.add_action(start_localization_slam_toolbox_node)

    return ld