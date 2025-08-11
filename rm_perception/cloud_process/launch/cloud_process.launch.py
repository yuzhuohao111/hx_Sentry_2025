from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
def generate_launch_description():
    configured_params = PathJoinSubstitution([
        FindPackageShare('cloud_process'),
        'config',
        'cloud_process.yaml'
    ])

    return LaunchDescription([
        Node(
            package='cloud_process',
            executable='cloud_process_node',
            name='cloud_process',
            output="screen",
            namespace="",
            remappings=[
                ("/tf", "tf"), 
                ("/tf_static", "tf_static")
            ],
            parameters=[configured_params]
        )
    ])