from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in',  ['/segmentation/obstacle']),
            # remappings=[('cloud_in',  ['/depthimage_to_pointcloud2/point_cloud']),
            # remappings=[('cloud_in',  ['/terrain_map']),
            # remappings=[('cloud_in',  ['/camera/depth/color/points']),
                        ('scan',  ['/scan'])],
            parameters=[{
                'target_frame': 'livox_frame',#目标坐标系，即将激光扫描数据发布到的坐标系。
                'transform_tolerance': 0.01,#标变换的容差值，用于确定在坐标变换时允许的最大误差。
                'min_height': -0.7,#激光扫描的最小高度。在这里，设置为 -1.0。
                'max_height': 0.0,#激光扫描的最大高度。在这里，设置为 0.1
                'angle_min': -3.14159,  #激光扫描的最小角度。在这里，设置为 -3.14159（即 -π）。
                'angle_max': 3.14159,  # 激光扫描的最大角度。在这里，设置为 3.14159（即 π）
                'angle_increment': 0.0043,  # 角度增量，即每个激光扫描点之间的角度间隔。在这里，设置为 0.0043（大约为 π/360.0）
                'scan_time': 0.3333,#一次完整扫描所需的时间。在这里，设置为 0.3333 秒
                'range_min': 0.35,#激光扫描的最小测距范围。在这里，设置为 0.45 米。
                'range_max': 20.0,#激光扫描的最大测距范围。在这里，设置为 10.0 米
                'use_inf': True,#一个布尔值，表示是否使用无穷远处的测距数据。在这里，设置为 True
                'inf_epsilon': 1.0#inf_epsilon：当 use_inf 为 True 时，用于表示无穷远处的测距值。在这里，设置为 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])