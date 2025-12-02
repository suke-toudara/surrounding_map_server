import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('pcd_range_publisher')

    # Default config file path
    default_config = os.path.join(pkg_dir, 'config', 'params.yaml')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the config file'
    )

    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value='',
        description='Path to PCD file (overrides config file if set)'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Publishing rate in Hz'
    )

    range_radius_arg = DeclareLaunchArgument(
        'range_radius',
        default_value='50.0',
        description='Radius around robot to extract points (meters)'
    )

    voxel_leaf_size_arg = DeclareLaunchArgument(
        'voxel_leaf_size',
        default_value='0.2',
        description='Voxel grid filter leaf size (meters)'
    )

    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    pcd_file = LaunchConfiguration('pcd_file')
    publish_rate = LaunchConfiguration('publish_rate')
    range_radius = LaunchConfiguration('range_radius')
    voxel_leaf_size = LaunchConfiguration('voxel_leaf_size')

    # Node
    pcd_range_publisher_node = Node(
        package='pcd_range_publisher',
        executable='pcd_range_publisher_node',
        name='pcd_range_publisher',
        output='screen',
        parameters=[
            config_file,
            {
                'pcd_file_path': pcd_file,
                'publish_rate': publish_rate,
                'range_radius': range_radius,
                'voxel_leaf_size': voxel_leaf_size,
            }
        ],
        remappings=[
            # Add remappings here if needed
        ]
    )

    return LaunchDescription([
        config_file_arg,
        pcd_file_arg,
        publish_rate_arg,
        range_radius_arg,
        voxel_leaf_size_arg,
        pcd_range_publisher_node,
    ])
