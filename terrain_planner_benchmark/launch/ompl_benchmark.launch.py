"""Launch file for OMPL benchmark node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for OMPL benchmark."""
    # Declare launch arguments
    location_arg = DeclareLaunchArgument(
        'location',
        default_value='gotthard',
        description='Location name for terrain model'
    )

    visualization_arg = DeclareLaunchArgument(
        'visualization',
        default_value='false',
        description='Enable RViz visualization'
    )

    # Get launch configurations
    location = LaunchConfiguration('location')
    visualization = LaunchConfiguration('visualization')

    # Paths to terrain model files
    terrain_models_share = FindPackageShare('terrain_models')
    terrain_planner_share = FindPackageShare('terrain_planner')

    map_path = PathJoinSubstitution([
        terrain_models_share, 'models', [location, '.tif']
    ])

    color_file_path = PathJoinSubstitution([
        terrain_models_share, 'models', [location, '_color.tif']
    ])

    output_file_path = PathJoinSubstitution([
        terrain_planner_share, '..', 'output', 'output.log'
    ])

    # Static transform publisher
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_map',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    # OMPL benchmark node
    ompl_benchmark_node = Node(
        package='terrain_planner_benchmark',
        executable='ompl_benchmark_node',
        name='benchmark_planner',
        output='screen',
        parameters=[{
            'map_path': map_path,
            'color_file_path': color_file_path,
            'output_file_path': output_file_path,
        }]
    )

    # RViz node (conditional)
    rviz_config = PathJoinSubstitution([
        terrain_planner_share, 'launch', 'config_ompl_segments.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(visualization)
    )

    return LaunchDescription([
        location_arg,
        visualization_arg,
        static_tf_node,
        ompl_benchmark_node,
        rviz_node,
    ])

