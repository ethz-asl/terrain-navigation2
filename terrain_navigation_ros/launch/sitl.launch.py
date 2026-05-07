import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_terrain_navigation_action(
    context: LaunchContext, *args, **kwargs
) -> [ExecuteProcess]:
    """Generate the terrain_navigation_ros launch action."""

    pkg_terrain_navigation_ros = get_package_share_directory("terrain_navigation_ros")

    # arguments.
    location = LaunchConfiguration("location").perform(context)
    minimum_turn_radius = float(
        LaunchConfiguration("minimum_turn_radius").perform(context)
    )
    alt_control_p = float(LaunchConfiguration("alt_control_p").perform(context))
    alt_control_max_climb_rate = float(
        LaunchConfiguration("alt_control_max_climb_rate").perform(context)
    )
    cruise_speed = float(LaunchConfiguration("cruise_speed").perform(context))
    px4_namespace = LaunchConfiguration("px4_namespace").perform(context)

    # terrain navigation node
    resource_path = os.path.join(pkg_terrain_navigation_ros, "resources")
    terrain_path = os.path.join(resource_path, location + ".tif")
    terrain_color_path = os.path.join(resource_path, location + "_color.tif")
    meshresource_path = "terrain_navigation_ros/resources/believer.dae"

    # Create action.
    node = Node(
        package="terrain_navigation_ros",
        executable="terrain_planner_node",
        name="terrain_planner",
        parameters=[
            {"meshresource_path": meshresource_path},
            {"minimum_turn_radius": minimum_turn_radius},
            {"resource_path": resource_path},
            {"terrain_path": terrain_path},
            {"terrain_color_path": terrain_color_path},
            {"alt_control_p": alt_control_p},
            {"alt_control_max_climb_rate": alt_control_max_climb_rate},
            {"cruise_speed": cruise_speed},
            {"px4_namespace": px4_namespace},
        ],
        output="screen",
    )
    return [node]


def generate_launch_description():
    """Generate a launch description for SITL (micro-ros-agent + terrain_planner + rviz)."""

    pkg_terrain_navigation_ros = get_package_share_directory("terrain_navigation_ros")
    pkg_terrain_navigation_planner = get_package_share_directory("terrain_planner")
    
    # Default output directory for rosbag recordings - use source directory
    workspace_root = os.path.join(pkg_terrain_navigation_ros, "..", "..", "..", "..")
    default_output_dir = os.path.join(workspace_root, "src", "terrain-navigation", "output")

    # micro-ros-agent process
    micro_ros_agent = ExecuteProcess(
        cmd=[
            "micro-ros-agent",
            "udp4",
            "-p",
            LaunchConfiguration("micro_ros_port"),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("micro_ros_agent")),
    )

    # terrain navigation node
    terrain_navigation = OpaqueFunction(function=generate_terrain_navigation_action)

    # rviz node
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_terrain_navigation_planner, "launch", "config.rviz"),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            # micro-ros-agent arguments
            DeclareLaunchArgument(
                "micro_ros_agent",
                default_value="true",
                description="Launch micro-ros-agent.",
            ),
            DeclareLaunchArgument(
                "micro_ros_port",
                default_value="8888",
                description="UDP port for micro-ros-agent.",
            ),
            # terrain_planner arguments
            DeclareLaunchArgument(
                "location", default_value="sacramento", description="Location."
            ),
            DeclareLaunchArgument(
                "minimum_turn_radius",
                default_value="80.0",
                description="Minimum turn radius.",
            ),
            DeclareLaunchArgument(
                "alt_control_p",
                default_value="0.5",
                description="Altitude controller proportional gain.",
            ),
            DeclareLaunchArgument(
                "alt_control_max_climb_rate",
                default_value="3.0",
                description="Altitude controller maximim climb rate (m/s).",
            ),
            DeclareLaunchArgument(
                "cruise_speed",
                default_value="15.0",
                description="Vehicle cruise speed (m/s).",
            ),
            DeclareLaunchArgument(
                "px4_namespace",
                default_value="",
                description="Namespace prefix for PX4 topics (e.g., '/px4_1').",
            ),
            # rviz arguments
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value=default_output_dir,
                description="Directory path for rosbag output.",
            ),
            # Actions
            micro_ros_agent,
            terrain_navigation,
            rviz,
        ]
    )

