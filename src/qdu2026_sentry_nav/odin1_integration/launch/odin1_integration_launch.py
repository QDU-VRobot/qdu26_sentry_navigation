import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("odin1_integration")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if true"
    )

    # TF Inverter Node (odom->map to map->odom)
    tf_inverter_node = Node(
        package="odin1_integration",
        executable="tf_inverter_node",
        name="tf_inverter",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "source_frame": "odom",
            "target_frame": "map",
            "inverted_source_frame": "map",
            "inverted_target_frame": "odom",
            "publish_rate": 20.0
        }]
    )

    # Point Cloud Fusion Node
    pointcloud_fusion_node = Node(
        package="odin1_integration",
        executable="pointcloud_fusion_node",
        name="pointcloud_fusion",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "odin1_topic": "odin1/cloud_raw",
            "mid360_topic": "/livox/lidar",
            "output_topic": "fused_pointcloud",
            "output_frame": "base_footprint",
            "publish_rate": 10.0,
            "sync_tolerance": 0.1
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(tf_inverter_node)
    ld.add_action(pointcloud_fusion_node)

    return ld
