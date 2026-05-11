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
    config_file = LaunchConfiguration("config_file")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation clock if true"
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(pkg_dir, "config", "static_tf.yaml"),
        description="Path to static TF configuration file"
    )

    # Static TF publisher node (using tf2_ros static_transform_publisher)
    # This will be configured via parameters from the YAML file
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mid360_to_odin1_tf",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "front_mid360",
            "--child-frame-id", "odin1_base_link"
        ]
    )



    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(static_tf_node)

    return ld
