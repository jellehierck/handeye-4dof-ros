from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """Start calibration with RealSense camera."""
    # ---------- Define common paths ----------
    this_pkg_share_path = get_package_share_directory("handeye_4dof_ros")

    default_rviz2_config_path = str(Path(this_pkg_share_path) / "rviz" / "calibrate.rviz")

    # ---------- Declare launch arguments ----------

    launch_args = []

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="(bool) Whether to use simulation time.",
        choices=["true", "false"],
    )
    launch_args.append(use_sim_time_arg)

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz2_config_path,
        description="(string) Absolute path to RViz2 configuration.",
    )
    launch_args.append(rviz_config_arg)

    # ---------- Declare executables ----------

    camera_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        parameters=[
            {"use_sim_time": LaunchConfiguration(use_sim_time_arg.name)},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        # arguments=["-d", LaunchConfiguration(rviz_config_arg.name)],
        parameters=[
            {"use_sim_time": LaunchConfiguration(use_sim_time_arg.name)},
        ],
    )

    # ---------- Construct launch description ----------

    return LaunchDescription(
        [
            *launch_args,  # Unpack the launch arguments
            camera_node,
            rviz_node,
        ]
    )
