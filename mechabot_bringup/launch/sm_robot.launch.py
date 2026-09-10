import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# -------------------------
# RViz configuration for SLAM
# -------------------------
slam_rviz_config_path = os.path.join(
    get_package_share_directory("mechabot_mapping"),
    "rviz",
    "slam.rviz"
)


def generate_launch_description():

    # -------------------------
    # Gazebo
    # -------------------------
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mechabot_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            "world_name": "small_house"
        }.items()
    )

    # -------------------------
    # ros2_control
    # -------------------------
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mechabot_controller"),
            "launch",
            "controller.launch.py"
        )
    )

    # -------------------------
    # SLAM Toolbox
    # -------------------------
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mechabot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    # -------------------------
    # Nav2
    # -------------------------
    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mechabot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    # -------------------------
    # Frontier exploration config
    # -------------------------
    explore_config_path = os.path.join(
        get_package_share_directory("mechabot_bringup"),
        "config",
        "explore.yaml"
    )

    # -------------------------
    # Frontier exploration
    # -------------------------
    exploration = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        parameters=[
            explore_config_path,
            {"use_sim_time": True}
        ]
    )

    # -------------------------
    # Wait 60 seconds before exploration
    # -------------------------
    exploration_delayed = TimerAction(
        period=60.0,
        actions=[exploration]
    )

    # -------------------------
    # RViz
    # -------------------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=[
            "-d",
            slam_rviz_config_path
        ]
    )

    # -------------------------
    # Launch everything
    # -------------------------
    return LaunchDescription([
        gazebo,
        controller,
        slam,
        navigation,
        rviz,
        exploration_delayed,
    ])