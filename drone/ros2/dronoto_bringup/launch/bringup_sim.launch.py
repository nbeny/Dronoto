"""Lancement complet en simulation : PX4 SITL + Gazebo + agent + nœuds embarqués.

Sépare délibérément la simulation (ce fichier) des nœuds embarqués
(bringup_drone.launch.py), qui restent réutilisables tels quels sur matériel.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("dronoto_bringup")
    repo_root = os.environ.get("DRONOTO_REPO", os.path.expanduser("~/Dronoto"))
    run_sim = os.path.join(repo_root, "infrastructure", "scripts", "run_sim.sh")

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("rviz")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(share, "config", "drone_params.yaml"),
        ),

        ExecuteProcess(
            cmd=["bash", run_sim],
            additional_env={"HEADLESS": headless},
            output="screen",
            name="px4_gazebo",
        ),

        # Laisser PX4 et Gazebo démarrer avant les nœuds embarqués : sans ce
        # délai px4_interface attend simplement, mais les journaux deviennent
        # illisibles et le diagnostic d'un vrai problème plus difficile.
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(share, "launch", "bringup_drone.launch.py")
                    ),
                    launch_arguments={
                        "autostart": autostart,
                        "params_file": params_file,
                    }.items(),
                ),
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", os.path.join(share, "rviz", "dronoto.rviz")],
            condition=IfCondition(use_rviz),
        ),
    ])
