"""Lancement des nœuds embarqués Dronoto.

Ce fichier ne référence AUCUN outil de simulation : il migrera tel quel sur le
calculateur embarqué. La propriété est vérifiée par
tests/conformance/test_bringup_no_sim_deps.py, pas seulement espérée.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("dronoto_bringup")
    default_params = os.path.join(share, "config", "drone_params.yaml")
    urdf_xacro = os.path.join(share, "urdf", "x500_dronoto.urdf.xacro")

    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Fichier de paramètres des nœuds embarqués.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="false",
            description="Démarrer la mission automatiquement. false par défaut : "
                        "lancer la pile ne doit jamais faire décoller un aéronef.",
        ),

        # --- description du robot et repères statiques ---
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                # value_type=str est obligatoire : sans lui, launch tente de
                # parser le XML produit par xacro comme du YAML et échoue.
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf_xacro]), value_type=str
                ),
            }],
        ),
        # map -> odom : identité en P1. Le graphe de poses la publiera en P4.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        ),

        # --- chaîne de vol ---
        Node(
            package="dronoto_interface",
            executable="px4_interface_node",
            name="px4_interface",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="dronoto_navigation",
            executable="trajectory_follower_node",
            name="trajectory_follower",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="dronoto_safety",
            executable="safety_supervisor_node",
            name="safety_supervisor",
            parameters=[params_file],
            # En P1, aucun évitement réactif ne s'intercale : le superviseur lit
            # directement la sortie du suivi de trajectoire. En P3 ce remappage
            # disparaît et reactive_avoidance produit control/setpoint_safe.
            remappings=[("control/setpoint_safe", "control/setpoint_raw")],
            output="screen",
        ),
        Node(
            package="dronoto_mission",
            executable="mission_executive_node",
            name="mission_executive",
            parameters=[params_file, {"autostart": autostart}],
            output="screen",
        ),
    ])
