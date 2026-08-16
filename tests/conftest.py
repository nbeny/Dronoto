"""Configuration pytest partagée."""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def pytest_collection_modifyitems(config, items):
    """Ignore les scénarios SITL si l'environnement ROS 2 n'est pas sourcé.

    Les tests de conformité sont du Python pur et doivent tourner partout,
    y compris sur un runner CI sans ROS. Les scénarios, eux, ont besoin de
    rclpy, de dronoto_msgs et d'une simulation : les lancer sans environnement
    produirait un ImportError illisible plutôt qu'un message utile.
    """
    import pytest

    if "AMENT_PREFIX_PATH" in os.environ:
        return

    skip = pytest.mark.skip(
        reason="environnement ROS 2 non sourcé — "
               "source /opt/ros/jazzy/setup.bash && source ~/dronoto_ws/install/setup.bash"
    )
    for item in items:
        if "scenario" in item.keywords:
            item.add_marker(skip)
