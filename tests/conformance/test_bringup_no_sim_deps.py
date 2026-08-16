"""bringup_drone.launch.py doit migrer tel quel sur le calculateur embarqué.

Propriété de docs/architecture/04-architecture-ros2.md, section 8 : le fichier
de lancement embarqué ne référence ni Gazebo, ni PX4 SITL, ni l'injecteur de
pannes. Ce test le vérifie, plutôt que de l'espérer.
"""

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
BRINGUP_DIR = REPO / "drone/ros2/dronoto_bringup/launch"
BRINGUP_DRONE = BRINGUP_DIR / "bringup_drone.launch.py"
BRINGUP_SIM = BRINGUP_DIR / "bringup_sim.launch.py"

FORBIDDEN = [
    "gz ",
    "gazebo",
    "px4_sitl",
    "run_sim.sh",
    "fault_injector",
    "sensor_faults",
    "PX4-Autopilot",
]


def _code_without_comments(path: Path) -> str:
    """Contenu du fichier, commentaires et docstring de module retirés.

    Expliquer pourquoi la simulation est absente est légitime ; la référencer
    dans le code ne l'est pas.
    """
    lines = path.read_text(encoding="utf-8").splitlines()
    out, in_docstring = [], False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('"""') or stripped.startswith("'''"):
            # Bascule sur le délimiteur ; une docstring d'une seule ligne
            # ouvre et ferme sur la même ligne.
            if stripped.count('"""') == 1 and stripped.count("'''") == 0:
                in_docstring = not in_docstring
            elif stripped.count("'''") == 1 and stripped.count('"""') == 0:
                in_docstring = not in_docstring
            continue
        if in_docstring or stripped.startswith("#"):
            continue
        out.append(line)
    return "\n".join(out).lower()


def test_bringup_drone_exists():
    assert BRINGUP_DRONE.is_file(), f"fichier introuvable : {BRINGUP_DRONE}"


@pytest.mark.parametrize("token", FORBIDDEN)
def test_bringup_drone_has_no_simulation_reference(token: str):
    code = _code_without_comments(BRINGUP_DRONE)
    assert token.lower() not in code, (
        f"bringup_drone.launch.py référence '{token}'. Ce fichier doit pouvoir "
        f"être lancé tel quel sur le Jetson : déplacer cette référence vers "
        f"bringup_sim.launch.py."
    )


def test_bringup_sim_includes_bringup_drone():
    """La simulation réutilise le lancement embarqué, elle ne le duplique pas."""
    content = BRINGUP_SIM.read_text(encoding="utf-8")
    assert "bringup_drone.launch.py" in content, (
        "bringup_sim.launch.py doit inclure bringup_drone.launch.py. "
        "Dupliquer la liste des nœuds ferait diverger simulation et matériel."
    )


def test_mission_autostart_defaults_to_false():
    """Lancer la pile ne doit jamais faire décoller un aéronef par surprise."""
    params = REPO / "drone/ros2/dronoto_bringup/config/drone_params.yaml"
    content = params.read_text(encoding="utf-8")
    assert "autostart: false" in content, (
        "mission_executive.autostart doit valoir false par défaut dans "
        "drone_params.yaml. Les scénarios de test l'activent explicitement."
    )

    launch = BRINGUP_DRONE.read_text(encoding="utf-8")
    assert 'default_value="false"' in launch, (
        "l'argument de lancement autostart doit valoir false par défaut."
    )
