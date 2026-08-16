"""Un seul nœud parle à PX4.

Propriété de docs/architecture/05-integration-px4.md : la connaissance des
conventions PX4 (repères NED, unités, sémantique des modes) est concentrée dans
px4_interface. Tout autre nœud qui s'abonnerait à /fmu/* la dupliquerait, et les
deux copies divergeraient.

Exception explicite et documentée : safety_supervisor lit
/fmu/out/vehicle_status. C'est délibéré — le superviseur doit observer ce que
PX4 a détecté sans passer par un intermédiaire qui pourrait lui-même défaillir.
C'est le principe de défense en profondeur de 10-failsafe.md, section 1 :
la couche de sécurité ne dépend pas de la couche qu'elle surveille.
"""

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
ROS_PACKAGES = REPO / "drone/ros2"

# None = tous les topics /fmu/* autorisés
ALLOWED: dict[str, set[str] | None] = {
    "dronoto_interface": None,
    "dronoto_safety": {"/fmu/out/vehicle_status"},
}

FMU_TOPIC = re.compile(r'"(/fmu/(?:in|out)/[a-z_0-9]+)"')


def test_only_px4_interface_talks_to_px4():
    violations = []

    for source in sorted(ROS_PACKAGES.rglob("*.cpp")):
        package = source.relative_to(ROS_PACKAGES).parts[0]
        topics = set(FMU_TOPIC.findall(source.read_text(encoding="utf-8")))
        if not topics:
            continue

        if package not in ALLOWED:
            violations.append(f"{package} ({source.name}) utilise {sorted(topics)}")
            continue

        allowed = ALLOWED[package]
        if allowed is None:
            continue
        extra = topics - allowed
        if extra:
            violations.append(f"{package} ({source.name}) utilise en trop {sorted(extra)}")

    assert not violations, (
        "Seul px4_interface doit communiquer avec PX4 :\n  "
        + "\n  ".join(violations)
        + "\n\nSi une nouvelle exception est justifiée, l'ajouter à ALLOWED "
          "ET la documenter dans docs/architecture/05-integration-px4.md."
    )


def test_px4_subscriptions_use_best_effort_qos():
    """PX4 publie en BEST_EFFORT : un abonné RELIABLE ne reçoit rien, sans erreur.

    C'est le piège le plus coûteux de l'intégration PX4/ROS 2, parce qu'il se
    manifeste par un silence total plutôt que par un message d'erreur. Ce test
    vérifie que chaque fichier qui s'abonne à /fmu/out/* utilise SensorDataQoS.
    """
    offenders = []

    for source in sorted(ROS_PACKAGES.rglob("*.cpp")):
        content = source.read_text(encoding="utf-8")
        if "/fmu/out/" not in content:
            continue
        if "SensorDataQoS" not in content:
            offenders.append(str(source.relative_to(REPO)))

    assert not offenders, (
        "Ces fichiers s'abonnent à /fmu/out/* sans utiliser rclcpp::SensorDataQoS() :\n  "
        + "\n  ".join(offenders)
        + "\n\nPX4 publie en BEST_EFFORT. Un abonné RELIABLE n'établit jamais la "
          "correspondance et ne reçoit RIEN, silencieusement."
    )
