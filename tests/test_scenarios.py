"""Exécution des scénarios SITL (niveau L3).

Lents et gourmands : marqués `scenario` pour être exclus des exécutions rapides.

    pytest -m "not scenario"     # rapide, sans simulation
    pytest -m scenario           # scénarios complets
"""

import json
from pathlib import Path

import pytest

SCENARIOS_DIR = Path(__file__).resolve().parent / "scenarios"
SCENARIOS = sorted(SCENARIOS_DIR.glob("*.yaml"))


@pytest.mark.scenario
@pytest.mark.parametrize("scenario_path", SCENARIOS, ids=lambda p: p.stem)
def test_scenario(scenario_path: Path, tmp_path: Path) -> None:
    # Import tardif : rclpy et dronoto_msgs n'existent que si l'environnement
    # ROS 2 est sourcé. conftest.py saute déjà ces tests sinon, mais un import
    # en tête de module échouerait avant même la collecte.
    from tests.runner.scenario_runner import run_scenario

    result = run_scenario(scenario_path)

    report = tmp_path / f"{scenario_path.stem}.json"
    report.write_text(
        json.dumps(
            {
                "name": result.name,
                "passed": result.passed,
                "failures": result.failures,
                "metrics": result.metrics,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    assert result.passed, (
        f"{result.name} a échoué :\n  "
        + "\n  ".join(result.failures)
        + f"\n\nMétriques : {json.dumps(result.metrics, indent=2)}"
    )
