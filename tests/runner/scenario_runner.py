"""Exécution d'un scénario YAML et évaluation de ses assertions.

Le DSL de P1 est volontairement restreint : chronologie, assertions
instantanées, assertions continues, assertions finales, et une action
`kill_node` pour simuler un plantage. P6 y ajoutera l'injection de pannes
complète, sans changer la structure du fichier.
"""

from __future__ import annotations

import subprocess
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
import yaml
from dronoto_msgs.msg import SafetyEvent, SafetyState, VehicleTelemetry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String

from .sim_process import SimStack, repo_root

_STATE_NAMES = {
    0: "BOOT", 1: "IDLE", 2: "PREFLIGHT", 3: "READY", 4: "ARMED", 5: "TAKEOFF",
    6: "NOMINAL", 7: "DEGRADED", 8: "HOLD", 9: "RETURNING", 10: "LANDING",
    11: "EMERGENCY_LAND", 12: "TERMINATED",
}


@dataclass
class Observation:
    """Instantané de l'état du système, échantillonné en continu."""

    t: float
    altitude_m: float
    battery: float
    armed: bool
    safety_state: str
    position: tuple[float, float, float]


class ScenarioObserver(Node):
    """Nœud ROS 2 qui observe la mission sans jamais y intervenir."""

    def __init__(self) -> None:
        super().__init__("scenario_observer")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        events_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.observations: list[Observation] = []
        self.events: list[str] = []
        self.phase: str = ""
        self._t0: float | None = None
        self._telemetry: VehicleTelemetry | None = None
        self._safety_state: int = 0

        self.create_subscription(VehicleTelemetry, "state/telemetry", self._on_telemetry, 10)
        self.create_subscription(SafetyState, "safety/state", self._on_safety, latched)
        self.create_subscription(SafetyEvent, "safety/events", self._on_event, events_qos)
        self.create_subscription(String, "mission/phase", self._on_phase, latched)
        self.create_timer(0.1, self._sample)

    def _on_telemetry(self, msg: VehicleTelemetry) -> None:
        self._telemetry = msg
        if self._t0 is None:
            self._t0 = time.monotonic()

    def _on_safety(self, msg: SafetyState) -> None:
        self._safety_state = msg.state

    def _on_event(self, msg: SafetyEvent) -> None:
        self.events.append(msg.code)

    def _on_phase(self, msg: String) -> None:
        self.phase = msg.data

    def _sample(self) -> None:
        if self._telemetry is None or self._t0 is None:
            return
        t = self._telemetry
        self.observations.append(
            Observation(
                t=time.monotonic() - self._t0,
                altitude_m=float(t.altitude_relative_m),
                battery=float(t.battery_remaining),
                armed=bool(t.armed),
                safety_state=_STATE_NAMES.get(self._safety_state, "?"),
                position=(t.pose.position.x, t.pose.position.y, t.pose.position.z),
            )
        )

    @property
    def elapsed(self) -> float:
        return 0.0 if self._t0 is None else time.monotonic() - self._t0

    def latest(self) -> Observation | None:
        return self.observations[-1] if self.observations else None


@dataclass
class ScenarioResult:
    """Résultat d'une exécution : verdict, échecs et métriques."""

    name: str
    passed: bool
    failures: list[str] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)


def _check(label: str, value: Any, constraint: Any) -> str | None:
    """Retourne un message d'échec, ou None si la contrainte est satisfaite."""
    if isinstance(constraint, dict):
        if "min" in constraint and value < constraint["min"]:
            return f"{label} = {value!r} < min {constraint['min']!r}"
        if "max" in constraint and value > constraint["max"]:
            return f"{label} = {value!r} > max {constraint['max']!r}"
        if "eq" in constraint and value != constraint["eq"]:
            return f"{label} = {value!r} != {constraint['eq']!r}"
        return None
    if value != constraint:
        return f"{label} = {value!r} != {constraint!r}"
    return None


def _write_params(scenario: dict) -> Path | None:
    """Matérialise la section 'params' du scénario en fichier de paramètres ROS 2."""
    params = scenario.get("params")
    if not params:
        return None
    tmp = Path(tempfile.mkdtemp(prefix="dronoto_scn_")) / "params.yaml"
    tmp.write_text(yaml.safe_dump(params, sort_keys=False), encoding="utf-8")
    return tmp


def _kill_node(executable: str) -> None:
    """Tue brutalement un nœud pour simuler son plantage.

    P1 n'a pas encore d'injecteur de pannes (P6) ; un SIGKILL sur le processus
    est la simulation la plus fidèle d'un plantage, et la plus simple.
    """
    subprocess.run(["pkill", "-9", "-f", executable], check=False)


def run_scenario(path: Path, domain_id: int = 0) -> ScenarioResult:
    """Exécute un scénario et retourne son résultat."""
    scenario = yaml.safe_load(path.read_text(encoding="utf-8"))
    name = scenario.get("name", path.stem)
    timeout_s = float(scenario.get("timeout_s", 300))
    failures: list[str] = []
    # Initialisé avant le try : le return final y accède même si la pile n'a
    # jamais démarré, ce qui doit produire un échec lisible et non un NameError.
    metrics: dict[str, Any] = {}

    params_file = _write_params(scenario)
    rclpy.init(args=None)
    observer = ScenarioObserver()

    stack = SimStack(
        domain_id=domain_id,
        headless=bool(scenario.get("headless", True)),
        params_file=params_file,
        autostart=True,
    )

    try:
        stack.start()

        pending = sorted(scenario.get("timeline", []), key=lambda step: float(step["at"]))
        continuous = scenario.get("assert_continuous", {})
        deadline = time.monotonic() + timeout_s + 40.0  # marge de démarrage

        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
            now = observer.elapsed
            latest = observer.latest()
            if latest is None:
                continue

            for key, constraint in continuous.items():
                msg = _check(key, getattr(latest, key), constraint)
                if msg:
                    failures.append(f"[continu t={now:.1f}s] {msg}")

            while pending and now >= float(pending[0]["at"]):
                step = pending.pop(0)
                if "kill_node" in step:
                    _kill_node(step["kill_node"])
                for key, constraint in step.get("assert", {}).items():
                    value = observer.phase if key == "phase" else getattr(latest, key)
                    msg = _check(key, value, constraint)
                    if msg:
                        failures.append(f"[t={step['at']}s] {msg}")

            if observer.phase in ("DONE", "ABORTED"):
                break

        latest = observer.latest()
        altitudes = [o.altitude_m for o in observer.observations]
        metrics = {
            "duration_s": round(observer.elapsed, 1),
            "max_altitude_m": round(max(altitudes), 2) if altitudes else 0.0,
            "final_phase": observer.phase,
            "samples": len(observer.observations),
            "events": sorted(set(observer.events)),
        }
        if latest is not None:
            metrics["final_altitude_m"] = round(latest.altitude_m, 2)
            metrics["final_position"] = [round(v, 2) for v in latest.position]
            metrics["distance_to_home_m"] = round(
                (latest.position[0] ** 2 + latest.position[1] ** 2) ** 0.5, 2
            )
            metrics["final_battery"] = round(latest.battery, 3)
            metrics["final_armed"] = latest.armed

        for key, constraint in scenario.get("final_assertions", {}).items():
            if key == "phase":
                value = observer.phase
            elif key == "events_contain":
                missing = [c for c in constraint if c not in observer.events]
                if missing:
                    failures.append(f"[final] événements absents : {missing}")
                continue
            elif key in metrics:
                value = metrics[key]
            elif latest is not None and hasattr(latest, key):
                value = getattr(latest, key)
            else:
                failures.append(f"[final] métrique inconnue : {key}")
                continue
            msg = _check(key, value, constraint)
            if msg:
                failures.append(f"[final] {msg}")

        if not observer.observations:
            failures.append("aucune télémétrie reçue : la pile n'a pas démarré")

    finally:
        stack.stop()
        observer.destroy_node()
        rclpy.shutdown()

    return ScenarioResult(name=name, passed=not failures, failures=failures, metrics=metrics)


def scenarios_dir() -> Path:
    return repo_root() / "tests" / "scenarios"
